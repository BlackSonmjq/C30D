"""WHEELTEC BLE -> complete JustFloat -> localhost UDP + durable CSV.

Adapted from C30D_11.17/.../tools/ble_vofa_bridge.py.
No chassis commands or GATT characteristic writes. start_notify only enables CCCD.
"""
import argparse
import asyncio
import csv
from datetime import datetime, timedelta, timezone
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import queue
import socket
import struct
import sys
import threading
import time

from vofa_protocol import Framer, PROFILES, REASON_NAMES, STATE_NAMES, TAIL, sample_packet

SERVICE = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC = "0000ffe1-0000-1000-8000-00805f9b34fb"
TZ = timezone(timedelta(hours=8))
WINRT_MODULES = (
    "bleak.backends.winrt.client", "bleak.backends.winrt.scanner", "bleak.backends.winrt.util",
    "winrt.windows.devices.bluetooth", "winrt.windows.devices.bluetooth.advertisement",
    "winrt.windows.devices.bluetooth.genericattributeprofile", "winrt.windows.devices.enumeration",
    "winrt.windows.devices.radios", "winrt.windows.foundation",
    "winrt.windows.foundation.collections", "winrt.windows.storage.streams",
)


def stamp():
    return datetime.now(TZ).isoformat(timespec="milliseconds")


def base_directory():
    return Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent


def load_bleak():
    sys.coinit_flags = 0  # Console program: MTA; never GUI allow_sta().
    from bleak import BleakClient, BleakScanner
    if sys.platform == "win32":
        from bleak.backends.winrt.util import uninitialize_sta
        uninitialize_sta()
    return BleakClient, BleakScanner


def client_options(device):
    if sys.platform != "win32":
        return {"services": [SERVICE]}

    # Read current GATT handles after a module swap. Pass the address type
    # from the advertisement so WinRT can resolve random BLE addresses.
    winrt = {"use_cached_services": False}
    details = getattr(device, "details", None)
    advertisement = getattr(details, "adv", None) or getattr(details, "scan", None)
    address_type = getattr(getattr(advertisement, "bluetooth_address_type", None), "name", "").lower()
    if address_type in ("public", "random"):
        winrt["address_type"] = address_type
    return {"winrt": winrt}


def bind_source(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        sock.bind(("127.0.0.1", port))
        return sock
    except OSError as exc:
        sock.close()
        raise OSError(f"无法绑定本机 UDP 源端口 {port}：{exc}。请关闭其他转发器，或修改 --local-port。") from exc


class Statistics:
    def __init__(self):
        self.lock = threading.Lock()
        self.values = dict(rx_bytes=0, frames=0, discarded=0, udp_errors=0, queue_dropped=0)
        self.last_rx = time.monotonic()

    def add(self, **amounts):
        with self.lock:
            for name, amount in amounts.items():
                self.values[name] += amount
            if amounts.get("rx_bytes"):
                self.last_rx = time.monotonic()

    def snapshot(self):
        with self.lock:
            return {**self.values, "idle_seconds": time.monotonic()-self.last_rx}


class Forwarder:
    """Same receive/record interface as the previous bridge, plus explicit profiles."""
    def __init__(self, sock, destination, record=None, profile="nav48", stats=None):
        self.sock, self.destination, self.record = sock, destination, record
        self.parser = Framer(profile)
        self.profile = self.parser.profile
        self.stats = stats or Statistics()
        self.last_frame_time = None
        self.last_status = None

    @property
    def buffer(self):
        return self.parser.buffer

    @property
    def frames(self):
        return self.stats.snapshot()["frames"]

    @property
    def discarded(self):
        return self.stats.snapshot()["discarded"]

    @property
    def bytes(self):
        return self.stats.snapshot()["rx_bytes"]

    def receive(self, _sender, data, received_at=None, count_received=True):
        if count_received:
            self.stats.add(rx_bytes=len(data))
        before = self.parser.discarded
        try:
            for packet, values in self.parser.feed(data):
                self.last_frame_time = received_at or stamp()
                self.stats.add(frames=1)
                try:
                    self.sock.sendto(packet, self.destination)
                except OSError as exc:
                    self.stats.add(udp_errors=1)
                    print(f"UDP 转发失败（CSV继续记录）：{exc}", flush=True)
                if self.record:
                    self.record(values)
                self._status(values)
        finally:
            self.stats.add(discarded=self.parser.discarded-before)

    def _status(self, values):
        if self.profile.name == "nav48":
            key = tuple(int(values[i]) for i in (15, 16, 17, 18, 13, 38, 41, 42, 43))
            if key != self.last_status:
                state, reason, target, confirmed, mask, allowed, imu, fresh, paused = key
                directions = "/".join(name for i, name in enumerate(("前", "后", "左", "右")) if mask & (1 << i)) or "无"
                print(f"状态：{STATE_NAMES[state]}；原因：{REASON_NAMES[reason]}；"
                      f"P{confirmed:02d}→P{target:02d}；参考={directions}；"
                      f"电机允许={allowed} IMU={imu} CAN={fresh} 机构停稳={paused}", flush=True)
                self.last_status = key
        elif self.profile.name == "align20":
            key = tuple(values[i] for i in (8, 9, 13, 19))
            if key != self.last_status:
                print(f"对中状态：phase={key[0]:g} locked={key[1]:g} ready={key[2]:g} blocked={key[3]:g}", flush=True)
                self.last_status = key

    def reset(self):
        dropped = self.parser.reset()
        self.stats.add(discarded=dropped)


class CsvRecorder:
    def __init__(self, folder, profile="nav48", prefix="telemetry"):
        self.profile = PROFILES[profile]
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        name = f"{prefix}-{profile}-{datetime.now(TZ).strftime('%Y%m%d-%H%M%S-%f')}.csv"
        self.path = folder / name
        self.file = self.path.open("x", newline="", encoding="utf-8-sig")
        self.writer = csv.writer(self.file)
        try:
            self.writer.writerow(["received_at", "connection_id", *self.profile.fields])
            self._sync()
        except Exception:
            self.file.close()
            raise

    def _sync(self):
        self.file.flush()
        os.fsync(self.file.fileno())

    def record(self, values, received_at, connection_id):
        if len(values) != self.profile.channels:
            raise ValueError("CSV channel count changed within the same run")
        self.writer.writerow([received_at, connection_id, *values])
        self._sync()

    def close(self):
        self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, *unused):
        self.close()


class NotificationPipeline:
    """Keep BLE callbacks fast. One worker owns framing, UDP and CSV per connection."""
    _STOP = object()

    def __init__(self, forwarder, recorder, connection_id, max_queue=512):
        self.forwarder, self.recorder, self.connection_id = forwarder, recorder, connection_id
        self.queue = queue.Queue(maxsize=max_queue)
        self.generation = 0
        self.closed = False
        self.callback_lock = threading.Lock()
        self.error = None
        forwarder.record = lambda values: recorder.record(values, forwarder.last_frame_time, connection_id)
        self.worker = threading.Thread(target=self._consume, name="vofa-csv", daemon=True)
        self.worker.start()

    def notify(self, _sender, data):
        with self.callback_lock:
            if self.closed:
                return
            block = bytes(data)
            self.forwarder.stats.add(rx_bytes=len(block))
            try:
                self.queue.put_nowait((self.generation, block, stamp()))
            except queue.Full:
                self.generation += 1
                self.forwarder.stats.add(discarded=len(block), queue_dropped=len(block))

    def _consume(self):
        generation = 0
        try:
            while True:
                item = self.queue.get()
                if item is self._STOP:
                    break
                current, data, received_at = item
                if current != generation:
                    self.forwarder.reset()
                    generation = current
                self.forwarder.receive(None, data, received_at, count_received=False)
        except Exception as exc:
            self.error = exc
        finally:
            self.forwarder.reset()

    def close(self):
        with self.callback_lock:
            if self.closed:
                return
            self.closed = True
        while self.worker.is_alive():
            try:
                self.queue.put(self._STOP, timeout=0.05)
                break
            except queue.Full:
                pass
        self.worker.join()


async def run(args, recorder, sock, *, client_type=None, scanner_type=None, max_connections=None):
    if client_type is None:
        client_type, scanner_type = load_bleak()
    stats = Statistics()
    connection_id = 0
    loop = asyncio.get_running_loop()
    while max_connections is None or connection_id < max_connections:
        pipeline = None
        try:
            print(f"扫描蓝牙：{args.address or args.name}。请先断开手机 APP 的连接。", flush=True)
            if args.address:
                device = await scanner_type.find_device_by_address(args.address, timeout=args.scan_timeout)
            else:
                device = await scanner_type.find_device_by_filter(
                    lambda d, ad: (ad.local_name or d.name or "").casefold() == args.name.casefold(),
                    timeout=args.scan_timeout)
            if device is None:
                print("未找到设备：检查小车供电、电脑蓝牙，以及手机/其他电脑是否仍占用连接。", flush=True)
                await asyncio.sleep(args.retry_delay)
                continue
            print(f"连接：{device.name} [{device.address}]", flush=True)
            disconnected = asyncio.Event()
            def on_disconnect(_client, event=disconnected):
                # A late callback from an old client must not end a new connection.
                if not loop.is_closed():
                    loop.call_soon_threadsafe(event.set)
            async with client_type(device, disconnected_callback=on_disconnect,
                                   timeout=args.connect_timeout, **client_options(device)) as client:
                service = client.services.get_service(SERVICE)
                char = service.get_characteristic(CHARACTERISTIC) if service else None
                if char is None or "notify" not in char.properties:
                    raise RuntimeError("未找到 FFE0 / FFE1 Notify 特征，请检查模块型号或服务配置。")
                connection_id += 1
                forwarder = Forwarder(sock, ("127.0.0.1", args.port), profile=args.profile, stats=stats)
                connection_start = stats.snapshot()
                pipeline = NotificationPipeline(forwarder, recorder, connection_id)
                await client.start_notify(char, pipeline.notify)
                print(f"已订阅：{args.profile}，{PROFILES[args.profile].channels}通道 → "
                      f"127.0.0.1:{args.port}；源端口 {sock.getsockname()[1]}；Ctrl+C 退出。", flush=True)
                while not disconnected.is_set() and client.is_connected:
                    try:
                        await asyncio.wait_for(disconnected.wait(), timeout=args.status_interval)
                    except asyncio.TimeoutError:
                        pass
                    if pipeline.error:
                        raise OSError(f"遥测记录失败：{pipeline.error}") from pipeline.error
                    s = stats.snapshot()
                    print(f"RX={s['rx_bytes']} bytes  有效帧={s['frames']}  丢弃={s['discarded']} bytes  "
                          f"UDP错误={s['udp_errors']}  队列丢弃={s['queue_dropped']}  空闲={s['idle_seconds']:.1f}s", flush=True)
                    if s["idle_seconds"] > 5:
                        print("没有新数据：检查已烧录蓝牙遥测固件、模块 UART 9600/8N1 和 TX 接线。", flush=True)
                    elif s["rx_bytes"]>connection_start["rx_bytes"] and s["frames"]==connection_start["frames"]:
                        print("收到字节但没有合法帧：核对 --profile、固件版本、波特率和串口是否混入文本。", flush=True)
        except asyncio.CancelledError:
            raise
        except OSError as exc:
            # CSV/UDP source/file failures are actionable; do not silently loop.
            if pipeline and pipeline.error:
                raise
            print(f"蓝牙连接发生系统错误：{exc}。检查蓝牙开关、驱动、供电及手机是否占用；稍后重试。", flush=True)
        except Exception as exc:
            print(f"蓝牙连接失败（{type(exc).__name__}）：{exc}。"
                  "请断开手机 APP，检查蓝牙与供电；稍后重试。", flush=True)
        finally:
            if pipeline:
                await asyncio.to_thread(pipeline.close)
                if pipeline.error:
                    raise OSError(f"CSV/遥测处理失败：{pipeline.error}") from pipeline.error
        print("连接已结束，已清空本连接残留数据。", flush=True)
        if max_connections is not None and connection_id >= max_connections:
            break
        await asyncio.sleep(args.retry_delay)


def check_dependencies():
    if sys.platform != "win32" or struct.calcsize("P") != 8:
        raise RuntimeError("此 EXE 的 BLE 后端要求 Windows x64。")
    load_bleak()
    modules = [importlib.import_module(name) for name in WINRT_MODULES]
    # Exercise native WinRT activation and buffers without touching the radio.
    from winrt.windows.storage.streams import DataReader, DataWriter
    writer = DataWriter()
    reader = None
    try:
        writer.write_bytes(b"WinRT self-test")
        reader = DataReader.from_buffer(writer.detach_buffer())
        copied = bytearray(reader.unconsumed_buffer_length)
        reader.read_bytes(copied)
        if copied != b"WinRT self-test":
            raise RuntimeError("WinRT native buffer roundtrip failed")
    finally:
        if reader is not None:
            reader.close()
        writer.close()
    if getattr(sys, "frozen", False):
        bundle = Path(sys._MEIPASS).resolve()
        for module in modules:
            location = getattr(module, "__file__", None)
            if location and not Path(location).resolve().is_relative_to(bundle):
                raise RuntimeError(f"依赖从 EXE 外部加载：{module.__name__}")
    return {"python": sys.version.split()[0], "bleak": importlib.metadata.version("bleak"),
            "winrt_modules": len(modules), "winrt_native_buffer": True,
            "frozen": bool(getattr(sys, "frozen", False))}


def self_test(args):
    deps = check_dependencies()
    profile = PROFILES[args.profile]
    fixture = Path(__file__).resolve().parent / "testdata" / "nav48-frame.bin"
    packet = fixture.read_bytes() if args.profile == "nav48" and fixture.is_file() else sample_packet(args.profile)
    if profile.decode(packet[:-4]) is None or packet[-4:] != TAIL:
        raise RuntimeError("内置固件帧样本与所选协议不匹配")
    folder = args.log_dir or base_directory() / "logs"
    with CsvRecorder(folder, args.profile, "selftest") as recorder, \
            bind_source(0) as receiver, bind_source(0) as sender:
        receiver.settimeout(2)
        bridge = Forwarder(sender, receiver.getsockname(), profile=args.profile)
        bridge.record = lambda values: recorder.record(values, bridge.last_frame_time, 0)
        bridge.receive(None, b"noise" + packet[100:])
        for i in range(0, len(packet), 7):
            bridge.receive(None, packet[i:i+7])
        bridge.receive(None, packet + packet)
        for _ in range(3):
            if receiver.recv(65535) != packet:
                raise RuntimeError("UDP 字节与完整帧不一致")
        if bridge.frames != 3:
            raise RuntimeError("自测帧数错误")
        path = recorder.path
    with path.open(encoding="utf-8-sig", newline="") as csvfile:
        rows = list(csv.reader(csvfile))
    if len(rows) != 4 or len(rows[0]) != profile.channels+2:
        raise RuntimeError("CSV 行数或字段数量错误")
    print("PASS：WinRT依赖、分片/连续帧/噪声恢复、完整帧UDP转发、逐帧CSV同步。")
    print(f"自测CSV（模拟数据）：{path.resolve()}")
    print("UDP自测使用临时回环端口，未扫描蓝牙或连接实车。")
    print(json.dumps({**deps, "profile": args.profile, "frames": 3, "csv": str(path.resolve())}, ensure_ascii=False))


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="WHEELTEC-IOS")
    parser.add_argument("--address")
    parser.add_argument("--port", type=int, default=9000, help="VOFA UDP destination port")
    parser.add_argument("--local-port", type=int, default=9001, help="UDP source port; 0 selects a free port")
    layouts = parser.add_mutually_exclusive_group()
    layouts.add_argument("--profile", choices=tuple(PROFILES), default="nav48")
    layouts.add_argument("--channels", type=int, choices=(8, 20, 48, 64, 72, 80, 104, 112, 128),
                        help="Compatibility: 48=nav48, 20=align20, others=legacyN; use --profile legacy48 for old route data")
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--scan-timeout", type=float, default=15)
    parser.add_argument("--connect-timeout", type=float, default=20)
    parser.add_argument("--retry-delay", type=float, default=3)
    parser.add_argument("--status-interval", type=float, default=3)
    return parser


def main(argv=None):
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    parser = make_parser()
    args = parser.parse_args(argv)
    if args.channels:
        args.profile = "nav48" if args.channels == 48 else "align20" if args.channels == 20 else f"legacy{args.channels}"
    if not (1 <= args.port <= 65535 and 0 <= args.local_port <= 65535) or args.port == args.local_port:
        parser.error("UDP端口范围错误，或本地源端口与VOFA监听端口相同。")
    if min(args.scan_timeout, args.connect_timeout, args.status_interval) <= 0 or args.retry_delay < 0:
        parser.error("超时必须大于0，重试间隔不能小于0。")
    try:
        if args.self_test:
            self_test(args)
            return 0
        load_bleak()  # Fail before opening the run log if dependencies are missing.
        with bind_source(args.local_port) as sock, \
                CsvRecorder(args.log_dir or base_directory() / "logs", args.profile) as recorder:
            print(f"CSV：{recorder.path.resolve()}", flush=True)
            print("仅接收遥测，不发送底盘控制命令。", flush=True)
            asyncio.run(run(args, recorder, sock))
    except KeyboardInterrupt:
        print("已停止，蓝牙、UDP和CSV资源已释放。")
        return 0
    except Exception as exc:
        print(f"错误（{type(exc).__name__}）：{exc}", file=sys.stderr, flush=True)
        if getattr(sys, "frozen", False) and len(sys.argv) == 1 and sys.stdin.isatty():
            input("按回车退出……")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
