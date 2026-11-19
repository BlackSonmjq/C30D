import argparse
import asyncio
import contextlib
import csv
import io
import math
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from ble_vofa_bridge import (CsvRecorder, Forwarder, NotificationPipeline, Statistics,
                             bind_source, client_options, make_parser, run)
from vofa_protocol import Framer, NAV_FIELDS, PROFILES, TAIL, sample_packet


class FakeSocket:
    def __init__(self):
        self.packets = []
    def sendto(self, packet, destination):
        self.packets.append((packet, destination))
    def getsockname(self):
        return ("127.0.0.1", 9001)


class MemoryRecorder:
    def __init__(self):
        self.rows = []
    def record(self, values, at, connection):
        self.rows.append((values, at, connection))


class FramingTests(unittest.TestCase):
    def test_fragment_sizes_and_continuous_frames(self):
        packet = sample_packet()
        for size in (1, 2, 3, 7, 20, 84, 196, 400):
            framer = Framer()
            frames = []
            for i in range(0, len(packet)*3, size):
                frames.extend(framer.feed((packet*3)[i:i+size]))
            self.assertEqual([p for p, _ in frames], [packet]*3)
            self.assertEqual(framer.discarded, 0)

    def test_noise_lost_fragment_and_recovery(self):
        packet = sample_packet()
        framer = Framer()
        frames = list(framer.feed(b"noise" + packet[100:] + packet[:60] + packet[80:] + packet))
        self.assertEqual([p for p, _ in frames], [packet])
        self.assertGreater(framer.discarded, 0)
        framer.feed(b"x"*20000)
        self.assertLessEqual(len(framer.buffer), 4096)

    def test_false_tail_inside_finite_payload(self):
        values = list(struct.unpack("<48f", sample_packet()[:-4]))
        values[22] = struct.unpack("<f", b"\x11\x00\x00\x80")[0]
        values[23] = struct.unpack("<f", b"\x7f\x00\x00\x3f")[0]
        self.assertTrue(math.isfinite(values[22]) and math.isfinite(values[23]))
        packet = struct.pack("<48f", *values) + TAIL
        self.assertIn(TAIL, packet[:-4])
        frames = list(Framer().feed(b"n"*120 + packet))
        self.assertEqual([p for p, _ in frames], [packet])

    def test_explicit_legacy_profiles(self):
        for name in PROFILES:
            packet = sample_packet(name)
            frames = list(Framer(name).feed(b"noise"+packet+packet))
            self.assertEqual([p for p, _ in frames], [packet, packet], name)
        self.assertIsNone(PROFILES["nav48"].decode(sample_packet("align20")[:-4]))
        bad = bytearray(sample_packet())
        bad[15*4:16*4] = struct.pack("<f", 99)
        self.assertFalse(list(Framer().feed(bad)))

    def test_cross_connection_reset(self):
        packet = sample_packet()
        f = Framer()
        self.assertFalse(list(f.feed(packet[:80])))
        self.assertEqual(f.reset(), 80)
        self.assertFalse(list(f.feed(packet[80:])))
        self.assertEqual([p for p, _ in f.feed(packet)], [packet])

    def test_invalid_laser_remains_visible(self):
        values = list(struct.unpack("<48f", sample_packet()[:-4]))
        values[0] = 65535
        values[12] = 14
        values[13] = 12
        packet = struct.pack("<48f", *values) + TAIL
        self.assertEqual(list(Framer().feed(packet))[0][1][0], 65535)

    def test_real_udp_one_frame_one_datagram(self):
        packet = sample_packet()
        with bind_source(0) as receiver, bind_source(0) as sender:
            receiver.settimeout(0.15)
            records = []
            bridge = Forwarder(sender, receiver.getsockname(), records.append)
            for part in (packet[:2], packet[2:31], packet[31:] + packet):
                bridge.receive(None, part)
            self.assertEqual(receiver.recv(65535), packet)
            self.assertEqual(receiver.recv(65535), packet)
            with self.assertRaises(socket.timeout):
                receiver.recv(65535)
            self.assertEqual(len(records), 2)
            self.assertEqual(bridge.frames, 2)

    def test_source_port_conflict_and_release(self):
        first = bind_source(0)
        port = first.getsockname()[1]
        try:
            with self.assertRaises(OSError):
                bind_source(port)
        finally:
            first.close()
        with bind_source(port):
            pass

    def test_c_firmware_fixture_mapping(self):
        packet = (ROOT / "tools/testdata/nav48-frame.bin").read_bytes()
        values = PROFILES["nav48"].decode(packet[:-4])
        self.assertIsNotNone(values)
        expected = {
            "front_raw_mm":100, "back_raw_mm":120, "left_raw_mm":300, "right_raw_mm":2500,
            "back_target_mm":0, "lidar_reference_mask":13, "main_error_mm":12.5,
            "cross_error_mm":-6.25, "command_vy_mps":-0.03, "wheel_target_d_mps":0.23,
            "wheel_actual_b_mps":0.19, "pwm_a":100, "pwm_b":-200, "pwm_c":300, "pwm_d":-400,
            "motor_allowed":1, "hardware_enable":1, "software_stop":0, "start_front_mm":4800,
            "lidar_sequence":5, "telemetry_sequence":1, "mcu_time_ms":1234,
        }
        row = dict(zip(NAV_FIELDS, values))
        for name, value in expected.items():
            self.assertAlmostEqual(row[name], value, places=5, msg=name)


class RecordingTests(unittest.TestCase):
    def test_overflow_resets_before_accepting_new_generation(self):
        packet = sample_packet()
        entered, release, partial_done = (threading.Event() for _ in range(3))
        class SlowRecorder(MemoryRecorder):
            def record(self, *args):
                super().record(*args)
                if len(self.rows) == 1:
                    entered.set()
                    if not release.wait(2):
                        raise RuntimeError("Test did not release disk writer")
        class ObservedForwarder(Forwarder):
            calls = 0
            def receive(self, *args, **kwargs):
                super().receive(*args, **kwargs)
                self.calls += 1
                if self.calls == 2:
                    partial_done.set()
        recorder, sock = SlowRecorder(), FakeSocket()
        bridge = ObservedForwarder(sock, ("127.0.0.1", 9000))
        pipeline = NotificationPipeline(bridge, recorder, 1, max_queue=1)
        try:
            pipeline.notify(None, packet)
            self.assertTrue(entered.wait(1))  # Worker is held at the first disk write.
            pipeline.notify(None, packet[:80])
            pipeline.notify(None, b"gap")  # Exactly one queued fragment: this is dropped.
            release.set()
            self.assertTrue(partial_done.wait(1))
            pipeline.notify(None, packet[80:]+packet)
        finally:
            release.set()
            pipeline.close()
        self.assertIsNone(pipeline.error)
        self.assertEqual([p for p, _ in sock.packets], [packet, packet])
        self.assertEqual(bridge.stats.snapshot()["queue_dropped"], 3)
        self.assertEqual(bridge.discarded, 3+len(packet))

    def test_udp_failure_still_records_csv(self):
        class BrokenSocket(FakeSocket):
            def sendto(self, *args):
                raise OSError("test destination failure")
        records = []
        bridge = Forwarder(BrokenSocket(), ("127.0.0.1", 9000), records.append)
        with contextlib.redirect_stdout(io.StringIO()):
            bridge.receive(None, sample_packet())
        self.assertEqual(len(records), 1)
        self.assertEqual(bridge.stats.snapshot()["udp_errors"], 1)

    def test_unique_csv_and_per_frame_fsync(self):
        values = struct.unpack("<48f", sample_packet()[:-4])
        with tempfile.TemporaryDirectory() as folder, mock.patch("ble_vofa_bridge.os.fsync") as fsync:
            with CsvRecorder(folder) as a, CsvRecorder(folder) as b:
                self.assertNotEqual(a.path, b.path)
                a.record(values, "2026-11-18T12:00:00.000+08:00", 1)
                self.assertEqual(fsync.call_count, 3)  # Two headers and one data row.
                with a.path.open(encoding="utf-8-sig", newline="") as file:
                    rows = list(csv.reader(file))
                self.assertEqual(rows[0][2:], list(NAV_FIELDS))
                self.assertEqual(len(rows), 2)

    def test_worker_owns_io_and_drops_residual_on_close(self):
        packet = sample_packet()
        sock, recorder = FakeSocket(), MemoryRecorder()
        bridge = Forwarder(sock, ("127.0.0.1",9000))
        pipeline = NotificationPipeline(bridge, recorder, 7)
        pipeline.notify(None, packet + packet[:10])
        pipeline.close()
        self.assertFalse(pipeline.worker.is_alive())
        self.assertEqual(len(sock.packets), 1)
        self.assertEqual(recorder.rows[0][2], 7)
        self.assertEqual(bridge.discarded, 10)
        self.assertFalse(bridge.buffer)
        pipeline.notify(None, packet)  # Late callback from a closed connection is ignored.
        self.assertEqual(len(sock.packets), 1)

    def test_csv_failure_does_not_kill_ble_callback(self):
        class BrokenRecorder:
            def record(self, *args):
                raise OSError("disk full")
        pipeline = NotificationPipeline(Forwarder(FakeSocket(),("127.0.0.1",9000)), BrokenRecorder(), 1)
        pipeline.notify(None, sample_packet())
        pipeline.close()
        self.assertIsInstance(pipeline.error, OSError)
        self.assertFalse(pipeline.worker.is_alive())


class ConnectionOptionsTests(unittest.TestCase):
    def test_winrt_reads_fresh_gatt_and_uses_the_advertised_address_type(self):
        advertisement = SimpleNamespace(bluetooth_address_type=SimpleNamespace(name="RANDOM"))
        device = SimpleNamespace(details=SimpleNamespace(adv=advertisement, scan=None))
        with mock.patch("ble_vofa_bridge.sys.platform", "win32"):
            self.assertEqual(client_options(device), {
                "winrt": {"use_cached_services": False, "address_type": "random"},
            })


class ConnectionTests(unittest.IsolatedAsyncioTestCase):
    def args(self):
        args = make_parser().parse_args([])
        args.retry_delay = 0
        args.status_interval = 0.01
        return args

    async def test_reconnect_without_cross_frame_or_writes(self):
        packet = sample_packet()
        clients = []
        class Scanner:
            @staticmethod
            async def find_device_by_filter(predicate, timeout):
                device = SimpleNamespace(name="WHEELTEC-IOS", address="AA:BB:CC:DD:EE:FF")
                assert predicate(device, SimpleNamespace(local_name=device.name))
                return device
        class Client:
            def __init__(self, device, disconnected_callback, **kwargs):
                self.callback=disconnected_callback
                self.is_connected=True
                self.closed=False
                self.orderly_disconnect=False
                self.timer=None
                clients.append(self)
                char=SimpleNamespace(properties=["notify"])
                self.services=SimpleNamespace(get_service=lambda uuid: SimpleNamespace(get_characteristic=lambda cid: char))
            async def __aenter__(self):
                return self
            async def __aexit__(self, *args):
                self.closed=True
                if self.timer:
                    self.timer.cancel()
            async def start_notify(self, char, callback):
                callback(None, packet[:80] if len(clients)==1 else packet[80:]+packet)
                def disconnect():
                    self.orderly_disconnect=True
                    self.is_connected=False
                    self.callback(self)
                if len(clients)==1:
                    disconnect()
                else:
                    clients[0].callback(clients[0])  # Late event from the previous connection.
                    await asyncio.sleep(0)
                    self.timer=asyncio.get_running_loop().call_later(0.03,disconnect)
            async def write_gatt_char(self, *args):
                raise AssertionError("Never send control writes")
        sock,recorder=FakeSocket(),MemoryRecorder()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            await run(self.args(),recorder,sock,client_type=Client,scanner_type=Scanner,max_connections=2)
        self.assertEqual(output.getvalue().count("已订阅"), 2)
        self.assertNotIn("连接失败", output.getvalue())
        self.assertNotIn("系统错误", output.getvalue())
        self.assertEqual([p for p, _ in sock.packets], [packet])
        self.assertEqual([r[2] for r in recorder.rows], [2])
        self.assertTrue(all(c.closed for c in clients))
        self.assertTrue(all(c.orderly_disconnect for c in clients))

    async def test_cancellation_disconnects_and_closes_worker(self):
        started = asyncio.Event()
        clients=[]
        class Scanner:
            @staticmethod
            async def find_device_by_filter(predicate, timeout):
                return SimpleNamespace(name="WHEELTEC-IOS", address="AA")
        class Client:
            def __init__(self,*args,**kwargs):
                self.is_connected=True
                self.closed=False
                clients.append(self)
                char=SimpleNamespace(properties=["notify"])
                self.services=SimpleNamespace(get_service=lambda _:SimpleNamespace(get_characteristic=lambda _:char))
            async def __aenter__(self): return self
            async def __aexit__(self,*args): self.closed=True
            async def start_notify(self,*args): started.set()
        task=asyncio.create_task(run(self.args(),MemoryRecorder(),FakeSocket(),client_type=Client,scanner_type=Scanner))
        await asyncio.wait_for(started.wait(),1)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        self.assertTrue(clients[0].closed)
        self.assertFalse(any(t.name=="vofa-csv" and t.is_alive() for t in threading.enumerate()))


if __name__ == "__main__":
    unittest.main(verbosity=2)
