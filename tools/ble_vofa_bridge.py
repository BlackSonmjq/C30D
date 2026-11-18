"""Receive FFE1 BLE notifications and forward 38-channel wheel/IMU S21C range and wheel-load, alignment, vision or legacy JustFloat frames.

No GATT writes or chassis commands are sent. start_notify enables the CCCD.
API reference: https://bleak.readthedocs.io/en/stable/api/client.html
"""
import argparse
import asyncio
import csv
import errno
from datetime import datetime
from pathlib import Path
import socket
import struct
import sys
import time

from vofa_capture import Framer, TAIL, PHASES, ERRORS, stamp, fields_for, camera_summary

SERVICE = '0000ffe0-0000-1000-8000-00805f9b34fb'
CHARACTERISTIC = '0000ffe1-0000-1000-8000-00805f9b34fb'


class Forwarder:
    def __init__(self, sock, destination, record=None):
        self.sock, self.destination = sock, destination
        self.record = record
        self.parser = Framer()
        self.buffer = self.parser.buffer
        self.bytes = self.frames = self.discarded = 0
        self.last_rx = time.monotonic()

    def receive(self, _sender, data):
        self.bytes += len(data)
        self.last_rx = time.monotonic()
        # One shared parser handles arbitrary BLE fragments and all supported layouts.
        for values in self.parser.feed(data):
            packet = struct.pack(f'<{len(values)}f', *values) + TAIL
            self.sock.sendto(packet, self.destination)
            if self.record:
                self.record(values)
            self.frames += 1
        self.discarded = self.parser.discarded


async def run(args):
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        raise SystemExit('Install dependency first: python -m pip install bleak')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        # Use a stable local source port; VOFA listens on the destination port.
        sock.bind(('127.0.0.1', args.local_port))
        while True:
            try:
                print('Scanning; disconnect phone nRF Connect / WHEELTEC APP first...', flush=True)
                if args.address:
                    device = await BleakScanner.find_device_by_address(args.address, timeout=15)
                else:
                    device = await BleakScanner.find_device_by_filter(
                        lambda d, ad: (ad.local_name or d.name or '').casefold() == args.name.casefold(),
                        timeout=15)
                if device is None:
                    print('Device not found. Check power, PC Bluetooth, and phone connection.', flush=True)
                    await asyncio.sleep(3)
                    continue
                print(f'Connecting {device.name} [{device.address}]', flush=True)
                disconnected = asyncio.Event()
                async with BleakClient(device, disconnected_callback=lambda _: disconnected.set()) as client:
                    service = client.services.get_service(SERVICE)
                    char = service.get_characteristic(CHARACTERISTIC) if service else None
                    if char is None or 'notify' not in char.properties:
                        raise RuntimeError('FFE0 / FFE1 Notify characteristic not available')
                    forwarder = Forwarder(sock, ('127.0.0.1', args.port), args.record)
                    await client.start_notify(char, forwarder.receive)
                    print(f'Subscribed FFE1 -> UDP 127.0.0.1:{args.port}; Ctrl+C to stop.', flush=True)
                    while not disconnected.is_set():
                        try:
                            await asyncio.wait_for(disconnected.wait(), timeout=5)
                        except asyncio.TimeoutError:
                            pass
                        idle = time.monotonic() - forwarder.last_rx
                        print(f'RX={forwarder.bytes} bytes frames={forwarder.frames} '
                              f'discarded={forwarder.discarded} idle={idle:.1f}s', flush=True)
                        if idle > 5:
                            print('No BLE data: check module UART 9600 and flashed firmware.', flush=True)
                        elif forwarder.bytes and not forwarder.frames:
                            print('Bytes received but no valid 8/13/20/21/31/32/33/34/35/37/38/40/48/64/72-channel frame; check UART/data loss.', flush=True)
                # Partial data from a previous connection must never join a new one.
                print('Disconnected; reconnecting...', flush=True)
            except (OSError, RuntimeError) as exc:
                print(f'Connection error: {exc}', flush=True)
            except Exception as exc:
                # Bleak backend errors differ by OS; preserve diagnostics and retry.
                print(f'BLE error ({type(exc).__name__}): {exc}', flush=True)
            await asyncio.sleep(3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='WHEELTEC-IOS')
    parser.add_argument('--address', help='Optional BLE address to select one specific robot')
    parser.add_argument('--port', type=int, default=9000, help='VOFA UDP listening port')
    parser.add_argument('--local-port', type=int, default=9001, help='Bridge UDP source port')
    parser.add_argument('--self-test', action='store_true', help='Check packaged BLE dependencies and UDP forwarding without connecting')
    parser.add_argument('--log-dir', type=Path, help='CSV folder (default: logs beside EXE/script)')
    args = parser.parse_args()
    if args.self_test:
        from bleak.backends.winrt.client import BleakClientWinRT
        from bleak.backends.winrt.scanner import BleakScannerWinRT
        assert BleakClientWinRT and BleakScannerWinRT
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(2)
            forwarder = Forwarder(sender, receiver.getsockname())
            values = [1, -20, 1, 3, 100, 3, 5, 60, 1, 0, 0, 3, 1]
            values += [0, -5, 5, -1, .125, 1, 1, 0]
            values += [.125, -.125, .25, -.25, 0, -.125, .25, 0, 16700, -1500, 1500, -16700]
            values = values[:6] + [values[8], values[7], 120] + values[9:11] + values[13:]
            values += [186, 127, 81, 200]
            values += [600, 0, 0]
            packet = struct.pack('<38f', *values) + TAIL
            stream = packet + packet
            for i in range(0, len(stream), 20):
                forwarder.receive(None, stream[i:i+20])
            assert receiver.recv(1024) == packet and receiver.recv(1024) == packet and forwarder.frames == 2
        print('PASS: Windows BLE dependencies, fragmented frame and UDP forwarding')
        return
    if not (1 <= args.port <= 65535 and 0 <= args.local_port <= 65535):
        parser.error('Invalid UDP port')
    if args.port == args.local_port:
        parser.error('VOFA and bridge must use different ports')
    try:
        base = Path(sys.executable).parent if getattr(sys, 'frozen', False) else Path(__file__).resolve().parent
        folder = args.log_dir or base / 'logs'
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / ('vision-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.csv')
        with path.open('x', newline='', encoding='utf-8-sig') as log:
            writer = csv.writer(log)
            channel_count = None
            last_status = None
            def record(values):
                nonlocal last_status, channel_count
                if channel_count is None:
                    channel_count = len(values)
                    writer.writerow(['received_at'] + fields_for(channel_count))
                elif len(values) != channel_count:
                    raise SystemExit('Telemetry layout changed; restart the bridge to create a new CSV.')
                writer.writerow([stamp()] + list(values))
                log.flush()
                if len(values) in (8, 13, 20, 21, 31, 32, 33, 34, 35, 37, 38, 40):
                    status = values[:4] + (values[8:14] + values[19:20] if len(values) in (20, 32, 37, 40) else ()) + (values[37:40] if len(values) == 40 else ())
                    if len(values) in (13, 21, 33, 34):
                        status = values[:4] + values[5:9] + values[11:13] + (values[13:] if len(values) in (21, 33, 34) else ())
                    if len(values) == 38:
                        status += values[35:]
                    if status != last_status:
                        print(camera_summary(values), flush=True)
                        last_status = status
                    return
                status = tuple(int(values[i]) for i in (1, 2, 3))
                if status != last_status:
                    point, phase, error = status
                    print(f'ROUTE P{point:02d} {PHASES[phase]} {ERRORS[error]} '
                          f'progress={values[4]:.0f}/{values[5]:.0f}mm '
                          f'laser={values[10]:.0f}/{values[11]:.0f}mm', flush=True)
                    last_status = status
            args.record = record
            print(f'CSV: {path.resolve()}', flush=True)
            asyncio.run(run(args))
    except OSError as exc:
        if exc.errno != errno.EADDRINUSE and getattr(exc, 'winerror', None) != 10048:
            raise
        print(f'UDP {args.local_port} is already in use. Close the running bridge before restarting, '
              'or choose a different --local-port.', flush=True)
        if path.stat().st_size == 0:
            path.unlink()
        if getattr(sys, 'frozen', False):
            input('Press Enter to close this window...')
    except KeyboardInterrupt:
        print('Stopped.')


if __name__ == '__main__':
    main()
