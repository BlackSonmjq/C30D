"""Receive FFE1 BLE notifications and forward complete 64-channel JustFloat frames.

No GATT writes or chassis commands are sent. start_notify enables the CCCD.
API reference: https://bleak.readthedocs.io/en/stable/api/client.html
"""
import argparse
import asyncio
import socket
import struct
import time

from vofa_capture import Framer, TAIL

SERVICE = '0000ffe0-0000-1000-8000-00805f9b34fb'
CHARACTERISTIC = '0000ffe1-0000-1000-8000-00805f9b34fb'


class Forwarder:
    def __init__(self, sock, destination):
        self.sock, self.destination = sock, destination
        self.buffer = bytearray()
        self.bytes = self.frames = self.discarded = 0
        self.last_rx = time.monotonic()

    def receive(self, _sender, data):
        self.bytes += len(data)
        self.last_rx = time.monotonic()
        self.buffer.extend(data)
        cursor = 0
        while True:
            tail = self.buffer.find(TAIL, cursor)
            if tail < 0:
                break
            # BLE fragments have no relation to UART frame boundaries. Validate
            # exactly 64 floats before forwarding one complete UDP datagram.
            if tail >= 256 and Framer.decode(self.buffer[tail - 256:tail]) is not None:
                packet = bytes(self.buffer[tail - 256:tail + 4])
                self.sock.sendto(packet, self.destination)
                self.frames += 1
                self.discarded += tail - 256
                del self.buffer[:tail + 4]
                cursor = 0
            else:
                cursor = tail + 1
        if len(self.buffer) > 8192:
            self.discarded += len(self.buffer) - 4096
            del self.buffer[:-4096]


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
                    forwarder = Forwarder(sock, ('127.0.0.1', args.port))
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
                            print('Bytes received but no valid 64-channel frame; check UART/data loss.', flush=True)
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
    args = parser.parse_args()
    if args.self_test:
        from bleak.backends.winrt.client import BleakClientWinRT
        from bleak.backends.winrt.scanner import BleakScannerWinRT
        assert BleakClientWinRT and BleakScannerWinRT
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(2)
            forwarder = Forwarder(sender, receiver.getsockname())
            packet = struct.pack('<64f', *([0.] * 64)) + TAIL
            for i in range(0, len(packet), 20):
                forwarder.receive(None, packet[i:i+20])
            assert receiver.recv(1024) == packet and forwarder.frames == 1
        print('PASS: Windows BLE dependencies, fragmented frame and UDP forwarding')
        return
    if not (1 <= args.port <= 65535 and 0 <= args.local_port <= 65535):
        parser.error('Invalid UDP port')
    if args.port == args.local_port:
        parser.error('VOFA and bridge must use different ports')
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print('Stopped.')


if __name__ == '__main__':
    main()
