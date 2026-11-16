import socket
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from ble_vofa_bridge import Forwarder, TAIL


class BridgeTest(unittest.TestCase):
    def test_fragments_noise_loss_and_udp(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(.2)
            records = []
            bridge = Forwarder(sender, receiver.getsockname(), records.append)
            values = [0.] * 64
            values[0] = 123456.
            packet = struct.pack('<64f', *values) + TAIL
            # Noise, a truncated frame, arbitrary 20-byte BLE notifications,
            # and multiple complete frames in one callback must resynchronize.
            bridge.receive(None, b'noise' + packet[100:])
            for i in range(0, len(packet), 20):
                bridge.receive(None, packet[i:i+20])
            self.assertEqual(receiver.recv(1024), packet)
            bridge.receive(None, packet + packet)
            self.assertEqual(receiver.recv(1024), packet)
            self.assertEqual(receiver.recv(1024), packet)
            self.assertEqual(bridge.frames, 3)
            self.assertEqual(len(records), 3)
            self.assertTrue(all(row == tuple(values) for row in records))
            with self.assertRaises(socket.timeout):
                receiver.recv(1024)
            bridge.receive(None, b'x' * 10000)
            self.assertLessEqual(len(bridge.buffer), 4096)


if __name__ == '__main__':
    unittest.main()
