import socket
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from ble_vofa_bridge import Forwarder, TAIL


class BridgeTest(unittest.TestCase):
    def test_camera_frames_and_legacy_layouts(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(.2)
            records = []
            bridge = Forwarder(sender, receiver.getsockname(), records.append)
            for count in (8, 20, 32, 48, 64, 72, 64, 72):
                values = [0.] * count
                values[0] = 123456.
                if count == 8:
                    values = [1, -128, 1, 4, 600, 100, 2, 1]
                if count == 20:
                    values = [1, -128, 1, 4, 600, 100, 2, 1, 2, 1, 254, 127, 1, 1, 0, -.125, 0, 500, 81, 0]
                if count == 32:
                    values = [1,-128,1,4,600,100,2,1,2,1,254,127,1,1,0,-.125,0,500,81,0] + [.125,-.125,.125,-.125,.0625,-.125,.125,-.0625,4000,-1600,1600,-4000]
                if count == 72:
                    values[64:72] = [1, -128, 1, 4, 600, 100, 2, 1]
                packet = struct.pack(f'<{count}f', *values) + TAIL
                # A different notification size also splits the signed offset bytes.
                stream = packet + packet if count == 8 else packet
                for i in range(0, len(stream), 17):
                    bridge.receive(None, stream[i:i+17])
                self.assertEqual(receiver.recv(1024), packet)
                if count == 8:
                    self.assertEqual(receiver.recv(1024), packet)
                self.assertEqual(records[-1], tuple(values))
            self.assertEqual(bridge.frames, 9)

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
            # Initial layout confirmation holds the first candidate until a
            # second frame rules out a truncated longer-format suffix.
            bridge.receive(None, packet)
            self.assertEqual(receiver.recv(1024), packet)
            self.assertEqual(receiver.recv(1024), packet)
            bridge.receive(None, packet + packet)
            self.assertEqual(receiver.recv(1024), packet)
            self.assertEqual(receiver.recv(1024), packet)
            self.assertEqual(bridge.frames, 4)
            self.assertEqual(len(records), 4)
            self.assertTrue(all(row == tuple(values) for row in records))
            with self.assertRaises(socket.timeout):
                receiver.recv(1024)
            bridge.receive(None, b'x' * 10000)
            self.assertLessEqual(len(bridge.buffer), 4096)


if __name__ == '__main__':
    unittest.main()
