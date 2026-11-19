import socket
import contextlib
import io
import tempfile
import types
from unittest import mock
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from ble_vofa_bridge import Forwarder, TAIL
import ble_vofa_bridge as bridge_module


class BridgeTest(unittest.TestCase):
    def test_merged_navigation_channels_forward_and_record(self):
        values = [0.] * 58
        values[:9] = [1, -2, 1, 2, 100, 2, 1, 45, 120]
        values[16:19] = [1, 1, 0]
        values[31:35] = [127, 127, 89, 480]
        values[38:43] = [0, 2, 1, 2, 12]
        values[43:] = [100, 2700, 20, 2, 0, 5, 4, -50, 12, .125, -.25, 15, 0, 12000, 15]
        packet = struct.pack('<58f', *values) + TAIL
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(('127.0.0.1', 0)); receiver.settimeout(.2)
            records = []; forwarder = Forwarder(sender, receiver.getsockname(), records.append)
            stream = packet + packet
            for offset in range(0, len(stream), 20):
                forwarder.receive(None, stream[offset:offset+20])
            self.assertEqual(receiver.recv(1024), packet)
            self.assertEqual(receiver.recv(1024), packet)
            self.assertEqual(records, [tuple(values)] * 2)

    def test_busy_port_exits_cleanly_without_empty_log(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as owner, tempfile.TemporaryDirectory() as folder:
            owner.bind(('127.0.0.1', 0))
            output=io.StringIO()
            arguments=['bridge', '--local-port', str(owner.getsockname()[1]), '--log-dir', folder]
            with mock.patch.object(sys, 'argv', arguments), mock.patch.object(sys, 'frozen', True, create=True), \
                    mock.patch.dict(sys.modules, {'bleak':types.SimpleNamespace(BleakClient=object,BleakScanner=object)}), \
                    mock.patch('builtins.input') as prompt, contextlib.redirect_stdout(output):
                bridge_module.main()
            self.assertIn('already in use',output.getvalue())
            self.assertNotIn('Traceback',output.getvalue())
            prompt.assert_called_once()
            self.assertEqual(list(Path(folder).glob('*.csv')),[])
            self.assertNotEqual(owner.fileno(),-1)

    def test_lock_channels_forward_and_record(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(('127.0.0.1',0));receiver.settimeout(.2)
            records=[];forwarder=Forwarder(sender,receiver.getsockname(),records.append)
            values=[0.0]*43
            values[0:6]=[1,-2,1,0,100,3]
            values[6:9]=[1,50,120]
            values[16:19]=[1,1,0]
            values[31:35]=[127,127,81,480]
            values[35:43]=[600,0,0,2,1,3,4,12]
            data=struct.pack('<43f',*values)+TAIL
            for start in range(0,len(data+data),17):forwarder.receive(None,(data+data)[start:start+17])
            self.assertEqual(receiver.recv(1024),data)
            self.assertEqual(receiver.recv(1024),data)
            self.assertEqual(records,[tuple(values)]*2)
            self.assertEqual(forwarder.frames,2)

    def test_camera_frames_and_legacy_layouts(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(.2)
            records = []
            bridge = Forwarder(sender, receiver.getsockname(), records.append)
            for count in (8, 13, 20, 21, 32, 33, 34, 37, 40, 48, 64, 72, 64, 72):
                values = [0.] * count
                values[0] = 123456.
                if count == 8:
                    values = [1, -128, 1, 4, 600, 100, 2, 1]
                if count in (13, 21, 33, 34):
                    values = [1, -20, 1, 3, 100, 3, 4, 60, 1, 0, 0, 3, 1]
                if count in (21, 33, 34):
                    values += [0, -5, 5, -1, .125, 1, 1, 0]
                if count in (33, 34):
                    values += [.125, -.125, .25, -.25, 0, -.125, .25, 0, 16700, -1500, 1500, -16700]
                if count == 20:
                    values = [1, -128, 1, 4, 600, 100, 2, 1, 2, 1, 254, 127, 1, 1, 0, -.125, 0, 500, 81, 0]
                if count in (32, 37, 40):
                    values = [1,-128,1,4,600,100,2,1,2,1,254,127,1,1,0,-.125,0,500,81,0] + [.125,-.125,.125,-.125,.0625,-.125,.125,-.0625,4000,-1600,1600,-4000]
                if count in (37, 40):
                    values += [120, 320, 540, 760, 1]
                if count == 40:
                    values += [4, 3, 1]
                if count == 72:
                    values[64:72] = [1, -128, 1, 4, 600, 100, 2, 1]
                if count == 34:
                    values += [120]
                packet = struct.pack(f'<{count}f', *values) + TAIL
                # A different notification size also splits the signed offset bytes.
                stream = packet + packet if count == 8 else packet
                for i in range(0, len(stream), 17):
                    bridge.receive(None, stream[i:i+17])
                self.assertEqual(receiver.recv(1024), packet)
                if count == 8:
                    self.assertEqual(receiver.recv(1024), packet)
                self.assertEqual(records[-1], tuple(values))
            self.assertEqual(bridge.frames, 15)

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
