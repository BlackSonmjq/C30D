"""Parser regressions: framing loss must not manufacture control-state evidence."""
import importlib.util
import csv
import io
import json
import struct
import sys
import tempfile
import types
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("vofa_capture", ROOT / "tools/vofa_capture.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def packet(count):
    if count == 60:
        return packet(58) + [2, 4]
    if count == 58:
        return packet(43) + [100, 2700, 20, 6, 10, 5, 4, -50, 12, .125, -.25, 15, 1, 12000, 15]
    if count == 43:
        return packet(41) + [3, 12]
    if count == 41:
        return packet(38) + [3, 2, 1]
    if count == 38:
        return packet(35) + [600, 2, 1]
    if count == 35:
        return packet(31) + [186, 127, 81, 200]
    if count == 31:
        v = packet(33)
        return v[:6] + [v[8], v[7], 120] + v[9:11] + v[13:]
    if count == 34:
        return packet(33) + [120]
    if count == 33:
        return packet(21) + [.125, -.125, .25, -.25, 0, -.125, .25, 0, 16700, -1500, 1500, -16700]
    if count == 21:
        return packet(13) + [0, -5, 5, -1, .125, 1, 1, 0]
    if count == 13:
        return [1, -20, 1, 3, 100, 3, 4, 60, 1, 0, 0, 3, 1]
    if count == 40:
        return packet(37) + [4, 3, 1]
    if count == 37:
        return packet(32) + [120, 320, 540, 760, 1]
    if count == 32:
        return packet(20) + [.125, -.125, .125, -.125, .0625, -.125, .125, -.0625, 4000, -1600, 1600, -4000]
    if count == 20:
        return packet(8) + [2, 1, 254, 127, 1, 1, 0, -.125, 0, 500, 81, 0]
    if count == 8:
        return [1, -20, 1, 3, 600, 100, 2, 1]
    values = [0.0] * count
    values[0], values[1], values[2], values[5] = 1234, 5, 10, 700
    values[26], values[27], values[31] = 1, 1, 3
    if count >= 64:
        values[48], values[54], values[58], values[62] = 2, 1, 2, 3
    if count >= 72:
        values[64:72] = [1, -20, 1, 3, 600, 100, 2, 1]
    return values


class FramingTests(unittest.TestCase):
    def test_stop_release_diagnostics(self):
        values = packet(60)
        self.assertEqual(capture.fields_for(60)[:58], capture.fields_for(58))
        self.assertEqual(capture.fields_for(60)[58:], ['vision_stop_reason', 'vision_release_reason'])
        self.assertEqual(capture.Framer.decode(struct.pack('<60f', *values)), tuple(values))
        self.assertIn('STOP=FRONT_LIMIT RELEASE=REGION_TIMEOUT', capture.camera_summary(values))
        for index, bad in ((58, -1), (58, .5), (58, 17), (59, 13), (59, float('nan'))):
            invalid = values.copy(); invalid[index] = bad
            self.assertIsNone(capture.Framer.decode(struct.pack('<60f', *invalid)))
        data = struct.pack('<60f', *values) + capture.TAIL
        for start in range(1, len(data)):
            parser = capture.Framer(); frames = []
            stream = data[start:] + data + data
            for offset in range(0, len(stream), 20):
                frames.extend(parser.feed(stream[offset:offset+20]))
            self.assertEqual(frames, [tuple(values)] * 2, start)

    def test_merged_navigation_diagnostics(self):
        values = packet(58)
        self.assertEqual(capture.fields_for(58)[:43], capture.fields_for(43))
        self.assertEqual(capture.fields_for(58)[43:], capture.NAV_FIELDS)
        self.assertEqual(len(capture.fields_for(58)), 58)
        self.assertEqual(capture.Framer.decode(struct.pack('<58f', *values)), tuple(values))
        self.assertIn('NAV=PAUSED/WORK P04>05 OWNER=VISION', capture.camera_summary(values))
        self.assertIn('L/R=100/2700mm AGE=20ms', capture.camera_summary(values))
        self.assertIn('REF/VALID=0xF/F', capture.camera_summary(values))
        for index, bad in ((43, -1), (44, 65536), (45, -2), (45, .5), (46, 9), (47, 14),
                           (48, 18), (49, -.5), (50, float('nan')), (51, float('inf')),
                           (54, 16), (55, 2), (56, -1), (57, .5), (0, 2)):
            invalid = values.copy(); invalid[index] = bad
            self.assertIsNone(capture.Framer.decode(struct.pack('<58f', *invalid)))
        values[45] = -1
        self.assertIsNotNone(capture.Framer.decode(struct.pack('<58f', *values)))
        values[47] = 13
        self.assertIn('EXIT_TIMEOUT', capture.camera_summary(values))

    def test_merged_partial_frame_never_becomes_legacy(self):
        for values in (packet(58), [0.] * 58):
            data = struct.pack('<58f', *values) + capture.TAIL
            for start in range(1, len(data)):
                parser = capture.Framer()
                stream = data[start:] + data + data
                frames = []
                for offset in range(0, len(stream), 20):
                    frames.extend(parser.feed(stream[offset:offset+20]))
                self.assertEqual(frames, [tuple(values)] * 2, start)

    def test_lock_diagnostics(self):
        values=packet(43)
        self.assertEqual(capture.fields_for(43)[41:], ['locked_region','lock_count'])
        self.assertEqual(capture.Framer.decode(struct.pack('<43f',*values)),tuple(values))
        self.assertIn('LOCK=3 LOCK_COUNT=12',capture.camera_summary(values))
        values[3]=0
        self.assertEqual(capture.Framer.decode(struct.pack('<43f',*values)),tuple(values))
        for index,bad in ((41,5),(41,-1),(41,.5),(42,-1),(42,.5),(42,float('nan'))):
            invalid=values.copy();invalid[index]=bad
            self.assertIsNone(capture.Framer.decode(struct.pack('<43f',*invalid)))

    def test_picking_diagnostics(self):
        values=packet(41)
        self.assertEqual(capture.fields_for(41)[38:], ['pick_phase','hit_region','hit_count'])
        self.assertIn('HIT_DONE=2 COUNT=1',capture.camera_summary(values))
        for index,bad in ((38,6),(39,5),(40,-1)):
            invalid=values.copy();invalid[index]=bad
            self.assertIsNone(capture.Framer.decode(struct.pack('<41f',*invalid)))

    def test_camera_receive_diagnostics(self):
        values=packet(38)
        self.assertEqual(capture.fields_for(38)[35:], ['camera_bytes','camera_bad','camera_uart_errors'])
        self.assertIn('CAM_RX=600 BAD=2 UART_ERR=1',capture.camera_summary(values))
        for index in (35,36,37):
            for bad in (-1, .5, float('nan'), 4294967808):
                invalid=values.copy();invalid[index]=bad
                self.assertIsNone(capture.Framer.decode(struct.pack('<38f',*invalid)))

    def test_current_layout(self):
        values = packet(31)
        self.assertEqual(len(capture.fields_for(31)), 31)
        self.assertEqual(capture.fields_for(31)[8], "s21c_front_mm")
        self.assertEqual(capture.Framer.decode(struct.pack("<31f", *values)), tuple(values))
        self.assertIn("FRONT=120mm", capture.camera_summary(values))

    def test_front_range_channel(self):
        values = packet(34)
        self.assertEqual(capture.fields_for(34)[33], 's21c_front_mm')
        self.assertEqual(capture.fields_for(34)[:33], capture.fields_for(33))
        self.assertIn('FRONT=120mm', capture.camera_summary(values))
        for distance in (0, 40, 65535):
            values[33] = distance
            self.assertIsNotNone(capture.Framer.decode(struct.pack('<34f', *values)))
        for distance in (-1, 65536, float('nan')):
            values[33] = distance
            self.assertIsNone(capture.Framer.decode(struct.pack('<34f', *values)))

    def test_split_corrupt_and_partial_start(self):
        for count in (8, 13, 20, 21, 31, 32, 33, 34, 35, 37, 38, 40, 41, 43, 48, 58, 64, 72):
            values = packet(count)
            data = struct.pack(f"<{count}f", *values) + capture.TAIL
            stream = b"noise" + data[100:] + data + data[:-7] + b"bad" + data + data
            parser = capture.Framer()
            frames = []
            for start in range(0, len(stream), 13):
                frames.extend(parser.feed(stream[start:start+13]))
            self.assertEqual(frames, [tuple(values)] * 3)

    def test_wheel_imu_layout_and_partial_start(self):
        values = packet(33)
        self.assertEqual(capture.fields_for(33)[:21], capture.IMU_FIELDS)
        self.assertEqual(capture.fields_for(33)[21:], capture.LOAD_FIELDS[20:32])
        self.assertIn("PWM=16700/-1500/1500/-16700", capture.camera_summary(values))
        for index, value in ((29, 16701), (32, -16701), (30, .5), (20, 16384), (18, 2)):
            invalid = values.copy()
            invalid[index] = value
            self.assertIsNone(capture.Framer.decode(struct.pack("<33f", *invalid)))
        for record in (values, [0.] * 33):
            data = struct.pack("<33f", *record) + capture.TAIL
            for start in range(1, len(data)):
                self.assertEqual(list(capture.Framer().feed(data[start:] + data + data)),
                                 [tuple(record)] * 2, start)

    def test_imu_layout_and_partial_start(self):
        values = packet(21)
        self.assertEqual(capture.fields_for(21)[:13], capture.COMPACT_FIELDS)
        self.assertEqual(capture.fields_for(21)[15], "yaw_error_deg")
        self.assertIn("ERR=+5.00deg", capture.camera_summary(values))
        self.assertIn("I=+0.125rad/s", capture.camera_summary(values))
        for index, value in ((18, 2), (19, -1), (20, 16384), (15, float("nan"))):
            invalid = values.copy()
            invalid[index] = value
            self.assertIsNone(capture.Framer.decode(struct.pack("<21f", *invalid)))
        for record in (values, [0.] * 21):
            data = struct.pack("<21f", *record) + capture.TAIL
            for start in range(1, len(data)):
                self.assertEqual(list(capture.Framer().feed(data[start:] + data + data)),
                                 [tuple(record)] * 2, start)

    def test_compact_layout_and_partial_start(self):
        values = packet(13)
        self.assertEqual(capture.fields_for(13)[11:], ["hit_region", "hit_count"])
        self.assertIn("PICK=COOLDOWN REAR=60mm fresh=1", capture.camera_summary(values))
        working = values.copy()
        working[6] = 5
        self.assertIn("PICK=WORK", capture.camera_summary(working))
        self.assertEqual(capture.Framer.decode(struct.pack("<13f", *working)), tuple(working))
        for index, value in ((4, -1), (5, 7), (6, 6), (7, 65536), (8, 2), (11, 5), (12, -1)):
            invalid = values.copy()
            invalid[index] = value
            self.assertIsNone(capture.Framer.decode(struct.pack("<13f", *invalid)))
        for record in (values, [0.] * 13):
            data = struct.pack("<13f", *record) + capture.TAIL
            for start in range(1, len(data)):
                self.assertEqual(list(capture.Framer().feed(data[start:] + data + data)),
                                 [tuple(record)] * 2, start)

    def test_completion_layout_and_partial_start(self):
        values = packet(40)
        self.assertEqual(capture.fields_for(40)[37:], ["pick_phase", "hit_region", "hit_count"])
        self.assertIn("PICK=COOLDOWN HIT_DONE=3 COUNT=1", capture.camera_summary(values))
        for index, value in ((37, 6), (38, 5), (39, -1)):
            invalid = values.copy()
            invalid[index] = value
            self.assertIsNone(capture.Framer.decode(struct.pack("<40f", *invalid)))
        for record in (values, [0.] * 40):
            data = struct.pack("<40f", *record) + capture.TAIL
            for start in range(1, len(data)):
                self.assertEqual(list(capture.Framer().feed(data[start:] + data + data)),
                                 [tuple(record)] * 2, start)

    def test_false_tail_straddling_finite_floats(self):
        values = packet(72)
        # Both floats are finite; their adjacent byte encodings contain a false frame tail.
        values[28] = struct.unpack("<f", bytes.fromhex("11 00 00 80"))[0]
        values[29] = struct.unpack("<f", bytes.fromhex("7f 00 00 00"))[0]
        data = struct.pack("<72f", *values) + capture.TAIL
        self.assertGreater(data.count(capture.TAIL), 1)
        parser = capture.Framer()
        self.assertEqual(list(parser.feed(data + data)), [tuple(values)] * 2)

    def test_partial_alignment_start_never_becomes_eight_channels(self):
        for values in (packet(20), [0.] * 20):
            data = struct.pack('<20f', *values) + capture.TAIL
            for start in range(1, len(data)):
                parser = capture.Framer()
                stream = data[start:] + data + data
                frames = []
                for offset in range(0, len(stream), 17):
                    frames.extend(parser.feed(stream[offset:offset+17]))
                self.assertEqual(frames, [tuple(values)] * 2, start)

    def test_wheel_load_layout_and_partial_start(self):
        values = packet(32)
        self.assertEqual(capture.Framer.decode(struct.pack('<32f', *values)), tuple(values))
        self.assertEqual(capture.fields_for(32)[:20], capture.ALIGN_FIELDS)
        self.assertEqual(capture.fields_for(32)[20], 'wheel_a_target_mps')
        self.assertEqual(capture.fields_for(32)[24], 'wheel_a_feedback_mps')
        self.assertEqual(capture.fields_for(32)[31], 'wheel_d_pwm')
        for index,value in ((28,16701),(29,-16701),(30,.5)):
            invalid=values.copy();invalid[index]=value
            self.assertIsNone(capture.Framer.decode(struct.pack('<32f',*invalid)))
        for values in (values,[0.]*32):
            data=struct.pack('<32f',*values)+capture.TAIL
            for start in range(1,len(data)):
                parser=capture.Framer()
                self.assertEqual(list(parser.feed(data[start:]+data+data)),[tuple(values)]*2,start)

    def test_layout_switch_is_exposed(self):
        counts = (8, 8, 13, 20, 21, 31, 32, 33, 34, 35, 37, 38, 40, 41, 43, 48, 58, 64, 72, 64, 48, 72, 32, 20, 8)
        stream = b''.join(struct.pack(f'<{n}f', *packet(n)) + capture.TAIL for n in counts)
        self.assertEqual([len(v) for v in capture.Framer().feed(stream)], list(counts))
        self.assertEqual(len(capture.FIELDS), 72)
        self.assertEqual(capture.FIELDS[48], "edge_state")
        self.assertEqual(capture.FIELDS[63], "edge_speed_cap_mps")
        self.assertEqual(capture.FIELDS[64:68], ["camera_pole", "camera_offset", "camera_green", "camera_region"])
        self.assertEqual(capture.fields_for(8), capture.FIELDS[64:72])
        self.assertEqual(capture.fields_for(20)[:8], capture.VISION_FIELDS)
        self.assertEqual(capture.fields_for(20)[8], 'align_phase')
        self.assertEqual(len(capture.fields_for(20)), 20)
        self.assertEqual(capture.fields_for(48), capture.FIELDS[:48])

    def test_camera_signed_offset_and_validation(self):
        self.assertEqual(capture.Framer.decode(struct.pack('<20f', *packet(20))), tuple(packet(20)))
        self.assertIn('ALIGN=TRACK LOCK=1', capture.camera_summary(packet(20)))
        for index, value in ((8, 7), (9, 2), (10, 256), (11, -1), (12, .5), (13, 2), (18, 128), (19, 16384)):
            values = packet(20)
            values[index] = value
            self.assertIsNone(capture.Framer.decode(struct.pack('<20f', *values)))
        for offset in (-128, -20, 0, 99, 102, 127):
            values = packet(8)
            values[1] = offset
            self.assertEqual(capture.Framer.decode(struct.pack('<8f', *values)), tuple(values))
        for index, value in ((0, 2), (1, -129), (1, 128), (1, .5), (2, 2),
                             (3, 5), (4, -1), (5, .5)):
            values = packet(8)
            values[index] = value
            self.assertIsNone(capture.Framer.decode(struct.pack('<8f', *values)))
        for offset in (-128, -20, 0, 99, 102, 127):
            values = packet(72)
            values[65] = offset
            self.assertEqual(capture.Framer.decode(struct.pack('<72f', *values)), tuple(values))
        for index, value in ((64, 2), (65, -129), (65, 128), (65, .5), (66, 2),
                             (67, 5), (68, -1), (69, .5)):
            values = packet(72)
            values[index] = value
            self.assertIsNone(capture.Framer.decode(struct.pack('<72f', *values)))
        values = packet(72)
        values[64:68] = [0, 0, 1, 0]  # Green may be present without a pole.
        self.assertIsNotNone(capture.Framer.decode(struct.pack('<72f', *values)))

    def test_new_channels_do_not_change_existing_payload(self):
        old, new = packet(64), packet(72)
        self.assertEqual(struct.pack('<72f', *new)[:256], struct.pack('<64f', *old))

    def test_partial_wait_frame_never_becomes_legacy_csv_row(self):
        # A zero-heavy WAIT snapshot previously passed as a 48/64-channel suffix
        # when BLE subscription began in the middle of the new frame.
        values = [0.] * 72
        values[0] = 1234.
        data = struct.pack('<72f', *values) + capture.TAIL
        for start in range(1, len(data)):
            parser = capture.Framer()
            frames = []
            stream = data[start:] + data + data
            for offset in range(0, len(stream), 20):
                frames.extend(parser.feed(stream[offset:offset+20]))
            self.assertEqual(frames, [tuple(values)] * 2, start)

    def test_visual_capture_csv_metadata_and_summary(self):
        for count in (8, 13, 20, 21, 31, 32, 33, 34, 35, 37, 38, 40, 41, 43, 58):
            values = packet(count)
            data = struct.pack(f'<{count}f', *values) + capture.TAIL

            class SerialPort:
                in_waiting = len(data) * 2
                def __init__(self, **kwargs):
                    self.sent = False
                def __enter__(self):
                    return self
                def __exit__(self, *args):
                    pass
                def open(self):
                    pass
                def read(self, size):
                    if self.sent:
                        raise KeyboardInterrupt
                    self.sent = True
                    return data * 2

            serial = types.SimpleNamespace(Serial=SerialPort, SerialException=OSError)
            with tempfile.TemporaryDirectory() as temporary:
                args = types.SimpleNamespace(output_root=Path(temporary), port='FAKE', baud=9600,
                                             dtr=False, rts=False, duration=60)
                with patch.dict(sys.modules, {'serial': serial}), redirect_stdout(io.StringIO()):
                    capture.capture(args)
                folder, = Path(temporary).iterdir()
                with (folder / 'frames.csv').open(encoding='utf-8-sig', newline='') as stream:
                    rows = list(csv.reader(stream))
                self.assertEqual(rows[0], ['received_at'] + [f'ch{i:02d}_{name}' for i, name in enumerate(capture.fields_for(count))])
                self.assertEqual(len(rows), 3)
                self.assertEqual([float(v) for v in rows[1][1:]], values)
                metadata = json.loads((folder / 'metadata.json').read_text())
                summary = json.loads((folder / 'summary.json').read_text())
                self.assertEqual(metadata['channel_count'], count)
                self.assertEqual(metadata['fields'], capture.fields_for(count))
                self.assertEqual(summary['latest'], dict(zip(capture.fields_for(count), values)))
                self.assertEqual(summary['frames'], 2)
                self.assertEqual((folder / 'raw.bin').read_bytes(), data * 2)


if __name__ == "__main__":
    unittest.main()
