"""Parser regressions: framing loss must not manufacture control-state evidence."""
import importlib.util
import struct
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("vofa_capture", ROOT / "tools/vofa_capture.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def packet(count):
    values = [0.0] * count
    values[0], values[1], values[2], values[5] = 1234, 5, 10, 700
    values[26], values[27], values[31] = 1, 1, 3
    if count == 64:
        values[48], values[54], values[58], values[62] = 2, 1, 2, 3
    return values


class FramingTests(unittest.TestCase):
    def test_split_corrupt_and_partial_start(self):
        for count in (48, 64):
            values = packet(count)
            data = struct.pack(f"<{count}f", *values) + capture.TAIL
            stream = b"noise" + data[100:] + data + data[:-7] + b"bad" + data + data
            parser = capture.Framer()
            frames = []
            for start in range(0, len(stream), 13):
                frames.extend(parser.feed(stream[start:start+13]))
            self.assertEqual(frames, [tuple(values)] * 3)

    def test_false_tail_straddling_finite_floats(self):
        values = packet(64)
        # Both floats are finite; their adjacent byte encodings contain a false frame tail.
        values[28] = struct.unpack("<f", bytes.fromhex("11 00 00 80"))[0]
        values[29] = struct.unpack("<f", bytes.fromhex("7f 00 00 00"))[0]
        data = struct.pack("<64f", *values) + capture.TAIL
        self.assertGreater(data.count(capture.TAIL), 1)
        parser = capture.Framer()
        self.assertEqual(list(parser.feed(data + data)), [tuple(values)] * 2)

    def test_layout_switch_is_exposed(self):
        old, new = packet(48), packet(64)
        stream = struct.pack("<48f", *old)+capture.TAIL+struct.pack("<64f", *new)+capture.TAIL
        self.assertEqual([len(v) for v in capture.Framer().feed(stream)], [48, 64])
        self.assertEqual(len(capture.FIELDS), 64)
        self.assertEqual(capture.FIELDS[48], "edge_state")
        self.assertEqual(capture.FIELDS[63], "edge_speed_cap_mps")


if __name__ == "__main__":
    unittest.main()
