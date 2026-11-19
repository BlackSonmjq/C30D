"""JustFloat framing and explicit telemetry layouts. No device/network access."""
from dataclasses import dataclass
import math
import struct

from vofa_legacy import FIELDS as LEGACY_FIELDS, Framer as LegacyFramer

TAIL = b"\x00\x00\x80\x7f"
NAV_FIELDS = tuple((
    "front_raw_mm back_raw_mm left_raw_mm right_raw_mm "
    "front_filtered_mm back_filtered_mm left_filtered_mm right_filtered_mm "
    "front_target_mm back_target_mm left_target_mm right_target_mm "
    "lidar_valid_mask lidar_reference_mask lidar_age_ms nav_state nav_reason "
    "target_point confirmed_point main_error_mm cross_error_mm heading_error_deg "
    "command_vx_mps command_vy_mps command_wz_radps "
    "wheel_target_a_mps wheel_target_b_mps wheel_target_c_mps wheel_target_d_mps "
    "wheel_actual_a_mps wheel_actual_b_mps wheel_actual_c_mps wheel_actual_d_mps "
    "pwm_a pwm_b pwm_c pwm_d battery_voltage_v motor_allowed hardware_enable "
    "software_stop imu_ready can_fresh work_stopped start_front_mm "
    "lidar_sequence telemetry_sequence mcu_time_ms"
).split())
ALIGN_FIELDS = tuple((
    "camera_pole camera_offset camera_green camera_region camera_bytes camera_frames "
    "camera_bad camera_uart_errors align_phase align_locked gamepad_ly gamepad_rx "
    "stick_active align_ready drive_vx_mps drive_vy_mps drive_wz_radps "
    "rc_velocity_mmps control_flags align_blocked"
).split())
assert len(NAV_FIELDS) == 48 and len(ALIGN_FIELDS) == 20

STATE_NAMES = ("初始化", "准备完成", "巡航", "减速校准", "等待激光", "制停中", "作业暂停", "已停止", "已完成")
REASON_NAMES = ("正常", "需要标定", "IMU未就绪", "CAN无帧或过期", "横向参考不足",
                "纵向参考不足", "参考相互矛盾", "近墙余量不足", "硬件或供电失能",
                "按键停止", "机构暂停")


def integer(value, low, high):
    return low <= value <= high and abs(value - round(value)) < 0.001


@dataclass(frozen=True)
class Profile:
    name: str
    fields: tuple[str, ...]

    @property
    def channels(self):
        return len(self.fields)

    @property
    def frame_bytes(self):
        return self.channels * 4 + 4

    def decode(self, payload):
        if len(payload) != self.channels * 4:
            return None
        if self.name.startswith("legacy") and self.channels in (48, 64, 80, 104, 112, 128):
            # Reuse the existing route decoder's finite/enum/version checks.
            return LegacyFramer.decode(payload)
        values = struct.unpack(f"<{self.channels}f", payload)
        if not all(math.isfinite(v) for v in values):
            return None
        if self.name == "nav48":
            limits = {12: (0, 15), 13: (0, 15), 14: (-1, 4294967296),
                      15: (0, 8), 16: (0, 10), 17: (0, 17), 18: (0, 17),
                      45: (0, 4294967296), 46: (1, 16777215), 47: (0, 4294967296)}
            limits.update({i: (0, 1) for i in range(38, 44)})
            if not all(integer(values[i], *bounds) for i, bounds in limits.items()):
                return None
            if not all(0 <= v <= 65535 for v in values[:12]):
                return None
            if values[18] > values[17] or int(values[13]) & ~int(values[12]):
                return None
        elif self.name == "align20":
            if not all(integer(values[i], 0, 1) for i in (9, 12, 13)):
                return None
        return values


PROFILES = {"nav48": Profile("nav48", NAV_FIELDS), "align20": Profile("align20", ALIGN_FIELDS)}
for count in (8, 48, 64, 72, 80, 104, 112, 128):
    # No 8/72 field definitions were present in the source project: use honest
    # generic names rather than assigning an unrelated layout to those channels.
    fields = tuple(LEGACY_FIELDS[:count]) if count not in (8, 72) else tuple(f"ch{i:02d}" for i in range(count))
    PROFILES[f"legacy{count}"] = Profile(f"legacy{count}", fields)


class Framer:
    """Reuse the old tail-search/window approach with one explicitly chosen layout."""
    def __init__(self, profile="nav48"):
        self.profile = PROFILES[profile] if isinstance(profile, str) else profile
        self.buffer = bytearray()
        self.discarded = 0

    def reset(self):
        discarded = len(self.buffer)
        self.discarded += discarded
        self.buffer.clear()
        return discarded

    def feed(self, block):
        self.buffer.extend(block)
        cursor = 0
        payload_size = self.profile.channels * 4
        while True:
            tail = self.buffer.find(TAIL, cursor)
            if tail < 0:
                break
            values = self.profile.decode(bytes(self.buffer[tail-payload_size:tail])) if tail >= payload_size else None
            if values is not None:
                packet = bytes(self.buffer[tail-payload_size:tail+4])
                self.discarded += tail-payload_size
                del self.buffer[:tail+4]
                cursor = 0
                yield packet, values
            else:
                # Do not split on a false tail spanning two finite float values.
                cursor = tail + 1
        if len(self.buffer) > 8192:
            self.discarded += len(self.buffer)-4096
            del self.buffer[:-4096]


def sample_packet(profile="nav48", sequence=1):
    p = PROFILES[profile]
    values = [0.0] * p.channels
    if profile == "nav48":
        values[:12] = [100, 120, 300, 2500] * 3
        values[12:19] = [15, 13, 20, 1, 0, 1, 0]
        values[37:48] = [12.2, 0, 1, 0, 1, 1, 0, 4800, 10, sequence, 1000+300*(sequence-1)]
    elif profile.startswith("legacy") and p.channels >= 80:
        values[76] = {80: 2, 104: 3, 112: 4, 128: 5}[p.channels]
    packet = struct.pack(f"<{p.channels}f", *values) + TAIL
    assert p.decode(packet[:-4]) is not None
    return packet
