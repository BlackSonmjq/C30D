# Reused legacy route decoder; original 48/64/80/104/112/128 validation retained.
"""Receive 48/64/80/104/112/128-channel JustFloat telemetry without sending commands."""
import argparse
import csv
import json
import math
import struct
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

TZ = timezone(timedelta(hours=8))
TAIL = bytes.fromhex("00 00 80 7f")
FIELDS = (
    "tick_ms target_point phase error progress_mm length_mm front_raw_mm behind_raw_mm "
    "left_raw_mm right_raw_mm active_filtered_mm target_mm odom_remaining_mm "
    "laser_remaining_mm mismatch_mm odom_dt_ms odom_max_dt_ms yaw_deg command_vx_mps "
    "command_vy_mps measured_vx_mps measured_vy_mps stop_flags frozen lidar_sequence "
    "active_age_ms can_fresh odom_valid x_mm y_mm gate_mm lane estimated_end_mm "
    "control_remaining_mm wall_innovation_mm wall_locked capture_confirmed "
    "odom_untrusted search_elapsed_ms reference_idle_ms confidence reject_reason "
    "good_frames bad_frames speed_limited odom_warn_count recovering recovery_resets "
    "edge_state edge_confirmed edge_last_near_mm edge_first_far_mm edge_anchor_mm "
    "edge_uncertainty_mm aux_sensor aux_filtered_mm aux_target_mm aux_remaining_mm "
    "aux_confidence aux_reject_reason aux_good_frames aux_bad_frames control_source edge_speed_cap_mps "
    "wheel_a_mps wheel_b_mps wheel_c_mps wheel_d_mps chassis_stopped aux_wait_stage "
    "aux_wait_elapsed_ms aux_verify_elapsed_ms aux_reset_reason aux_reset_count "
    "edge_transition_age_ms wall_history_rejected diagnostic_version wall_no_progress_limit_ms "
    "aux_wait_limit_ms aux_verify_limit_ms "
    "wheel_target_a_mps wheel_target_b_mps wheel_target_c_mps wheel_target_d_mps "
    "applied_pwm_a applied_pwm_b applied_pwm_c applied_pwm_d battery_voltage_v "
    "velocity_kp velocity_ki motor_output_allowed drive_block_flags drive_command_age_ms "
    "drive_tick_ms drive_enable drive_software_stop drive_tool_busy pwm_clear_active "
    "smooth_vx_mps smooth_vy_mps smooth_vz_radps drive_control_owned drive_snapshot_valid "
    "fine_stage fine_elapsed_ms fine_pulses fine_brake_mm fine_pulse_elapsed_ms "
    "fine_settle_elapsed_ms fine_frames fine_pulse_travel_mm "
    "terrain_stage terrain_reason terrain_attempts terrain_elapsed_ms terrain_probe_elapsed_ms "
    "terrain_travel_mm terrain_limit_mm terrain_stall_ms terrain_odom_delta_mm terrain_laser_delta_mm "
    "terrain_sensor terrain_frames terrain_speed_cap_mps terrain_result terrain_correction_mm terrain_yaw_deg"
).split()
PHASES = ("WAIT", "ODOM", "APPROACH", "CONFIRM", "WAIT_LIDAR", "WAIT_READY",
          "WORK_STOP", "WAIT_TOOL", "DONE", "FAULT", "CAPTURE", "RECOVER")
ERRORS = ("NONE", "CALIB", "READY", "ODOM", "LIDAR", "MISMATCH", "OVERRUN",
          "NO_PROGRESS", "SEG_TIMEOUT", "RUN_TIMEOUT", "TOOL_FAIL", "TOOL_TIMEOUT",
          "NO_WORK", "REC_TIMEOUT", "REFERENCE", "SEARCH_TIME", "FINE_TIME", "FINE_LIMIT")


def stamp():
    return datetime.now(TZ).isoformat(timespec="milliseconds")


class Framer:
    def __init__(self, channels=None):
        if channels not in (None, 48, 64, 80, 104, 112, 128):
            raise ValueError("channels must be 48, 64, 80, 104, 112, 128 or None")
        self.expected_channels = channels
        self.buffer = bytearray()
        self.discarded = 0
        self.channel_count = None

    @staticmethod
    def decode(payload):
        if len(payload) not in (192, 256, 320, 416, 448, 512):
            return None
        values = struct.unpack(f"<{len(payload)//4}f", payload)
        limits = {1: 17, 2: 11, 3: 15, 23: 1, 26: 1, 27: 1, 31: 7}
        if len(values) >= 112:
            limits.update({3: 17, 104: 3, 106: 255, 110: 255})
        if len(values) == 128:
            limits.update({112: 4, 113: 7, 114: 3, 122: 4, 123: 255, 125: 3})
        if len(values) >= 64:
            limits.update({48: 2, 54: 4, 58: 3, 62: 3})
        if len(values) >= 80:
            limits.update({68: 1, 69: 5, 72: 12, 75: 1})
            if len(values) >= 112:
                limits.update({69: 8, 72: 16})  # c679c06 appended entry-recovery enums.
            if values[76] != (5 if len(values) == 128 else 4 if len(values) == 112 else 3 if len(values) == 104 else 2):
                return None
        if len(values) >= 104:
            limits.update({91: 1, 92: 63, 95: 1, 96: 1, 97: 1, 98: 1, 102: 1, 103: 1})
        if not all(math.isfinite(v) for v in values):
            return None
        if not all(0 <= values[i] <= limit and abs(values[i]-round(values[i])) < .001
                   for i, limit in limits.items()):
            return None
        return values

    def feed(self, block):
        self.buffer.extend(block)
        cursor = 0
        while True:
            tail = self.buffer.find(TAIL, cursor)
            if tail < 0:
                break
            if self.expected_channels:
                sizes = [self.expected_channels * 4]
            else:
                sizes = [tail] if tail in (192, 256, 320, 416, 448, 512) else []
                sizes += [512, 448, 416, 320, 256, 192]
            values = None
            for size in dict.fromkeys(sizes):
                if tail >= size:
                    values = self.decode(bytes(self.buffer[tail-size:tail]))
                    if values is not None:
                        self.discarded += tail-size
                        del self.buffer[:tail+4]
                        self.channel_count = len(values)
                        cursor = 0
                        yield values
                        break
            if values is None:
                # The tail pattern can straddle two finite float values. Keep the bytes
                # until another candidate validates a whole packet, rather than splitting it.
                cursor = tail+1
        if len(self.buffer) > 8192:
            self.discarded += len(self.buffer) - 4096
            del self.buffer[:-4096]

