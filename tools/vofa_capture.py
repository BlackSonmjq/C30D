"""Receive 32-channel wheel-load, alignment, vision or legacy route telemetry."""
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
    "camera_pole camera_offset camera_green camera_region camera_bytes camera_frames camera_bad camera_uart_errors"
).split()
VISION_FIELDS = FIELDS[64:72]
ALIGN_FIELDS = VISION_FIELDS + (
    'align_phase align_locked gamepad_ly gamepad_rx stick_active align_ready '
    'drive_vx_mps drive_vy_mps drive_wz_radps rc_velocity_mmps control_flags align_blocked'
).split()
LOAD_FIELDS = ALIGN_FIELDS + [f'wheel_{w}_target_mps' for w in 'abcd'] + [
    f'wheel_{w}_feedback_mps' for w in 'abcd'] + [f'wheel_{w}_pwm' for w in 'abcd']
CHANNEL_COUNTS = (72, 64, 48, 32, 20, 8)
ALIGN_PHASES = ('IDLE', 'ACQUIRE', 'TRACK', 'DONE', 'LOST', 'TIMEOUT', 'WAIT')
PHASES = ("WAIT", "ODOM", "APPROACH", "CONFIRM", "WAIT_LIDAR", "WAIT_READY",
          "WORK_STOP", "WAIT_TOOL", "DONE", "FAULT", "CAPTURE", "RECOVER")
ERRORS = ("NONE", "CALIB", "READY", "ODOM", "LIDAR", "MISMATCH", "OVERRUN",
          "NO_PROGRESS", "SEG_TIMEOUT", "RUN_TIMEOUT", "TOOL_FAIL", "TOOL_TIMEOUT",
          "NO_WORK", "REC_TIMEOUT", "REFERENCE", "SEARCH_TIME")


def stamp():
    return datetime.now(TZ).isoformat(timespec="milliseconds")


def fields_for(count):
    return LOAD_FIELDS if count == 32 else ALIGN_FIELDS if count == 20 else VISION_FIELDS if count == 8 else FIELDS[:count]


def camera_summary(values):
    summary = (f'VISION B={values[0]:.0f} DX={values[1]:+.0f} G={values[2]:.0f} '
            f'REGION={values[3]:.0f} bytes={values[4]:.0f} frames={values[5]:.0f} '
            f'bad={values[6]:.0f} uart_errors={values[7]:.0f}')
    if len(values) in (20, 32):
        summary += (f' ALIGN={ALIGN_PHASES[int(values[8])]} LOCK={values[9]:.0f}'
                    f' LY/RX={values[10]:.0f}/{values[11]:.0f}'
                    f' v=({values[14]:+.3f},{values[15]:+.3f},{values[16]:+.3f})'
                    f' RC={values[17]:.0f} BLOCK=0x{int(values[19]):04X}')
    if len(values) == 32:
        summary += ' PWM=' + '/'.join(f'{v:.0f}' for v in values[28:32])
    return summary


class Framer:
    def __init__(self):
        self.buffer = bytearray()
        self.discarded = 0
        self.channel_count = None
        self.pending = None
        self.confirmed = False

    @staticmethod
    def decode(payload):
        if len(payload) not in tuple(count * 4 for count in CHANNEL_COUNTS):
            return None
        values = struct.unpack(f"<{len(payload)//4}f", payload)
        limits = {1: 17, 2: 11, 3: 15, 23: 1, 26: 1, 27: 1, 31: 7} if len(values) >= 48 else {0: 1, 2: 1, 3: 4}
        if len(values) in (20, 32):
            limits.update({8: 6, 9: 1, 10: 255, 11: 255, 12: 1, 13: 1, 18: 127, 19: 16383})
        if len(values) >= 64:
            limits.update({48: 2, 54: 4, 58: 3, 62: 3})
        if len(values) >= 72:
            limits.update({64: 1, 66: 1, 67: 4})
        if not all(math.isfinite(v) for v in values):
            return None
        # The firmware tick is a uint32_t converted to float32. Reject shifted
        # suffixes of a longer frame that happen to resemble legacy flag fields.
        if len(values) >= 48 and not (0 <= values[0] <= 4294967296 and values[0] == round(values[0])):
            return None
        if not all(0 <= values[i] <= limit and values[i] == round(values[i])
                   for i, limit in limits.items()):
            return None
        if len(values) == 32 and any(abs(v) > 16700 or v != round(v) for v in values[28:32]):
            return None
        if len(values) in (8, 20, 32, 72):
            camera = values[:8] if len(values) in (8, 20, 32) else values[64:72]
            pole, offset, green, region = camera[:4]
            if not (-128 <= offset <= 127 and offset == round(offset)):
                return None
            if (not pole and offset != 0) or (not (pole and green) and region != 0):
                return None
            if pole and green and region == 0:
                return None
            if any(v < 0 or v > 4294967296 or v != round(v) for v in camera[4:8]):
                return None
        return values

    def feed(self, block):
        self.buffer.extend(block)
        cursor = 0
        while True:
            tail = self.buffer.find(TAIL, cursor)
            if tail < 0:
                break
            sizes = [tail] if tail in tuple(count * 4 for count in CHANNEL_COUNTS) else []
            sizes += [self.channel_count * 4] if self.channel_count else [count * 4 for count in CHANNEL_COUNTS]
            values = None
            for size in dict.fromkeys(sizes):
                if tail >= size:
                    values = self.decode(bytes(self.buffer[tail-size:tail]))
                    if values is not None:
                        self.discarded += tail-size
                        del self.buffer[:tail+4]
                        self.channel_count = len(values)
                        cursor = 0
                        # On connection we may start inside a 72-channel frame.
                        # Its suffix can resemble a valid legacy frame, especially
                        # while WAIT fields are zero. Confirm the initial layout
                        # against the next frame before releasing either record.
                        if self.confirmed:
                            yield values
                        elif self.pending is not None and len(self.pending) == len(values):
                            self.confirmed = True
                            yield self.pending
                            self.pending = None
                            yield values
                        else:
                            if self.pending is not None:
                                self.discarded += len(self.pending) * 4 + 4
                            self.pending = values
                        break
            if values is None:
                # The tail pattern can straddle two finite float values. Keep the bytes
                # until another candidate validates a whole packet, rather than splitting it.
                cursor = tail+1
        if len(self.buffer) > 8192:
            self.discarded += len(self.buffer) - 4096
            del self.buffer[:-4096]


def capture(args):
    import serial
    folder = args.output_root / datetime.now(TZ).strftime("%Y%m%d_%H%M%S")
    folder.mkdir(parents=True, exist_ok=False)
    metadata = {"started": stamp(), "port": args.port, "baud": args.baud,
                "format": "8/20/32/48/64/72 little-endian float32 + 00 00 80 7f", "fields": None,
                "channel_count": None,
                "dtr": args.dtr, "rts": args.rts,
                "stop_file": str(folder / "stop.request")}
    (folder / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print("SESSION_DIR=" + str(folder), flush=True)
    parser = Framer()
    started = time.monotonic()
    count = total_bytes = 0
    first = last = None
    previous = None
    last_report = last_flush = started
    terminal_at = None
    warned = set()
    stop_reason = "duration_limit"

    def status():
        return {"updated": stamp(), "port": args.port, "frames": count,
                "raw_bytes": total_bytes, "discarded_bytes": parser.discarded,
                "seconds": round(time.monotonic() - started, 3),
                "latest": dict(zip(fields_for(len(last)), last)) if last else None}

    def save_status():
        temporary = folder / "status.tmp"
        try:
            temporary.write_text(json.dumps(status(), indent=2), encoding="utf-8")
            temporary.replace(folder / "status.json")
        except OSError:
            pass  # A viewer holding the status file must not stop the raw capture.

    try:
        with serial.Serial(port=None, baudrate=args.baud, timeout=.2, xonxoff=False,
                           rtscts=False, dsrdtr=False) as port:
            port.port = args.port
            port.dtr = args.dtr
            port.rts = args.rts
            port.open()
            print("SERIAL_OPEN: waiting for telemetry; no bytes will be sent", flush=True)
            with (folder / "raw.bin").open("wb", buffering=0) as raw, \
                 (folder / "frames.csv").open("w", newline="", encoding="utf-8-sig") as csvfile, \
                 (folder / "events.jsonl").open("w", encoding="utf-8") as eventfile:
                writer = csv.writer(csvfile)

                def event(kind, values):
                    item = {"at": stamp(), "event": kind, "frame": count,
                            "data": dict(zip(fields_for(len(values)), values))}
                    eventfile.write(json.dumps(item) + "\n")
                    if len(values) in (8, 20, 32):
                        print(f"{kind}: {camera_summary(values)}", flush=True)
                        return
                    print(f"{kind}: P{int(values[1]):02d} {PHASES[int(values[2])]} "
                          f"error={ERRORS[int(values[3])]} S={values[4]:.1f}/{values[5]:.1f} "
                          f"G={values[32]:.1f} R={values[33]:.1f} "
                          f"laser={values[10]:.0f}/{values[11]:.0f} "
                          f"v=({values[18]:+.3f},{values[19]:+.3f}) "
                          f"Q={values[40]:.0f}/{int(values[41]):02X} D={int(values[22]):02X}", flush=True)
                    if len(values) >= 64 and int(values[1]) in (5, 7, 9):
                        print(f"EDGE: E={values[48]:.0f} anchor={values[52]:.1f} "
                              f"aux={values[55]:.0f}/{values[56]:.0f} "
                              f"A={values[58]:.0f}/{int(values[59]):02X} "
                              f"source={values[62]:.0f} cap={values[63]:.3f}", flush=True)

                while time.monotonic() - started < args.duration:
                    if (folder / "stop.request").exists():
                        stop_reason = "requested"
                        break
                    block = port.read(min(max(port.in_waiting, 1), 4096))
                    if block:
                        raw.write(block)
                        total_bytes += len(block)
                    for values in parser.feed(block):
                        if metadata["channel_count"] is None:
                            metadata["channel_count"] = len(values)
                            metadata["fields"] = fields_for(len(values))
                            (folder / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
                            writer.writerow(["received_at"] + [f"ch{i:02d}_{name}" for i, name in enumerate(metadata["fields"])])
                            print(f"LAYOUT={len(values)} channels", flush=True)
                        elif len(values) != metadata["channel_count"]:
                            raise OSError("Telemetry layout changed during capture; start a new session after flashing.")
                        if len(values) >= 48 and last and values[0]+1000 < last[0] and last[23] == 0:
                            event("BOARD_TIME_RESTART", values)
                        last = values
                        first = first or values
                        count += 1
                        writer.writerow([stamp()] + list(values))
                        if len(values) in (8, 20, 32):
                            key = values[:4] + (values[8:10] + values[19:20] if len(values) in (20, 32) else ())
                            if key != previous:
                                event("VISION", values)
                                previous = key
                            continue  # Visual flags never drive route terminal/stop handling.
                        key = (values[1], values[2], values[3], values[23])
                        if key != previous:
                            event("STATE", values)
                            previous = key
                        point = int(values[1])
                        if point in (5, 7, 9) and values[4] > values[5] + 15 and point not in warned:
                            event("EDGE_ODOM_PAST_PLAN", values)
                            warned.add(point)
                        terminal = values[2] in (8, 9) or values[23] == 1
                        if terminal:
                            terminal_at = terminal_at or time.monotonic()
                        else:
                            terminal_at = None
                    now = time.monotonic()
                    if now - last_flush >= 1:
                        csvfile.flush()
                        eventfile.flush()
                        save_status()
                        last_flush = now
                    if now - last_report >= 10:
                        if last and len(last) in (8, 20, 32):
                            print(f"LIVE: frames={count} bytes={total_bytes} {camera_summary(last)}", flush=True)
                        elif last:
                            print(f"LIVE: frames={count} bytes={total_bytes} tick={last[0]:.0f} "
                                  f"P{int(last[1]):02d} {PHASES[int(last[2])]} "
                                  f"S={last[4]:.1f} G={last[32]:.1f} R={last[33]:.1f} "
                                  f"L={last[8]:.0f} Rlaser={last[9]:.0f} "
                                  f"CAN={last[26]:.0f} seq={last[24]:.0f}", flush=True)
                        else:
                            print(f"WAIT_DATA: bytes={total_bytes}, no valid 8/20/32/48/64/72-channel frame yet", flush=True)
                        last_report = now
                    if terminal_at is not None and now - terminal_at >= 5:
                        stop_reason = "fault_snapshot" if last[2] == 9 or last[23] == 1 else "route_done"
                        break
    except KeyboardInterrupt:
        stop_reason = "interrupted"
    except (serial.SerialException, OSError) as exc:
        stop_reason = f"serial_error: {exc}"
    finally:
        result = status()
        result.update({"stop_reason": stop_reason, "finished": stamp(),
                       "first": dict(zip(fields_for(len(first)), first)) if first else None})
        (folder / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        save_status()
        print("FINISHED=" + json.dumps({"frames": count, "bytes": total_bytes,
                                       "reason": stop_reason, "folder": str(folder)}), flush=True)


if __name__ == "__main__":
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", required=True)
    cli.add_argument("--baud", type=int, default=9600)
    cli.add_argument("--duration", type=float, default=900)
    cli.add_argument("--dtr", action="store_true")
    cli.add_argument("--rts", action="store_true")
    cli.add_argument("--output-root", type=Path,
                     default=Path(__file__).resolve().parents[2] / "logs/vofa")
    capture(cli.parse_args())
