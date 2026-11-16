"""Receive 48/64-channel robot JustFloat telemetry without sending commands."""
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
    "aux_confidence aux_reject_reason aux_good_frames aux_bad_frames control_source edge_speed_cap_mps"
).split()
PHASES = ("WAIT", "ODOM", "APPROACH", "CONFIRM", "WAIT_LIDAR", "WAIT_READY",
          "WORK_STOP", "WAIT_TOOL", "DONE", "FAULT", "CAPTURE", "RECOVER")
ERRORS = ("NONE", "CALIB", "READY", "ODOM", "LIDAR", "MISMATCH", "OVERRUN",
          "NO_PROGRESS", "SEG_TIMEOUT", "RUN_TIMEOUT", "TOOL_FAIL", "TOOL_TIMEOUT",
          "NO_WORK", "REC_TIMEOUT", "REFERENCE", "SEARCH_TIME")


def stamp():
    return datetime.now(TZ).isoformat(timespec="milliseconds")


class Framer:
    def __init__(self):
        self.buffer = bytearray()
        self.discarded = 0
        self.channel_count = None

    @staticmethod
    def decode(payload):
        if len(payload) not in (192, 256):
            return None
        values = struct.unpack(f"<{len(payload)//4}f", payload)
        limits = {1: 17, 2: 11, 3: 15, 23: 1, 26: 1, 27: 1, 31: 7}
        if len(values) == 64:
            limits.update({48: 2, 54: 4, 58: 3, 62: 3})
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
            sizes = [tail] if tail in (192, 256) else []
            sizes += [self.channel_count*4] if self.channel_count else [256, 192]
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


def capture(args):
    import serial
    folder = args.output_root / datetime.now(TZ).strftime("%Y%m%d_%H%M%S")
    folder.mkdir(parents=True, exist_ok=False)
    metadata = {"started": stamp(), "port": args.port, "baud": args.baud,
                "format": "48/64 little-endian float32 + 00 00 80 7f", "fields": FIELDS,
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
                "latest": dict(zip(FIELDS, last)) if last else None}

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
                            "data": dict(zip(FIELDS, values))}
                    eventfile.write(json.dumps(item) + "\n")
                    print(f"{kind}: P{int(values[1]):02d} {PHASES[int(values[2])]} "
                          f"error={ERRORS[int(values[3])]} S={values[4]:.1f}/{values[5]:.1f} "
                          f"G={values[32]:.1f} R={values[33]:.1f} "
                          f"laser={values[10]:.0f}/{values[11]:.0f} "
                          f"v=({values[18]:+.3f},{values[19]:+.3f}) "
                          f"Q={values[40]:.0f}/{int(values[41]):02X} D={int(values[22]):02X}", flush=True)
                    if len(values) == 64 and int(values[1]) in (5, 7, 9):
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
                            metadata["fields"] = FIELDS[:len(values)]
                            (folder / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
                            writer.writerow(["received_at"] + [f"ch{i:02d}_{name}" for i, name in enumerate(metadata["fields"])])
                            print(f"LAYOUT={len(values)} channels", flush=True)
                        elif len(values) != metadata["channel_count"]:
                            raise OSError("Telemetry layout changed during capture; start a new session after flashing.")
                        if last and values[0]+1000 < last[0] and last[23] == 0:
                            event("BOARD_TIME_RESTART", values)
                        last = values
                        first = first or values
                        count += 1
                        writer.writerow([stamp()] + list(values))
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
                        if last:
                            print(f"LIVE: frames={count} bytes={total_bytes} tick={last[0]:.0f} "
                                  f"P{int(last[1]):02d} {PHASES[int(last[2])]} "
                                  f"S={last[4]:.1f} G={last[32]:.1f} R={last[33]:.1f} "
                                  f"L={last[8]:.0f} Rlaser={last[9]:.0f} "
                                  f"CAN={last[26]:.0f} seq={last[24]:.0f}", flush=True)
                        else:
                            print(f"WAIT_DATA: bytes={total_bytes}, no valid 48/64-channel frame yet", flush=True)
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
                       "first": dict(zip(FIELDS, first)) if first else None})
        (folder / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        save_status()
        print("FINISHED=" + json.dumps({"frames": count, "bytes": total_bytes,
                                       "reason": stop_reason, "folder": str(folder)}), flush=True)


if __name__ == "__main__":
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", required=True)
    cli.add_argument("--baud", type=int, default=115200)
    cli.add_argument("--duration", type=float, default=900)
    cli.add_argument("--dtr", action="store_true")
    cli.add_argument("--rts", action="store_true")
    cli.add_argument("--output-root", type=Path,
                     default=Path(__file__).resolve().parents[2] / "logs/vofa")
    capture(cli.parse_args())
