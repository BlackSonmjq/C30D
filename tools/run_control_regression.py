"""Compare every tested control output with the pre-BLE backup; no board access."""
from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import subprocess
from host_test_calibration import build_validator

ROOT = Path(__file__).resolve().parents[1]
backup = Path(json.loads((ROOT / "build/ble-baseline.json").read_text(encoding="utf-8"))["backup"])
cc = os.environ.get("HOST_CC") or shutil.which("gcc")
if not cc:
    raise SystemExit("GCC required")
out = ROOT / "OBJ/tests/control-regression"
out.mkdir(parents=True, exist_ok=True)
harness = (backup / "tests/navigation_tests.c").read_text(encoding="utf-8")
trace = r'''
static uint64_t trace_hash=14695981039346656037ULL;
static void trace_u32(uint32_t v) {
    unsigned i; for(i=0;i<4;i++) { trace_hash^=(v>>(8*i))&255U; trace_hash*=1099511628211ULL; }
}
static void trace_float(float v) { uint32_t bits; memcpy(&bits,&v,4); trace_u32(bits); }
static void trace_nav(const NavCore *n) {
    unsigned i,j;
    trace_u32(n->state); trace_u32(n->reason); trace_u32(n->target); trace_u32(n->confirmed);
    trace_u32(n->estimated_point); trace_u32(n->arrival_count); trace_u32(n->recovery_count);
    trace_u32(n->zero_reset_done); trace_u32(n->pause_latched);
    trace_float(n->output.vx); trace_float(n->output.vy); trace_float(n->output.wz);
    trace_u32(n->output.drive_enabled); trace_u32(n->output.quick_stop);
    trace_u32(n->output.reset_pi); trace_u32(n->output.work_stopped);
    trace_float(n->start_front_mm); trace_float(n->heading_origin_rad);
    trace_float(n->main_error_mm); trace_float(n->cross_error_mm); trace_float(n->heading_error_rad);
    for(i=0;i<4;i++) {
        trace_float(n->filter[i].value); trace_u32(n->filter[i].last_sequence);
        trace_u32(n->filter[i].count); trace_u32(n->filter[i].next); trace_u32(n->filter[i].role);
        for(j=0;j<NAV_FILTER_FRAMES;j++) trace_float(n->filter[i].samples[j]);
    }
}
'''
harness = harness.replace("typedef struct {", trace + "\ntypedef struct {", 1)
harness = harness.replace("Nav_Tick(&s->nav,&s->input,&s->frame);", "Nav_Tick(&s->nav,&s->input,&s->frame); trace_nav(&s->nav);")
harness = harness.replace("Nav_Tick(&s.nav,&s.input,&s.frame);", "Nav_Tick(&s.nav,&s.input,&s.frame); trace_nav(&s.nav);")
harness = harness.replace('    return 0;\n}', '    printf("CONTROL_TRACE=%016llx\\n",(unsigned long long)trace_hash);\n    return 0;\n}')
source = out / "trace.c"
source.write_text(harness, encoding="utf-8")
result = {}
for window in (1, 3, 5):
    hashes = {}
    for name, base in (("before", backup), ("after", ROOT)):
        exe = out / f"{name}_{window}.exe"
        cmd = [cc, "-std=c99", "-O2", f"-DNAV_FILTER_FRAMES={window}", "-I", str(base / "USER"),
               "-I", str(base / "BALANCE"), str(source)]
        cmd += [str(base / p) for p in ("USER/nav_can.c", "USER/nav_core.c",
                                       "USER/nav_button.c", "BALANCE/wheel_control.c")]
        cmd += [str(ROOT/"tests/fixtures/nav_calibration_nominal.c"),
                str(build_validator(base,cc,out,f"{name}-validator-{window}",window))]
        cmd += ["-lm", "-o", str(exe)]
        subprocess.run(cmd, check=True, cwd=ROOT)
        log = subprocess.check_output([exe], cwd=ROOT).decode("utf-8")
        (out / f"{name}_{window}.log").write_text(log, encoding="utf-8")
        hashes[name] = re.search(r"CONTROL_TRACE=([0-9a-f]+)", log).group(1)
    assert hashes["before"] == hashes["after"], hashes
    result[str(window)] = hashes
    print(f"PASS window={window}: original and BLE firmware control trace {hashes['after']}")
# Allow the telemetry publication and the explicit output gate, then compare the loop exactly.
before = (backup / "BALANCE/balance.c").read_bytes()
after = (ROOT / "BALANCE/balance.c").read_bytes()
after=re.sub(rb"^[ \t]*Motor_OutputEnable\([01]\);\r?\n",b"",after,flags=re.M)
after=after.replace(b"Motor_OutputEnable(0); ",b"")
after = after.replace(b"NavBoard_PublishDrive(0)", b"NavBoard_PublishDrive()").replace(
    b"NavBoard_PublishDrive(1)", b"NavBoard_PublishDrive()")
assert before == after, "Unexpected change in the motor-control source"
for name in ("BALANCE/wheel_control.c", "BALANCE/imu_task.c"):
    assert (backup / name).read_bytes() == (ROOT / name).read_bytes(), name
robot_profile = "BALANCE/robot_select_init.c"
expected_profile = (backup / robot_profile).read_bytes()
# The confirmed GMR hardware uses the vendor's 500-line profile.
expected_profile = expected_profile.replace(b"mecanum + Hall encoder", b"mecanum + GMR encoder")
expected_profile = expected_profile.replace(b"HALL_30F,Hall_13,Mecanum_75", b"HALL_30F,Photoelectric_500,Mecanum_75")
assert (ROOT / robot_profile).read_bytes() == expected_profile, robot_profile
motor_baseline=ROOT/"build/motor-startup-baseline.json"
if motor_baseline.exists():
    original=Path(json.loads(motor_baseline.read_text(encoding="utf-8"))["backup"])
    for name in ("USER/nav_calibration.c","USER/nav_config.h","USER/nav_core.c","USER/nav_button.c"):
        assert (original/name).read_bytes()==(ROOT/name).read_bytes(),name
result["control_loop_unchanged_except_publication_and_output_gate"] = True
result["fixture"] = "tests/fixtures/nav_calibration_nominal.c"
(out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
