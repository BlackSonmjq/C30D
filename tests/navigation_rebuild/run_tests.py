"""Run the incoming navigation tests on the ARM firmware toolchain."""
from pathlib import Path
import os
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26"
HERE = Path(__file__).resolve().parent
BUILD = ROOT / "tests/mango/.build/navigation_rebuild"
BUILD.mkdir(parents=True, exist_ok=True)
for name in ("nav_core.h", "nav_core.c", "nav_config.h", "nav_can.h", "nav_can.c",
             "nav_plan.h", "nav_button.h", "nav_button.c"):
    shutil.copyfile(FW / "USER" / name, BUILD / name)
for name in ("navigation_tests.c", "nav_calibration_nominal.c", "wheel_control.c", "wheel_control.h"):
    shutil.copyfile(HERE / name, BUILD / name)
calibration = (FW / "USER/nav_calibration.c").read_text(encoding="utf-8")
validator = calibration[calibration.index("static int range_ok("):]
(BUILD / "calibration_validator.inc").write_text(validator, encoding="utf-8")
environment = os.environ.copy()
environment["PYTHONPATH"] = str(ROOT / "tests/camera/.build/python_deps")
environment["ARM_TEST_MAX_INSTRUCTIONS"] = "200000000"
for window in (1, 3, 5):
    source = f"""#define NAV_FILTER_FRAMES {window}
#include "nav_can.c"
#include "nav_core.c"
#include "nav_button.c"
#include "nav_calibration_nominal.c"
#include "calibration_validator.inc"
#include "wheel_control.c"
#define main upstream_main
#include "navigation_tests.c"
#undef main
int main(void) {{
    upstream_main();
    puts("PASS: incoming navigation, filter window {window}");
    return 0;
}}
"""
    (BUILD / "test.c").write_text(source, encoding="utf-8")
    subprocess.run([sys.executable, str(ROOT / "tests/mango/run_tests.py"), str(BUILD / "test.c")],
                   cwd=ROOT, env=environment, check=True)
