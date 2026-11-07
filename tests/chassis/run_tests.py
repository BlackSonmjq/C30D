"""Exercise production chassis loop and heading control in ARM emulation.

uv run --with unicorn --with pyelftools python tests/chassis/run_tests.py
Only board/RTOS dependencies are mocked. Extract functions to avoid linking unrelated
remote-control, flash and kinematics code; fail if any function cannot be found.
"""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
BALANCE = ROOT / "R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE"
BUILD = ROOT / "tests/mango/.build"
BUILD.mkdir(exist_ok=True)
source = (BALANCE / "balance.c").read_text(encoding="gbk")


def function(signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


header = (ROOT / "tests/chassis/test_chassis.c").read_text()
parts = [header]
parts += [function(name) for name in [
    "void Balance_task(void *pvParameters)",
    *[f"int Incremental_PI_{wheel} (" for wheel in "ABCD"],
]]
parts += ['#include "../../../R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/heading_control.c"']
test = BUILD / "test_chassis.c"
test.write_text("\n\n".join(parts) + "\n", encoding="utf-8")
# heading_control.h includes sys.h from this test's stub directory.
stub = BUILD / "sys.h"
stub.write_bytes((ROOT / "tests/chassis/stubs/sys.h").read_bytes())
subprocess.run([sys.executable, str(ROOT / "tests/mango/run_tests.py"),
                str(test)], cwd=ROOT, check=True)
