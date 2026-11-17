"""Test the production A4988 GPIO direction code with mocked GPIO hardware."""
from pathlib import Path
import runpy
import sys

root = Path(__file__).resolve().parents[2]
build = root / "tests/mango/.build"
build.mkdir(exist_ok=True)
source = root / "R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/a4988.c"
# Keep the production function bodies; replace only hardware header dependencies.
code = source.read_bytes().splitlines(keepends=True)
(build / "a4988_direction.inc").write_bytes(
    b"".join(line for line in code if not line.startswith(b"#include"))
)
sys.argv = [str(root / "tests/mango/run_tests.py"), "tests/mango/test_stepper_dir.c"]
runpy.run_path(sys.argv[0], run_name="__main__")
