"""Run the production vision qualifier and dispatcher in Cortex-M4 emulation."""
from pathlib import Path
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26'
BUILD = ROOT / 'tests/camera/.build/vision_strike'
BUILD.mkdir(parents=True, exist_ok=True)
for directory, name in [('USER', 'vision_strike.h'), ('HARDWARE', 'camera_protocol.h')]:
    (BUILD / name).write_bytes((FW / directory / name).read_bytes())
production = (FW / 'HARDWARE/usartx.c').read_text(encoding='utf-8')
start = production.index('void VisionStrike_Process(void)')
opening = production.index('{', start)
depth = 1
end = opening + 1
while depth:
    depth += (production[end] == '{') - (production[end] == '}')
    end += 1
(BUILD / 'production_dispatch.inc').write_text(production[start:end] + '\n', encoding='utf-8')
# Exercise the optional enabled feature explicitly; the current firmware defaults
# to disabled, verified by run_vision_align_integration.py using the same header.
(BUILD / 'test_vision_strike.c').write_bytes(b'#define VISION_STRIKE_ENABLE 1\n' + (ROOT / 'tests/camera/test_vision_strike.c').read_bytes())
environment = os.environ.copy()
deps = str(ROOT / 'tests/camera/.build/python_deps')
environment['PYTHONPATH'] = deps + os.pathsep + environment.get('PYTHONPATH', '')
subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'),
                str(BUILD / 'test_vision_strike.c')], cwd=ROOT, env=environment, check=True)
