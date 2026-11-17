"""Compile the actual DX controller and run it without physical hardware."""
from pathlib import Path
import os
import subprocess
import sys
ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26'
BUILD = ROOT / 'tests/camera/.build/vision_align'
BUILD.mkdir(parents=True, exist_ok=True)
for directory, name in [('USER','vision_align.h'), ('HARDWARE','camera_protocol.h')]:
    (BUILD / name).write_bytes((FW / directory / name).read_bytes())
(BUILD / 'test_vision_align.c').write_bytes((ROOT / 'tests/camera/test_vision_align.c').read_bytes())
environment = os.environ.copy()
environment['PYTHONPATH'] = str(ROOT / 'tests/camera/.build/python_deps') + os.pathsep + environment.get('PYTHONPATH','')
subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'),
                str(BUILD / 'test_vision_align.c')], cwd=ROOT, env=environment, check=True)
