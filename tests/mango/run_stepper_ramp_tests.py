"""Execute the production pulse-period ramp with timer-register mocks."""
from pathlib import Path
import runpy
import sys
root = Path(__file__).resolve().parents[2]
build = root / 'tests/mango/.build'
build.mkdir(exist_ok=True)
code = (root / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/stepper_motor.c').read_bytes().decode('gbk').replace('\r\n','\n')
start = code.index('static void StepperMotor_StartNext(u8 motor)\n{')
opening = code.index('{', start)
end, depth = opening+1, 1
while depth:
    depth += (code[end]=='{')-(code[end]=='}')
    end += 1
defines = '\n'.join(line for line in code.splitlines() if line.startswith('#define STEPPER_'))
(build / 'stepper_ramp.inc').write_text(defines+'\n'+code[start:end]+'\n', encoding='utf-8')
sys.argv=[str(root/'tests/mango/run_tests.py'),'tests/mango/test_stepper_ramp.c']
runpy.run_path(sys.argv[0],run_name='__main__')
