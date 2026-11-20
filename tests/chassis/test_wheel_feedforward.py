"""Exercise real kinematics, four PI loops and the shared per-wheel output path."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile
ROOT = Path(__file__).resolve().parents[2]
BALANCE = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE'
source = (BALANCE / 'balance.c').read_bytes().decode('gbk')
def function(signature):
    start = source.index(signature)
    opening = source.index('{',start)
    depth,end=1,opening+1
    while depth:
        depth += (source[end]=='{')-(source[end]=='}')
        end += 1
    return source[start:end]+'\n'
logic='\n'.join(function(f'int Incremental_PI_{w} (') for w in 'ABCD')
logic+='static void production_targets(float Vx,float Vy,float Vz){float amplitude=3.5f,LeftWheelDiff=1,RightWheelDiff=1;\n'
logic+=function('if (Car_Mode==Mec_Car||Car_Mode==Mec_Car_V550)')+'}\n'
a=source.index('wheel_output_active = !start_clear')
b=source.index('Limit_Pwm(16700);',a)+len('Limit_Pwm(16700);')
logic+='static void production_pwm(void){'+source[a:b]+'}\n'
compiler=os.environ.get('ROUTE_HOST_CC') or shutil.which('gcc')
with tempfile.TemporaryDirectory(prefix='wheel-load-') as temp:
    build=Path(temp) if compiler else ROOT/'tests/mango/.build/wheel-load'
    build.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(BALANCE/'wheel_output.h',build/'wheel_output.h')
    shutil.copyfile(ROOT/'tests/chassis/test_wheel_load.c',build/'test.c')
    for name in ('heading_control.c','heading_control.h'):
        shutil.copyfile(BALANCE/name,build/name)
    shutil.copyfile(ROOT/'tests/chassis/stubs/sys.h',build/'sys.h')
    shutil.copyfile(BALANCE.parent/'USER/nav_config.h',build/'nav_config.h')
    (build/'production.inc').write_text(logic,encoding='utf-8')
    if not compiler:
        # Finite multi-wheel plant loops execute software floating point on
        # this test runner and need more instructions than state-only tests.
        environment=dict(os.environ,ARM_TEST_MAX_INSTRUCTIONS='200000000')
        subprocess.run([sys.executable,str(ROOT/'tests/mango/run_tests.py'),str(build/'test.c')],cwd=ROOT,env=environment,check=True)
    else:
        executable=build/'test.exe'
        subprocess.run([compiler,'-std=c99','-Wall','-Wextra','-Werror','-I',str(build),str(build/'test.c'),'-lm','-o',str(executable)],check=True)
        subprocess.run([str(executable)],check=True)
