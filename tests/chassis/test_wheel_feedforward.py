"""Check actual wheel PI functions with signed feedforward and output headroom."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
BALANCE = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE'
source = (BALANCE / 'balance.c').read_bytes().decode('gbk')


def function(signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


test = r'''
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include "wheel_output.h"
#include "lateral_friction.h"
enum {Mec_Car, Mec_Car_V550, Diff_Car, Akm_Car, Tank_Car, Omni_Car};
static int Car_Mode=Mec_Car,start_clear,clear_state,clear_done_once;
static float Velocity_KP=300,Velocity_KI=300;
'''
test += '\n'.join(function(f'int Incremental_PI_{wheel} (') for wheel in 'ABCD')
test += r'''
int main(void) {
    assert(WheelOutput_Feedforward(0,1)==0);
    assert(WheelOutput_Feedforward(.001f,1)==0);
    assert(WheelOutput_Feedforward(.04f,0)==0);
    assert(fabsf(WheelOutput_Feedforward(.04f,1)-1160)<.01f);
    assert(fabsf(WheelOutput_Feedforward(-.04f,1)+1160)<.01f);
    assert(fabsf(WheelOutput_Feedforward(.1f,1)-1400)<.01f);
    assert(WheelOutput_Feedforward(10,1)==6000);
    assert(WheelOutput_Feedforward(-10,1)==-6000);
    assert(WheelOutput_Apply(0,0,1)==0);
    assert(WheelOutput_Apply(-1000,0,1)==-1000); /* Zero-speed PI braking survives. */
    assert(WheelOutput_Apply(1000,.1f,0)==1000); /* Other chassis unchanged. */
    int (*pi[])(float,float)={Incremental_PI_A,Incremental_PI_B,Incremental_PI_C,Incremental_PI_D};
    for(unsigned wheel=0;wheel<4;wheel++) {
        int pwm=pi[wheel](0,.04f);
        assert(pwm==24);
        int output=WheelOutput_Apply(pwm,.04f,1);
        assert(output==1184); /* Base drive exists before a long integral wait. */
        int side=LateralFriction_Apply(wheel,output,.04f,0,0,.04f);
        assert(side==(wheel==1 || wheel==2 ? 1684 : 1184));
        for(unsigned n=0;n<2000;n++)pwm=pi[wheel](0,.5f);
        assert(pwm<=WheelOutput_ClampPi(1e6f,.5f,1));
        assert(WheelOutput_Apply(pwm,.5f,1)==16700);
        for(unsigned n=0;n<2000;n++)pwm=pi[wheel](0,-.5f);
        assert(pwm>=WheelOutput_ClampPi(-1e6f,-.5f,1));
        assert(WheelOutput_Apply(pwm,-.5f,1)==-16700);
        pwm=pi[wheel](0,0);
        assert(pwm>=-16700 && pwm<=16700);
        assert(WheelOutput_Apply(pwm,0,1)==pwm);
    }
    assert(WheelOutput_Apply(30000,.5f,1)==16700);
    assert(WheelOutput_Apply(-30000,-.5f,1)==-16700);
    puts("PASS: four real PI loops, immediate signed speed feedforward, zero-speed braking, B/C side assist and total-output anti-windup limits");
}
'''
compiler = os.environ.get('ROUTE_HOST_CC') or shutil.which('gcc')
if not compiler:
    raise SystemExit('Set ROUTE_HOST_CC to a C compiler')
with tempfile.TemporaryDirectory(prefix='wheel-feedforward-') as temp:
    build = Path(temp)
    for name in ('wheel_output.h', 'lateral_friction.h'):
        shutil.copyfile(BALANCE / name, build / name)
    (build / 'test.c').write_text(test, encoding='utf-8')
    exe = build / 'test.exe'
    subprocess.run([compiler, '-std=c99', '-Wall', '-Wextra', '-Werror', '-I', str(build),
                    str(build / 'test.c'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
