"""Legacy helper regression with explicit opt-in; production uses global wheel output."""
from pathlib import Path
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/lateral_friction.h'
source = r'''
#include <assert.h>
#include <stdio.h>
#include "lateral_friction.h"
int main(void) {
    unsigned left=LATERAL_FRICTION_LEFT_MOTOR, right=LATERAL_FRICTION_RIGHT_MOTOR;
    assert(LateralFriction_Apply(left,1000,.1f,0,0,.1f)==3500);
    assert(LateralFriction_Apply(right,-1000,-.1f,0,0,-.1f)==-3500);
    for(unsigned wheel=0;wheel<4;wheel++) {
        if (wheel!=left && wheel!=right)
            assert(LateralFriction_Apply(wheel,1000,.1f,0,0,.1f)==1000);
    }
    int half=LateralFriction_Apply(left,1000,.1f,.04f,0,.1f);
    assert(half>=2249 && half<=2251);
    assert(LateralFriction_Apply(left,1000,.1f,.08f,0,.1f)==1000);
    assert(LateralFriction_Apply(left,1000,.1f,0,.1f,0)==1000);
    assert(LateralFriction_Apply(left,1000,.1f,0,0,0)==1000);
    assert(LateralFriction_Apply(left,1000,.1f,0,.1f,.1f)==1000);
    assert(LateralFriction_Apply(left,1000,0,0,0,.1f)==1000);
    assert(LateralFriction_Apply(left,-1000,.1f,.02f,0,.1f)==-1000);
    assert(LateralFriction_Apply(left,1000,-.1f,-.02f,0,-.1f)==1000);
    assert(LateralFriction_Apply(left,1000,.02f,.03f,0,.1f)==1000);
    assert(LateralFriction_Apply(right,-1000,-.02f,-.03f,0,-.1f)==-1000);
    assert(LateralFriction_Apply(left,16600,.1f,0,0,.1f)==16700);
    assert(LateralFriction_Apply(right,-16600,-.1f,0,0,-.1f)==-16700);
    assert(LateralFriction_Apply(left,1000,.1f,0,0,.1f)==3500); /* No accumulation. */
    puts("PASS: front wheel selection, signs, low-speed fade, stop/forward/braking guards and PWM limits");
}
'''
compiler = os.environ.get('ROUTE_HOST_CC') or shutil.which('gcc')
with tempfile.TemporaryDirectory(prefix='lateral-friction-') as temp:
    build = Path(temp) if compiler else ROOT / 'tests/mango/.build/lateral-friction'
    build.mkdir(parents=True, exist_ok=True)
    header = HEADER.read_text(encoding='utf-8')
    # Cover every possible distinct front-wheel pair, including swapped wiring.
    for left in range(4):
        for right in range(4):
            if left == right:
                continue
            active = header
            for name, value in (
                ('LATERAL_FRICTION_ENABLE', '1'),
                ('LATERAL_FRICTION_FRONT_MASK', str((1 << left) | (1 << right)) + 'U'),
                ('LATERAL_FRICTION_LEFT_MOTOR', str(left) + 'U'),
                ('LATERAL_FRICTION_RIGHT_MOTOR', str(right) + 'U'),
            ):
                active, count = re.subn(r'(?m)^(#define\s+' + name + r'\s+)\S+',
                                        lambda m: m[1]+value, active)
                assert count == 1
            (build / HEADER.name).write_text(active, encoding='utf-8')
            (build / 'test.c').write_text(source, encoding='utf-8')
            if not compiler:
                subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'),
                                str(build / 'test.c')], cwd=ROOT, check=True, stdout=subprocess.PIPE)
                continue
            exe = build / f'test-{left}-{right}.exe'
            subprocess.run([compiler, '-std=c99', '-Wall', '-Wextra', '-Werror', '-I', str(build),
                            str(build / 'test.c'), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True, stdout=subprocess.PIPE)
print('PASS: 12 front-wheel wiring combinations; lateral friction and braking/stop guards')
