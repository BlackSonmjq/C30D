"""Test production heavy-load heading damping and angular command smoothing."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26'
source = (FW / 'BALANCE/balance.c').read_bytes().decode('gbk')
start = source.index('void Smooth_control(float vx,float vy,float vz)')
opening = source.index('{', start)
depth, end = 1, opening+1
while depth:
    depth += (source[end] == '{') - (source[end] == '}')
    end += 1
smooth = source[start:end]
test = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "heading_control.c"
static struct { float VX,VY,VZ; } smooth_control;
static int PS2_ON_Flag;
static float float_abs(float v){return fabsf(v);}
static float target_limit_float(float v,float lo,float hi){return v<lo?lo:v>hi?hi:v;}
'''+smooth+r'''
int main(void) {
    HeadingControl_Reset();
    assert(HeadingControl_HoldYaw(0,0,0)==0);
    float small=HeadingControl_HoldYaw(.011f,0,0);
    float medium=HeadingControl_HoldYaw(.05f,0,0);
    assert(small>0 && small<.02f && medium>small && medium<.08f);
    assert(HeadingControl_HoldYaw(-.011f,0,0)<0);
    assert(fabsf(HeadingControl_HoldYaw(2,0,0)-.50f)<1e-6f);
    assert(fabsf(HeadingControl_HoldYaw(-2,0,0)+.50f)<1e-6f);
    /* Already turning toward target: brake before the angle crosses zero. */
    assert(HeadingControl_Output(.05f,.2f,.01f)<0);
    assert(HeadingControl_Output(-.05f,-.2f,.01f)>0);
    assert(fabsf(HeadingControl_Output(.05f,.05f*1.20f/.55f,.01f))<1e-6f);
    assert(HeadingControl_Output(0,.2f,.01f)<0);
    assert(HeadingControl_Output(.005f,.01f,.01f)==0);
    /* Public entry uses the measured gyro, not an error difference. */
    assert(HeadingControl_HoldYaw(.05f,0,751)<0);
    for(int ps2=0;ps2<2;ps2++) {
        PS2_ON_Flag=ps2;
        smooth_control.VZ=.3f;
        Smooth_control(.1f,.1f,-.05f);
        assert(fabsf(smooth_control.VZ+.05f)<1e-6f);
        Smooth_control(0,0,0);assert(smooth_control.VZ==0);
        Smooth_control(0,0,.2f);
        assert(smooth_control.VZ>0 && smooth_control.VZ<.2f);
        smooth_control.VZ=-.3f;
        Smooth_control(0,0,.05f);assert(fabsf(smooth_control.VZ-.05f)<1e-6f);
        Smooth_control(0,0,.02f);assert(fabsf(smooth_control.VZ-.02f)<1e-6f);
    }
    /* Integrated gyro path retains startup lock and produces damping. */
    HeadingControl_Reset();assert(HeadingControl_Update(0)==0);
    assert(HeadingControl_Update(751)<0);
    puts("PASS: tapered startup assist, bounded gains, pre-crossing damping, no false output at PD balance, immediate angular braking/reversal");
}
'''
compiler = os.environ.get('ROUTE_HOST_CC') or shutil.which('gcc')
if not compiler:
    build = ROOT / 'tests/mango/.build/heading-heavy'
    build.mkdir(parents=True, exist_ok=True)
    (build / 'test.c').write_text(test, encoding='utf-8')
    for name in ('heading_control.c', 'heading_control.h'):
        shutil.copyfile(FW / 'BALANCE' / name, build / name)
    shutil.copyfile(ROOT / 'tests/chassis/stubs/sys.h', build / 'sys.h')
    subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'), str(build / 'test.c')], cwd=ROOT, check=True)
    raise SystemExit(0)
with tempfile.TemporaryDirectory(prefix='heading-heavy-') as temp:
    build = Path(temp)
    (build / 'test.c').write_text(test, encoding='utf-8')
    for name in ('heading_control.c', 'heading_control.h'):
        shutil.copyfile(FW / 'BALANCE' / name, build / name)
    shutil.copyfile(ROOT / 'tests/chassis/stubs/sys.h', build / 'sys.h')
    exe = build / 'test.exe'
    subprocess.run([compiler, '-std=c99', '-Wall', '-Wextra', '-Werror', '-I', str(build),
                    str(build / 'test.c'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
