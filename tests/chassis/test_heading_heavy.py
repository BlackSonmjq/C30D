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
    /* A large stored effort from an earlier turn must not drive farther away. */
    heading_state.integral=-.335f;
    assert(HeadingControl_Output(.0376f,0,.01f)>0);
    heading_state.integral=.335f;
    assert(HeadingControl_Output(-.0376f,0,.01f)<0);
    HeadingControl_Reset();
    float small=HeadingControl_HoldYaw(.011f,0,0);
    float medium=HeadingControl_HoldYaw(.05f,0,0);
    assert(small>0 && small<.04f && medium>small && medium<.17f);
    assert(HeadingControl_HoldYaw(-.011f,0,0)<0);
    HeadingControl_Reset();
    assert(HeadingControl_HoldYaw(.015f,0,0)<.04f);
    HeadingControl_Reset();
    assert(HeadingControl_HoldYaw(.05236f,0,0)>=.159f);

    HeadingControl_Reset();
    assert(fabsf(HeadingControl_HoldYaw(2,0,0)-HEADING_CONTROL_MAX_ANGULAR_VELOCITY)<1e-6f);
    assert(fabsf(HeadingControl_HoldYaw(-2,0,0)+HEADING_CONTROL_MAX_ANGULAR_VELOCITY)<1e-6f);
    for(int n=0;n<10000;n++) HeadingControl_Output(2,0,.01f);
    assert(heading_state.integral==0);
    for(int n=0;n<10000;n++) HeadingControl_Output(-2,0,.01f);
    assert(heading_state.integral==0);
    /* Small persistent offsets must build bounded correction in both directions. */
    HeadingControl_Reset();
    float first=HeadingControl_Output(.02f,0,.01f),last=first;
    for(int n=0;n<3000;n++) last=HeadingControl_Output(.02f,0,.01f);
    assert(last>first+.15f && heading_state.integral<=HEADING_CONTROL_INTEGRAL_LIMIT);
    float stored=heading_state.integral;
    HeadingControl_Output(-.02f,0,.01f);
    assert(heading_state.integral<stored);
    HeadingControl_Reset();
    first=HeadingControl_Output(-.02f,0,.01f);
    for(int n=0;n<3000;n++) last=HeadingControl_Output(-.02f,0,.01f);
    assert(last<first-.15f);
    /* Already turning toward target: preserve pre-crossing braking. */
    HeadingControl_Reset();assert(HeadingControl_Output(.05f,.2f,.01f)<0);
    HeadingControl_Reset();assert(HeadingControl_Output(-.05f,-.2f,.01f)>0);
    HeadingControl_Reset();
    float balance_rate=.05f*(HEADING_CONTROL_KP+HEADING_CONTROL_KI*.01f)/HEADING_CONTROL_KD;
    assert(fabsf(HeadingControl_Output(.05f,balance_rate,.01f))<1e-6f);
    assert(HeadingControl_Output(0,.2f,.01f)<0);
    stored=heading_state.integral;
    assert(fabsf(HeadingControl_Output(.005f,.01f,.01f)-HEADING_CONTROL_KI*stored)<1e-6f);
    assert(heading_state.integral==stored);
    HeadingControl_Reset();assert(HeadingControl_Output(.005f,.01f,.01f)==0);
    /* Once across the target, release the old I effort inside tolerance. */
    for(int sign=-1;sign<=1;sign+=2) {
        heading_state.integral=sign*.4f;
        for(int n=0;n<100;n++) HeadingControl_Output(-sign*.005f,0,.01f);
        assert(fabsf(heading_state.integral)<.06f);
    }

    for(int sign=-1;sign<=1;sign+=2) {
        heading_state.integral=sign*.4f;
        for(int n=0;n<100;n++) HeadingControl_Output(sign*.002f,0,.01f);
        assert(fabsf(heading_state.integral)<.15f);
    }
    HeadingControl_Reset();assert(HeadingControl_HoldYaw(.05f,0,751)<0);
    /* Constant yaw disturbance leaves a residual with PD; I must reduce it. */
    for(int direction=-1;direction<=1;direction+=2) {
        float yaw=0,rate=0;
        HeadingControl_Reset();
        for(int n=0;n<4000;n++) {
            float command=HeadingControl_Output(-yaw,rate,.01f);
            rate+=(command+direction*.08f-rate)*.05f;
            yaw+=rate*.01f;
        }
        assert(fabsf(yaw)<.02f);
        /* Removing the external bias must let the retained integral unwind. */
        for(int n=0;n<4000;n++) {
            float command=HeadingControl_Output(-yaw,rate,.01f);
            rate+=(command-rate)*.05f;
            yaw+=rate*.01f;
        }
        assert(fabsf(yaw)<.02f && fabsf(rate)<.01f);
    }
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
    puts("PASS: sustained yaw bias rejection, integral anti-windup, damping, bounded correction and immediate angular braking/reversal");
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
    environment = dict(os.environ, ARM_TEST_MAX_INSTRUCTIONS='200000000')
    subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'), str(build / 'test.c')],
                   cwd=ROOT, env=environment, check=True)
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
