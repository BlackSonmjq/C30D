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
static int PS2_ON_Flag,route_owned,vision_owned;
static unsigned char RouteIO_ControlOwned(void){return route_owned;}
static unsigned char VisionAlign_ControlOwned(void){return vision_owned;}
static float float_abs(float v){return fabsf(v);}
static float target_limit_float(float v,float lo,float hi){return v<lo?lo:v>hi?hi:v;}
'''+smooth+r'''
int main(void) {
    /* Compare with the upstream PD law over both signs and saturation. */
    for(int e=-100;e<=100;e++) for(int g=-20;g<=20;g++) {
        float error=e*.01f,rate=g*.02f,expected;
        if(fabsf(error)<=.035f) {
            if(fabsf(rate)<=.03f) expected=0;
            else expected=-.75f*rate;
        } else expected=1.8f*error-.75f*rate;
        if(expected>.25f) expected=.25f;
        if(expected<-.25f) expected=-.25f;
        HeadingControl_Reset();
        assert(fabsf(HeadingControl_Output(error,rate,.01f)-expected)<1e-6f);
        assert(HeadingControl_GetIntegralOutput()==0);
    }
    HeadingControl_Reset();
    for(int n=0;n<10000;n++)
        assert(fabsf(HeadingControl_Output(.1f,0,.01f)-.18f)<1e-6f);
    assert(HeadingControl_GetIntegralOutput()==0);
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
    /* Route startup reaches its existing limits promptly, with no overshoot. */
    PS2_ON_Flag=0;route_owned=1;vision_owned=0;
    smooth_control.VX=smooth_control.VY=smooth_control.VZ=0;
    for(int n=0;n<10;n++) Smooth_control(.28f,-.48f,.2f);
    assert(fabsf(smooth_control.VX-.28f)<1e-6f);
    assert(smooth_control.VY<-.299f && smooth_control.VY>-.301f);
    assert(smooth_control.VZ>.099f && smooth_control.VZ<.101f);
    for(int n=0;n<6;n++) Smooth_control(.28f,-.48f,.2f);
    assert(fabsf(smooth_control.VY+.48f)<1e-6f);
    Smooth_control(.12f,-.18f,0);
    assert(fabsf(smooth_control.VX-.12f)<1e-6f && fabsf(smooth_control.VY+.18f)<1e-6f);
    Smooth_control(0,0,0);
    assert(smooth_control.VX==0 && smooth_control.VY==0);
    for(int mode=0;mode<3;mode++) {
        route_owned=mode!=0;vision_owned=mode==1;PS2_ON_Flag=mode==2;
        smooth_control.VX=smooth_control.VY=smooth_control.VZ=0;
        Smooth_control(.48f,.48f,.2f);
        float expected=mode==2?.05f:.01f;
        assert(fabsf(smooth_control.VX-expected)<1e-6f && fabsf(smooth_control.VY-expected)<1e-6f);
    }
    /* Integrated gyro path retains startup lock and produces damping. */
    HeadingControl_Reset();assert(HeadingControl_Update(0)==0);
    assert(HeadingControl_Update(751)<0);
    puts("PASS: REBUILD PD law, no integral drift, navigation ramp and immediate angular braking/reversal");
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
