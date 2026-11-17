"""Exercise actual manual driving and heading controller with simulated IMU.

Only chassis I/O, pose reads and the RTOS tick are mocked. The heading target,
gyro damping, safety checks and output limits execute production C code.
"""
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26'
BUILD = ROOT / 'tests/mango/.build/manual-imu'
BUILD.mkdir(parents=True, exist_ok=True)
source = (FW / 'BALANCE/balance.c').read_bytes().decode('gbk')
start = source.index('static void ManualMecanum_Drive(')
opening = source.index('{', start)
depth, end = 1, opening + 1
while depth:
    depth += (source[end] == '{') - (source[end] == '}')
    end += 1
helper = source[start:end]
test = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "heading_control.c"
typedef struct { float yaw_rad; uint32_t tick_ms; u8 valid; } ChassisOdomFrame;
#define ROUTE_ODOM_MAX_GAP_MS 100U
#define CONTROL_DELAY 1000U
static u8 manual_heading_locked;
static float manual_heading_target;
static int EN=1,Flag_Stop;
static float Voltage=12,Move_X,Move_Y,Move_Z,dx,dy,dz;
static struct { float VX,VY,VZ; } smooth_control;
static struct { unsigned Time_count; } SysVal={2000};
static struct { struct { short z; } gyro; } imu;
static unsigned now=10000;
static ChassisOdomFrame pose={.4f,10000,1};
static void ChassisOdom_Read(ChassisOdomFrame *p){*p=pose;}
static unsigned xTaskGetTickCount(void){return now;}
static void Drive_Motor(float x,float y,float z){dx=x;dy=y;dz=z;}
''' + helper + r'''
int main(void) {
    ManualMecanum_Drive(.2f,-.1f);
    assert(dx==.2f && dy==-.1f && dz==0 && manual_heading_locked);
    /* Measured clockwise/counterclockwise rate produces opposite correction
       even before a yaw position error has accumulated. */
    imu.gyro.z=751; ManualMecanum_Drive(.2f,-.1f);
    assert(dx==.2f && dy==-.1f && dz<-.10f);
    imu.gyro.z=-751; ManualMecanum_Drive(.2f,-.1f);
    assert(dx==.2f && dy==-.1f && dz>0);
    imu.gyro.z=0;pose.yaw_rad=.5f;ManualMecanum_Drive(.2f,-.1f);
    assert(fabsf(dz+.12f)<.001f); /* Stronger correction for 0.1 rad yaw drift. */
    pose.yaw_rad=.3f;ManualMecanum_Drive(.2f,-.1f);assert(dz>0);
    imu.gyro.z=32767;ManualMecanum_Drive(.2f,0);
    assert(dz<0 && fabsf(dz)<=HEADING_CONTROL_MAX_ANGULAR_VELOCITY);
    EN=0;ManualMecanum_Drive(.2f,-.1f);
    assert(dx==0 && dy==0 && dz==0 && !manual_heading_locked);
    EN=1;imu.gyro.z=0;ManualMecanum_Drive(.2f,-.1f);
    assert(dz==0); /* Re-entry latches the current heading. */
    now+=101;ManualMecanum_Drive(.2f,-.1f);
    assert(dx==0 && dy==0 && dz==0 && !manual_heading_locked);
    puts("PASS: manual translation retains IMU yaw hold, rate damping, limits and disable/stale stop");
}
'''
for name in ('heading_control.c', 'heading_control.h'):
    shutil.copyfile(FW / 'BALANCE' / name, BUILD / name)
shutil.copyfile(ROOT / 'tests/chassis/stubs/sys.h', BUILD / 'sys.h')
path = BUILD / 'test_manual_imu.c'
path.write_text(test, encoding='utf-8')
subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'), str(path)], cwd=ROOT, check=True)
