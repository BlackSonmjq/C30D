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
#include "route_io.h"
typedef struct { float yaw_rad; uint32_t tick_ms; u8 valid; } ChassisOdomFrame;
#define ROUTE_ODOM_MAX_GAP_MS 100U
#define CONTROL_DELAY 1000U
static u8 manual_heading_locked;
static u8 manual_heading_pole=1;
static float manual_heading_target;
static int EN=1,Flag_Stop;
static float Voltage=12,Move_X,Move_Y,Move_Z,dx,dy,dz;
static struct { float VX,VY,VZ; } smooth_control;
static struct { unsigned Time_count; } SysVal={2000};
static struct { struct { short z; } gyro; } imu;
static unsigned now=10000;
static ChassisOdomFrame pose={.4f,10000,1};
static RouteLidarFrame manual_lidar={{1000,1000,1000,1000},1,1};
void RouteIO_ReadLidar(RouteLidarFrame *data) { *data=manual_lidar; }
static void ChassisOdom_Read(ChassisOdomFrame *p){*p=pose;}
static unsigned xTaskGetTickCount(void){return now;}
static void Drive_Motor(float x,float y,float z){dx=x;dy=y;dz=z;}
''' + helper + r'''
int main(void) {
    ManualMecanum_Drive(.2f,-.1f,0U);
    assert(dx==.2f && dy==-.1f && dz==0 && manual_heading_locked);
    /* Measured clockwise/counterclockwise rate produces opposite correction
       even before a yaw position error has accumulated. */
    imu.gyro.z=751; ManualMecanum_Drive(.2f,-.1f,0U);
    assert(dx==.2f && dy==-.1f && dz<-.10f && smooth_control.VZ==dz);
    imu.gyro.z=-751; ManualMecanum_Drive(.2f,-.1f,0U);
    assert(dx==.2f && dy==-.1f && dz>0);
    imu.gyro.z=0;pose.yaw_rad=.5f;ManualMecanum_Drive(.2f,-.1f,0U);
    assert(fabsf(dz+.2404f)<.001f); /* Stronger correction for 0.1 rad yaw drift. */
    pose.yaw_rad=.3f;ManualMecanum_Drive(.2f,-.1f,0U);assert(dz>0);
    imu.gyro.z=32767;ManualMecanum_Drive(.2f,0,0U);
    assert(dz<0 && fabsf(dz)<=HEADING_CONTROL_MAX_ANGULAR_VELOCITY);
    EN=0;ManualMecanum_Drive(.2f,-.1f,0U);
    assert(dx==0 && dy==0 && dz==0 && !manual_heading_locked);
    EN=1;imu.gyro.z=0;ManualMecanum_Drive(.2f,-.1f,0U);
    assert(dz==0); /* Re-entry latches the current heading. */
    now+=101;ManualMecanum_Drive(.2f,-.1f,0U);
    assert(dx==0 && dy==0 && dz==0 && !manual_heading_locked);
    now=pose.tick_ms;
    ManualMecanum_Drive(0,.4f,0U);
    pose.yaw_rad=manual_heading_target+.35f; smooth_control.VY=.4f;
    ManualMecanum_Drive(.1f,.4f,0U);
    assert(dx==.1f && dy==.4f && dz<0);
    pose.yaw_rad=manual_heading_target-.174533f; smooth_control.VY=-.4f;
    ManualMecanum_Drive(0,-.4f,0U);
    assert(dx==0 && dy==-.4f && dz>0);
    manual_heading_pole=0;
    ManualMecanum_Drive(.1f,-.4f,0U); assert(dx==.1f && dy==-.4f && dz>0);
    manual_heading_pole=1;
    pose.yaw_rad=manual_heading_target;
    ManualMecanum_Drive(0,.4f,0U); assert(dy==.4f);
    manual_lidar.mm[ROUTE_LIDAR_FRONT]=45;smooth_control.VX=.2f;
    ManualMecanum_Drive(.2f,.1f,0U);assert(dx==0 && dy==.1f && smooth_control.VX==0);
    ManualMecanum_Drive(-.2f,0,0U);assert(dx==-.2f);
    manual_lidar.mm[ROUTE_LIDAR_FRONT]=44;ManualMecanum_Drive(.2f,0,0U);assert(dx==0);
    manual_lidar.mm[ROUTE_LIDAR_FRONT]=80;smooth_control.VX=.2f;
    ManualMecanum_Drive(.2f,0,0U);assert(fabsf(dx-.07f)<.0001f && smooth_control.VX<=.07001f);
    manual_lidar.mm[ROUTE_LIDAR_FRONT]=46;ManualMecanum_Drive(.2f,0,0U);assert(fabsf(dx-.002f)<.0001f);
    manual_lidar.mm[ROUTE_LIDAR_BEHIND]=45;smooth_control.VX=-.2f;
    ManualMecanum_Drive(-.2f,0,0U);assert(dx==0 && smooth_control.VX==0);
    manual_lidar.mm[ROUTE_LIDAR_FRONT]=1000;ManualMecanum_Drive(.2f,0,0U);assert(dx==.2f);
    manual_lidar.mm[ROUTE_LIDAR_BEHIND]=80;ManualMecanum_Drive(-.2f,0,0U);assert(fabsf(dx+.07f)<.0001f);
    manual_lidar.mm[ROUTE_LIDAR_BEHIND]=46;ManualMecanum_Drive(-.2f,0,0U);assert(fabsf(dx+.002f)<.0001f);
    manual_lidar.mm[ROUTE_LIDAR_FRONT]=0;ManualMecanum_Drive(.2f,0,0U);assert(dx==0);
    manual_lidar.fresh=0;smooth_control.VX=-.2f;imu.gyro.z=751;
    ManualMecanum_Drive(-.2f,.1f,0U);assert(dx==0 && dy==.1f && dz<0 && smooth_control.VX==0);
    puts("PASS: manual translation retains IMU yaw hold, rate damping, limits and disable/stale stop");
}
'''
for name in ('heading_control.c', 'heading_control.h'):
    shutil.copyfile(FW / 'BALANCE' / name, BUILD / name)
shutil.copyfile(ROOT / 'tests/chassis/stubs/sys.h', BUILD / 'sys.h')
shutil.copyfile(FW / 'USER/route_io.h', BUILD / 'route_io.h')
path = BUILD / 'test_manual_imu.c'
path.write_text(test, encoding='utf-8')
subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'), str(path)], cwd=ROOT, check=True)
