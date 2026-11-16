"""Test production manual mappings and heading latch with mocked chassis I/O."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/balance.c').read_bytes().decode('gbk')


def block(start):
    opening = source.index('{', start)
    depth, end = 1, opening+1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


helper = block(source.index('static void ManualMecanum_Drive('))
branches = []
for function in ('void PS2_control(void)', 'void Remote_Control(void)'):
    branches.append(block(source.index('if (Car_Mode == Mec_Car', source.index(function))))
stub = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
typedef uint8_t u8;
typedef struct { float yaw_rad; uint32_t tick_ms; u8 valid; } ChassisOdomFrame;
#define ROUTE_ODOM_MAX_GAP_MS 100U
#define CONTROL_DELAY 1000U
#define HEADING_CONTROL_ENABLE 1
static u8 manual_heading_locked;
static float manual_heading_target;
static int EN=1,Flag_Stop,Car_Mode=1,RC_Velocity=500;
enum { Mec_Car=1, Mec_Car_V550=2 };
static float Voltage=12,Move_X,Move_Y,Move_Z,dx,dy,dz;
static struct { unsigned Time_count; } SysVal={2000};
static struct { struct { short z; } gyro; } imu;
static struct { int LY,RX; } gamepad={127,127}, *GamePadInterface=&gamepad;
static int Remoter_Ch2=1500,Remoter_Ch1=1500;
static unsigned now=10000,resets;
static ChassisOdomFrame pose={1.0f,10000,1};
static void ChassisOdom_Read(ChassisOdomFrame *p){*p=pose;}
static unsigned xTaskGetTickCount(void){return now;}
static void HeadingControl_Reset(void){++resets;}
static float HeadingControl_HoldYaw(float target,float yaw,short gyro){(void)gyro;return target-yaw;}
static void Drive_Motor(float x,float y,float z){dx=x;dy=y;dz=z;}
static int target_limit_int(int x,int low,int high){return x<low?low:x>high?high:x;}
'''
main = r'''
int main(void){
  Remoter_Ch2=2000;Remoter_Ch1=1500;remote();
  assert(fabsf(dx-.5f)<1e-6f && dy==0 && dz==0);
  Remoter_Ch2=1500;Remoter_Ch1=2000;remote();
  assert(dx==0 && fabsf(dy+.5f)<1e-6f);
  Remoter_Ch2=1000;Remoter_Ch1=1000;remote();
  assert(fabsf(dx+.5f)<1e-6f && fabsf(dy-.5f)<1e-6f);
  Remoter_Ch2=1500;Remoter_Ch1=1500;pose.yaw_rad=1.2f;remote();
  assert(dx==0 && dy==0 && dz<-.19f && resets==1);
  EN=0;remote();assert(dx==0 && dy==0 && dz==0 && !manual_heading_locked);
  EN=1;remote();assert(dz==0 && resets==2);
  gamepad.LY=254;gamepad.RX=0;ps2();
  assert(fabsf(dx-.5f)<1e-6f && fabsf(dy-.5f)<1e-6f);
  gamepad.LY=0;gamepad.RX=254;ps2();
  assert(fabsf(dx+.5f)<1e-6f && fabsf(dy+.5f)<1e-6f);
  gamepad.LY=130;gamepad.RX=125;ps2();assert(dx==0 && dy==0);
  now+=101;ps2();assert(dx==0 && dy==0 && dz==0 && !manual_heading_locked);
  puts("PASS: radio/PS2 axes, diagonal motion, deadband, yaw latch, disable/reacquire and stale IMU stop");
}
'''
compiler = os.environ.get('ROUTE_HOST_CC') or shutil.which('gcc')
if not compiler:
    raise SystemExit('Set ROUTE_HOST_CC to a C compiler')
with tempfile.TemporaryDirectory(prefix='manual-control-') as temp:
    build = Path(temp)
    test = stub + helper + '\nvoid ps2(void){' + branches[0] + '}\n'
    test += 'void remote(void){int Threshold=100;static u8 thrice=0;' + branches[1] + '}\n' + main
    (build / 'test.c').write_text(test, encoding='utf-8')
    exe = build / 'test.exe'
    subprocess.run([compiler, '-std=c99', '-Wall', '-Wextra', '-Werror', str(build / 'test.c'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
