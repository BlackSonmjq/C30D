"""Test production manual mappings and heading latch with mocked chassis I/O."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
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
#include <string.h>
typedef uint8_t u8;
enum {ROUTE_LIDAR_FRONT,ROUTE_LIDAR_BEHIND};
typedef struct {uint16_t mm[4];uint32_t sequence;uint8_t fresh;} RouteLidarFrame;
static unsigned char RouteIO_ControlOwned(void){return 0;}
static unsigned char VisionAlign_ControlOwned(void){return 0;}
static void RouteIO_ReadLidar(RouteLidarFrame *p){*p=(RouteLidarFrame){{1000,1000,1000,1000},1,1};}
static float Velocity_KP,Velocity_KI;
static unsigned LineDiffParam=50;
static int flash_words[4]={-1,-1,-1,-1};
static int Read_Flash(unsigned index){return flash_words[index];}
typedef struct { float yaw_rad; uint32_t tick_ms; u8 valid; } ChassisOdomFrame;
#define ROUTE_ODOM_MAX_GAP_MS 100U
#define CONTROL_DELAY 1000U
#define HEADING_CONTROL_ENABLE 1
#define HEADING_CONTROL_GYRO_FULL_SCALE_DPS 500.0f
#define HEADING_CONTROL_RECOVERY_ERROR_RAD 0.034906585f
#define HEADING_CONTROL_RECOVERY_RATE_RADPS 0.087266463f
#define MANUAL_RC_MAX_MMPS 480.0f
static u8 manual_heading_locked;
static u8 manual_heading_pole;
static float manual_heading_target;
static int EN=1,Flag_Stop,Car_Mode=1,RC_Velocity=500;
enum { Mec_Car=1, Mec_Car_V550=2 };
static float Voltage=12,Move_X,Move_Y,Move_Z,dx,dy,dz;
static struct { float VX,VY,VZ; } smooth_control;
static int PS2_ON_Flag=1;
static float float_abs(float x){return fabsf(x);}
static float target_limit_float(float x,float low,float high){return x<low?low:x>high?high:x;}
void Smooth_control(float x,float y,float z);
static struct { unsigned Time_count; } SysVal={2000};
static struct { struct { short z; } gyro; } imu;
static struct { int LY,RX; } gamepad={127,127}, *GamePadInterface=&gamepad;
static int Remoter_Ch2=1500,Remoter_Ch1=1500;
static unsigned now=10000,resets;
static ChassisOdomFrame pose={1.0f,10000,1};
static void ChassisOdom_Read(ChassisOdomFrame *p){*p=pose;}
static unsigned xTaskGetTickCount(void){return now;}
static void HeadingControl_Reset(void){++resets;}
static float HeadingControl_NormalizeAngle(float value){return atan2f(sinf(value),cosf(value));}
static float HeadingControl_HoldYaw(float target,float yaw,short gyro,float forward){(void)gyro;(void)forward;return target-yaw;}
static void Drive_Motor(float x,float y,float z){dx=x;dy=y;dz=z;Smooth_control(x,y,z);}
static int target_limit_int(int x,int low,int high){return x<low?low:x>high?high:x;}
'''
main = r'''
int main(void){
  float stored=200;memcpy(&flash_words[0],&stored,4);FlashParam_Read();assert(RC_Velocity==480);
  stored=300;memcpy(&flash_words[0],&stored,4);FlashParam_Read();assert(RC_Velocity==480);
  stored=150;memcpy(&flash_words[0],&stored,4);FlashParam_Read();assert(RC_Velocity==150);
  stored=0;memcpy(&flash_words[0],&stored,4);FlashParam_Read();assert(RC_Velocity==0);
  stored=500;memcpy(&flash_words[0],&stored,4);FlashParam_Read();assert(RC_Velocity==480);
  RC_Velocity=500;
  Remoter_Ch2=2000;Remoter_Ch1=1500;remote();
  assert(fabsf(dx-.48f)<1e-6f && dy==0 && dz==0);
  Remoter_Ch2=1500;Remoter_Ch1=2000;remote();
  assert(dx==0 && fabsf(dy+.48f)<1e-6f);
  Remoter_Ch2=1000;Remoter_Ch1=1000;remote();
  assert(fabsf(dx+.48f)<1e-6f && fabsf(dy-.48f)<1e-6f);
  Remoter_Ch2=1500;Remoter_Ch1=1500;pose.yaw_rad=1.2f;remote();
  assert(dx==0 && dy==0 && dz<-.19f && resets==1);
  EN=0;remote();assert(dx==0 && dy==0 && dz==0 && !manual_heading_locked);
  EN=1;remote();assert(dz==0 && resets==2);
  gamepad.LY=254;gamepad.RX=0;ps2();
  assert(fabsf(dx-.48f)<1e-6f && fabsf(dy-.48f)<1e-6f);
  smooth_control.VX=.5f;smooth_control.VY=.5f;
  gamepad.LY=0;gamepad.RX=254;ps2();
  assert(fabsf(dx+.48f)<1e-6f && fabsf(dy+.48f)<1e-6f);
  assert(smooth_control.VX<0 && smooth_control.VY<0);
  assert(smooth_control.VX>=-.051f && smooth_control.VY>=-.051f);
  gamepad.LY=130;gamepad.RX=125;ps2();assert(dx==0 && dy==0);
  now+=101;ps2();assert(dx==0 && dy==0 && dz==0 && !manual_heading_locked);
  puts("PASS: radio axes, reversed PS2 mapping LY-127/127-RX, immediate reversal ramp, yaw latch and stale IMU stop");
}
'''
compiler = os.environ.get('ROUTE_HOST_CC') or shutil.which('gcc')
with tempfile.TemporaryDirectory(prefix='manual-control-') as temp:
    build = Path(temp)
    test = stub + helper + '\nstatic void VisionAlign_Drive(float x,float y){ManualMecanum_Drive(x,y,0U);}' + '\nvoid ps2(void){' + branches[0] + '}\n'
    test += 'void remote(void){int Threshold=100;static u8 thrice=0;' + branches[1] + '}\n'
    test += block(source.index('void Smooth_control(')) + '\n' + block(source.index('void FlashParam_Read(')) + '\n' + main
    if not compiler:
        # Reuse the repository's ARMCC/Unicorn runner on Windows without GCC.
        arm_build = ROOT / 'tests/mango/.build'
        arm_build.mkdir(parents=True, exist_ok=True)
        arm_source = arm_build / 'manual_controls.c'
        arm_source.write_text(test, encoding='utf-8')
        subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'), str(arm_source)], cwd=ROOT, check=True)
        raise SystemExit(0)
    (build / 'test.c').write_text(test, encoding='utf-8')
    exe = build / 'test.exe'
    subprocess.run([compiler, '-std=c99', '-Wall', '-Wextra', '-Werror', str(build / 'test.c'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
