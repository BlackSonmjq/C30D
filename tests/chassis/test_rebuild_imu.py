"""Exercise the actual REBUILD sensor task and single-source odometry yaw on ARM."""
from pathlib import Path
import os
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26'
BUILD = ROOT / 'tests/mango/.build/rebuild-imu'
BUILD.mkdir(parents=True, exist_ok=True)
task = (FW / 'BALANCE/imu_task.c').read_text(encoding='utf-8')
header = (FW / 'BALANCE/imu_task.h').read_bytes().decode('gbk')
odom = (FW / 'USER/chassis_odometry.c').read_text(encoding='utf-8')
def without_includes(text):
    return '\n'.join(line for line in text.splitlines() if not line.startswith('#include'))
registers = sorted(set(re.findall(r'\b(?:REG_ADD_|MPU6050_RA_)[A-Z_]+', task)))
definitions = '\n'.join(f'#define {name} {i+1}' for i, name in enumerate(registers))
fixture = r'''
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <setjmp.h>
#include "nav_config.h"
#include "heading_control.c"
#include "chassis_odometry.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define RATE_100_HZ 100
#define CONTROL_DELAY 1000
#define F2T(x) 10
#define ROUTE_ODOM_MAX_GAP_MS 100U
#define ROUTE_ODOM_FORWARD_SCALE 1.0f
#define ROUTE_ODOM_LATERAL_SCALE 1.0f
#define I2C_ADD_ICM20948 1
#define REG_VAL_WIA 234
#define MPU6050_DEFAULT_ADDRESS 104
#define devAddr 104
static struct { float Encoder; } MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;
static struct { unsigned Time_count; } SysVal={2000};
static int Flag_Stop,Led_Count,index=-1;
static uint32_t now=1000;
static jmp_buf finished;
static float float_abs(float v) { return fabsf(v); }
static uint32_t RouteIO_Millis(void) { return now; }
static uint32_t getSysTickCnt(void) { return now; }
static int ICM20948_getDeviceID(void) { return index==150 || index==362 ? 0:234; }
static int MPU6050_getDeviceID(void) { return index==150 || index==362 ? 0:104; }
static uint8_t I2C_ReadOneByte(uint8_t address,uint8_t reg);
static void vTaskDelayUntil(uint32_t *wake,unsigned period);
'''
mock = r'''
static IMU_HeadingFrame published[402];
static void vTaskDelayUntil(uint32_t *wake,unsigned period) {
    (void)wake;(void)period;
    if(index>=0) Imu_ReadHeading(&published[index]);
    if(++index==402) longjmp(finished,1);
    now+=index==363?200:10;
    MOTOR_A.Encoder=index<50?.1f:0;
}
static uint8_t I2C_ReadOneByte(uint8_t address,uint8_t reg) {
    int value=0;
    (void)address;
    switch(reg) {
    case REG_ADD_GYRO_XOUT_H: case MPU6050_RA_GYRO_XOUT_H: value=10;break;
    case REG_ADD_GYRO_YOUT_H: case MPU6050_RA_GYRO_YOUT_H: value=-6;break;
    case REG_ADD_GYRO_ZOUT_H: case MPU6050_RA_GYRO_ZOUT_H:
        value=index==361?1000:(index%2?100:101);break;
    case REG_ADD_ACCEL_ZOUT_H: case MPU6050_RA_ACCEL_ZOUT_H: value=16384;break;
    default: break;
    }
    /* Register low-byte addresses are distinct; return the matching high sample's low byte. */
    switch(reg) {
    case REG_ADD_GYRO_XOUT_L: case MPU6050_RA_GYRO_XOUT_L: return 10;
    case REG_ADD_GYRO_YOUT_L: case MPU6050_RA_GYRO_YOUT_L: return (uint8_t)-6;
    case REG_ADD_GYRO_ZOUT_L: case MPU6050_RA_GYRO_ZOUT_L:
        return (uint8_t)(index==361?1000:(index%2?100:101));
    default: return (uint8_t)((uint16_t)value>>8);
    }
}
int main(void) {
    for(int device=0;device<2;device++) {
        index=-1;now=1000;
        if(setjmp(finished)==0) sample_task(device);
        for(int i=0;i<350;i++) assert(!published[i].ready);
        assert(published[350].ready && Flag_Stop==0);
        assert(fabsf(published[350].rate_rad_s-(.5f/65.5f*.0174532925f))<1e-7f);
        assert(published[361].ready && published[361].rate_rad_s>.23f);
        assert(!published[362].ready && !published[363].ready && published[364].ready);
        assert(published[362].yaw_rad==published[361].yaw_rad);
        assert(published[363].yaw_rad==published[361].yaw_rad);
    }
    now=10000;heading.ready=1;heading.tick_ms=now;heading.yaw_rad=1;
    ChassisOdom_Reset(now);
    ChassisOdom_UpdateDelta(0,0,0,now,1);
    now+=10;heading.tick_ms=now;heading.yaw_rad=1.1f;
    ChassisOdom_UpdateDelta(10,0,32767,now,1);
    ChassisOdomFrame p;ChassisOdom_Read(&p);
    assert(p.valid && p.yaw_rad==heading.yaw_rad);
    assert(fabsf(p.x_mm-10*cosf(1.05f))<.001f);
    assert(fabsf(p.y_mm-10*sinf(1.05f))<.001f);
    now+=100;ChassisOdom_UpdateDelta(0,0,0,now,1);ChassisOdom_Read(&p);assert(!p.valid);
    heading.ready=0;heading.tick_ms=now;
    ChassisOdom_UpdateDelta(0,0,0,now,1);ChassisOdom_Read(&p);assert(!p.valid);
    puts("PASS: both IMUs, motion/ID calibration reset, fractional bias, invalid/gap readiness, single yaw and 100ms expiry");
    return 0;
}
'''
for name in ('heading_control.c', 'heading_control.h'):
    shutil.copyfile(FW / 'BALANCE' / name, BUILD / name)
shutil.copyfile(ROOT / 'tests/chassis/stubs/sys.h', BUILD / 'sys.h')
test = fixture + definitions + '\n' + without_includes(header) + '\n'
test += without_includes(task) + '\n' + without_includes(odom) + '\n' + mock
(BUILD / 'test.c').write_text(test, encoding='utf-8')
environment = dict(os.environ, ARM_TEST_MAX_INSTRUCTIONS='200000000')
subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'), str(BUILD / 'test.c')],
               cwd=ROOT, env=environment, check=True)
