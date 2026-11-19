"""Execute the merged CAN filter and RX IRQ with register/FIFO mocks."""
from pathlib import Path
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26"
BUILD = ROOT / "tests/mango/.build/navigation_can"
BUILD.mkdir(parents=True, exist_ok=True)
source = (FW / "HARDWARE/can.c").read_bytes().decode("gbk")
start = source.index("/*** Filter initialization")
filter_block = source[start:source.index("#if CAN1_RX0_INT_ENABLE", start)]
start = source.index("void CAN1_RX0_IRQHandler(void)")
irq = source[start:source.index("#endif", start)]
model = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nav_can.c"
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#define CAN_FMR_FINIT 1U
#define CAN_Id_Standard 0U
#define CAN_RTR_Data 0U
#define CHECK(c) do { if(!(c)) {printf("FAIL line %d: %s\n",__LINE__,#c);return 1;} } while(0)
typedef struct {uint32_t FR1,FR2;} FilterBank;
static struct {uint32_t FMR,FA1R,FS1R,FM1R,FFA1R;FilterBank sFilterRegister[28];} registers;
#define CAN1 (&registers)
static struct {u16 left_mm,front_mm,right_mm,behind_mm;} lidar_frame;
static u8 lidar_pending;
static NavLidar published;
static unsigned queued,read_count;
static int Get_Charging_HardWare,CAN_ON_Flag,PS2_ON_Flag,Remote_ON_Flag,APP_ON_Flag;
static int Usart1_ON_Flag,Usart5_ON_Flag,command_lost_count,Car_Mode,Akm_Car;
static int charger_check,Allow_Recharge,Charging,Charging_Current,L_A,L_B,R_B,R_A,RED_STATE;
static float Move_X,Move_Y,Move_Z,Velocity_KP,Velocity_KI;
static float Recharge_Red_Move_X,Recharge_Red_Move_Y,Recharge_Red_Move_Z;
static float Vz_to_Akm_Angle(float x,float z){(void)x;return z;}
static u8 CAN1_Msg_Pend(u8 fifo){return fifo==0?(u8)queued:0;}
static void CAN1_Rx_Msg(u8 fifo,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *bytes)
{
    const u8 packet[8]={3,0xe8,7,0xd0,0x0b,0xb8,0x0f,0xa0};
    (void)fifo;*id=queued==2?0x302:0x301;*ide=*rtr=0;*len=8;
    memcpy(bytes,packet,8);--queued;++read_count;
}
static void RouteIO_ReceiveCan(u32 id,u8 ide,u8 rtr,u8 len,const u8 *bytes)
{if(NavCan_Decode(id,ide,rtr,len,bytes,&published))++published.sequence;}
static void CAN_Test_Parse(u8 ide,u8 rtr,u8 len,const u8 *bytes)
{(void)ide;(void)rtr;(void)len;(void)bytes;}
static void apply_filter(void)
{
%FILTER%
}
%IRQ%
int main(void)
{
    unsigned pass;
    for(pass=0;pass<2;++pass) {
        memset(CAN1,0,sizeof(registers));
        CAN1->FMR=pass?0x2a1c0000:0x2a1c0e01;
        CAN1->FM1R=CAN1->FFA1R=5;
        CAN1->sFilterRegister[2].FR1=0x12345678;
        apply_filter();
        CHECK(((CAN1->FMR>>8)&63)==14 && !(CAN1->FMR&CAN_FMR_FINIT));
        CHECK((CAN1->FMR&~0x3f01U)==0x2a1c0000);
        CHECK((CAN1->FA1R&1) && (CAN1->FS1R&1) && !(CAN1->FM1R&1) && !(CAN1->FFA1R&1));
        CHECK(CAN1->sFilterRegister[2].FR1==0x12345678);
    }
    queued=3;CAN1_RX0_IRQHandler();
    CHECK(!queued && read_count==3 && published.sequence==2 && lidar_pending);
    CHECK(published.mm[NAV_LEFT]==1000 && published.mm[NAV_FRONT]==2000);
    CHECK(published.mm[NAV_RIGHT]==3000 && published.mm[NAV_BACK]==4000);
    CHECK(lidar_frame.left_mm==1000 && lidar_frame.front_mm==2000 && lidar_frame.behind_mm==4000);
    puts("PASS: actual CAN filter reset/reinit, FIFO drain and 0x301 direction mapping");
    return 0;
}
'''
(BUILD / "test.c").write_text(model.replace("%FILTER%", filter_block).replace("%IRQ%", irq),
                              encoding="utf-8")
environment = os.environ.copy()
environment["PYTHONPATH"] = str(ROOT / "tests/camera/.build/python_deps")
subprocess.run([sys.executable, str(ROOT / "tests/mango/run_tests.py"), str(BUILD / "test.c")],
               cwd=ROOT, env=environment, check=True)
