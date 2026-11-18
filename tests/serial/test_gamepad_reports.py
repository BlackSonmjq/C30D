"""Exercise production USB report guards and failed-decode neutralization."""
from pathlib import Path
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26'
BUILD = ROOT / 'tests/mango/.build/gamepad-reports'
BUILD.mkdir(parents=True, exist_ok=True)


def extract(path, signature):
    source = path.read_bytes().decode('gbk')
    start = source.index(signature)
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


app = FW / 'HARDWARE/USB/USB_HOST/App'
test = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
typedef int USBH_StatusTypeDef;
enum {USBH_OK, USBH_FAIL, UnKnown_Dev, Xbox360, PS2_USB_Wiredless, PS2_USB_Wired, PS2_USB_WiredV2};
typedef struct { uint8_t *buf; } Fifo;
typedef struct { uint16_t length; Fifo fifo; } HID_HandleTypeDef;
typedef struct {void *pData;} ActiveClass;
typedef struct {ActiveClass *pActiveClass;} USBH_HandleTypeDef;
static uint8_t GamePadReportData[64], input[64];
static uint16_t GamePadReceivedLength;
static int gamepad_brand=Xbox360, decoded, fifo_ready=1;
static struct {int ready;} GamePadDebug;
static struct {unsigned LX,LY,RX,RY,LT,RT,StartFlag;} pad={127,127,127,127,0,0,1},*GamePadInterface=&pad;
static int PS2_ON_Flag,Remote_ON_Flag,APP_ON_Flag,CAN_ON_Flag,Usart1_ON_Flag,Usart5_ON_Flag;
static struct {unsigned Time_count;} SysVal={2000};
#define CONTROL_DELAY 1000
static unsigned USBH_HID_FifoRead(Fifo *f,uint8_t *p,unsigned n) {
    (void)f;if(!fifo_ready)return 0;memcpy(p,input,n);return n;
}
static void Xbox360_gamepad_Decode(USBH_HandleTypeDef *h,uint8_t *p,unsigned n) {
    (void)h;(void)n;++decoded;pad.LY=p[9];
}
static void Wired_USB_PS2gamepad_Decode(USBH_HandleTypeDef *h,uint8_t *p,unsigned n) {
    (void)h;(void)n;++decoded;pad.LY=p[4];
}
static void Wired_USB_V2_PS2gamepad_Decode(USBH_HandleTypeDef *h,uint8_t *p,unsigned n) {
    (void)h;(void)n;++decoded;pad.LY=p[1];
}
'''
test += extract(app / 'usbh_hid_GamePad.c', 'USBH_StatusTypeDef USBH_HID_PS2_Decode(')
test += extract(app / 'usb_host.c', 'void USBH_HID_EventCallback(')
test += r'''
int main(void) {
    HID_HandleTypeDef hid={64,{input}};ActiveClass active={&hid};USBH_HandleTypeDef host={&active};
    memset(input,185,sizeof(input));
    /* Same stale endpoint buffer: a short packet may not turn LY=185 into motion. */
    for(unsigned n=0;n<14;++n) {
        GamePadReceivedLength=n;pad.LY=185;pad.RX=0;PS2_ON_Flag=0;
        USBH_HID_EventCallback(&host);
        assert(decoded==0 && pad.LY==127 && pad.RX==127 && !PS2_ON_Flag && !GamePadDebug.ready);
    }
    GamePadReceivedLength=20;USBH_HID_EventCallback(&host);
    assert(decoded==1 && pad.LY==185 && PS2_ON_Flag && GamePadDebug.ready);
    GamePadReceivedLength=3;USBH_HID_EventCallback(&host);
    assert(decoded==1 && pad.LY==127 && pad.RX==127 && PS2_ON_Flag);
    GamePadReceivedLength=65;USBH_HID_EventCallback(&host);assert(decoded==1);
    for(gamepad_brand=PS2_USB_Wired;gamepad_brand<=PS2_USB_WiredV2;++gamepad_brand) {
        int before=decoded;GamePadReceivedLength=6;USBH_HID_EventCallback(&host);
        assert(decoded==before && pad.LY==127);
        GamePadReceivedLength=7;USBH_HID_EventCallback(&host);
        assert(decoded==before+1 && pad.LY==185);
    }
    fifo_ready=0;USBH_HID_EventCallback(&host);assert(pad.LY==127);
    puts("PASS: short/stale USB reports cannot decode axes or arm control; valid Xbox/PS2 reports retain control");
    return 0;
}
'''
path = BUILD / 'test.c'
path.write_text(test, encoding='utf-8')
environment = os.environ.copy()
environment['PYTHONPATH'] = str(ROOT / 'tests/camera/.build/python_deps')
subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'), str(path)],
               cwd=ROOT, env=environment, check=True)
