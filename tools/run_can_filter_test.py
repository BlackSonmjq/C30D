"""Run the real CAN filter block/IRQ against a host register model, no hardware."""
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
OUT = ROOT / "OBJ/tests/can-filter"
OUT.mkdir(parents=True, exist_ok=True)
CC = os.environ.get("HOST_CC") or shutil.which("gcc")
if not CC:
    raise SystemExit("GCC required")

MODEL = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nav_can.h"
typedef uint8_t u8;
typedef uint32_t u32;
#define CAN_FMR_FINIT 1U
typedef struct { uint32_t FR1,FR2; } FilterBank;
static struct { uint32_t FMR,FA1R,FS1R,FM1R,FFA1R; FilterBank sFilterRegister[28]; } regs;
#define CAN1 (&regs)
#define CHECK(c) do { if(!(c)) { printf("FAIL %s (line %d)\n",#c,__LINE__); return 1; } } while(0)
static unsigned queued;
static NavLidar published;
static u8 CAN1_Msg_Pend(u8 fifo) { return fifo==0?(u8)queued:0; }
static void CAN1_Rx_Msg(u8 fifo,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *bytes) {
    const u8 packet[8]={3,0xE8,7,0xD0,0x0B,0xB8,0x0F,0xA0};
    (void)fifo; *id=0x301; *ide=0; *rtr=0; *len=8;
    memcpy(bytes,packet,8); queued=0;
}
static void NavBoard_CanRx(u32 id,u8 ide,u8 rtr,u8 len,const u8 *bytes) {
    if(NavCan_Decode(id,ide,rtr,len,bytes,&published)) published.sequence++;
}
static void apply_real_filter_block(void) {
%FILTER_BLOCK%
}
%REAL_IRQ%
static int accepts_can1_fifo0(u32 id) {
    if((CAN1->FMR&CAN_FMR_FINIT) || ((CAN1->FMR>>8)&63U)==0 ||
       !(CAN1->FA1R&1U) || !(CAN1->FS1R&1U) || (CAN1->FM1R&1U) || (CAN1->FFA1R&1U)) return 0;
    return (((id<<21)^CAN1->sFilterRegister[0].FR1)&CAN1->sFilterRegister[0].FR2)==0;
}
int main(void) {
    const uint32_t reserved=0x2A1C0000U;
    memset(CAN1,0,sizeof(regs)); CAN1->FMR=0x2A1C0E01U;
    apply_real_filter_block();
    CHECK(((CAN1->FMR>>8)&63U)==14U);
    CHECK(!(CAN1->FMR&CAN_FMR_FINIT));
    CHECK((CAN1->FMR&~0x3F01U)==reserved);
    CHECK(accepts_can1_fifo0(0x301));
    queued=1; CAN1_RX0_IRQHandler();
    CHECK(!queued && published.sequence==1);
    CHECK(published.mm[NAV_FRONT]==2000 && published.mm[NAV_BACK]==4000);
    CHECK(published.mm[NAV_LEFT]==1000 && published.mm[NAV_RIGHT]==3000);
    puts("PASS reset state: real filter block admits 0x301; real IRQ routes L/F/R/B to F/B/L/R");
    CAN1->FMR=reserved; CAN1->FA1R=4; CAN1->FS1R=4;
    CAN1->FM1R=5; CAN1->FFA1R=5;
    CAN1->sFilterRegister[2].FR1=0x12345678;
    CAN1->sFilterRegister[2].FR2=0x87654321;
    apply_real_filter_block();
    CHECK(accepts_can1_fifo0(0x301));
    CHECK(CAN1->FA1R==5 && CAN1->FS1R==5 && CAN1->FM1R==4 && CAN1->FFA1R==4);
    CHECK((CAN1->FMR&~0x3F01U)==reserved);
    CHECK(CAN1->sFilterRegister[2].FR1==0x12345678 && CAN1->sFilterRegister[2].FR2==0x87654321);
    puts("PASS reinitialization: CAN1 split restored, mask/FIFO0 selected, other banks preserved");
    return 0;
}
'''


def run_model(source, label):
    text = source.read_bytes().decode("gbk")
    begin = text.index("/*** Filter initialization")
    block = text[begin:text.index("#if CAN1_RX0_INT_ENABLE", begin)]
    begin = text.index("void CAN1_RX0_IRQHandler(void)")
    irq = text[begin:text.index("#endif", begin)]
    generated = OUT / f"{label}.c"
    generated.write_text(MODEL.replace("%FILTER_BLOCK%", block).replace("%REAL_IRQ%", irq), encoding="utf-8")
    exe = OUT / f"{label}.exe"
    subprocess.run([CC,"-std=c99","-O2","-Wall","-Wextra","-Werror",
                    "-I",str(ROOT/"USER"),str(generated),str(ROOT/"USER/nav_can.c"),"-o",str(exe)],check=True)
    result = subprocess.run([exe],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    log = result.stdout.decode("utf-8",errors="replace")
    (OUT / f"{label}.log").write_text(log,encoding="utf-8")
    return result.returncode,log


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-before",action="store_true",help="Also prove the saved pre-fix source fails")
    args=parser.parse_args()
    if args.check_before:
        saved=json.loads((ROOT/"build/can-filter-baseline.json").read_text(encoding="utf-8"))
        code,log=run_model(Path(saved["backup"])/"HARDWARE/can.c","before")
        if code==0:
            raise SystemExit("Expected pre-fix CAN filter regression did not reproduce")
        print("PASS original receive failure reproduced: "+log.strip())
    code,log=run_model(ROOT/"HARDWARE/can.c","after")
    print(log,end="")
    raise SystemExit(code)
