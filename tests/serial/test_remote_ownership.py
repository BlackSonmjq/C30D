"""Run the production TIM8 takeover guard against USB ownership and bad pulses."""
from pathlib import Path
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26'
BUILD = ROOT / 'tests/mango/.build/remote-ownership'
BUILD.mkdir(parents=True, exist_ok=True)
source = (FW / 'HARDWARE/timer.c').read_bytes().decode('gbk')
start = source.index('if(Remoter_Ch2>1600')
opening = source.index('{', start)
end = source.index('}', opening) + 1
guard = source[start:end]
test = r'''
#include <assert.h>
#include <stdio.h>
static int Remoter_Ch2=1800,PS2_ON_Flag,Remote_ON_Flag,APP_ON_Flag,CAN_ON_Flag,Usart1_ON_Flag,Usart5_ON_Flag;
static struct {unsigned Time_count;} SysVal={2000};
#define CONTROL_DELAY 1000
static void capture(void) {
''' + guard + r'''
}
int main(void) {
    PS2_ON_Flag=1;
    for(unsigned i=0;i<1000;++i)capture();
    assert(PS2_ON_Flag && !Remote_ON_Flag);
    PS2_ON_Flag=0;Remoter_Ch2=2500;capture();assert(!Remote_ON_Flag);
    Remoter_Ch2=1600;capture();assert(!Remote_ON_Flag);
    Remoter_Ch2=1800;SysVal.Time_count=0;capture();assert(!Remote_ON_Flag);
    SysVal.Time_count=2000;capture();assert(Remote_ON_Flag && !PS2_ON_Flag);
    puts("PASS: TIM8 cannot steal active USB control; invalid PWM cannot arm; idle valid RC still works");
}
'''
path = BUILD / 'test.c'
path.write_text(test, encoding='utf-8')
env = os.environ.copy()
env['PYTHONPATH'] = str(ROOT / 'tests/camera/.build/python_deps')
subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'), str(path)],
               cwd=ROOT, env=env, check=True)
