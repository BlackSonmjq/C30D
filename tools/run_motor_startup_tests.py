"""Test real PWM/SPL functions and CAL=0/1 startup on a host model. No board access."""
from pathlib import Path
import argparse
import json
import os
import re
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"OBJ/tests/motor-startup"
OUT.mkdir(parents=True,exist_ok=True)
CC=os.environ.get("HOST_CC") or shutil.which("gcc")
sys.stdout.reconfigure(encoding="utf-8",errors="replace")
if not CC:
    raise SystemExit("GCC required")


def read(path):
    return Path(path).read_bytes().decode("gbk",errors="replace")


def function(source,name):
    match=re.search(r"\b(?:void|int|u8)\s+"+re.escape(name)+r"\s*\(",source)
    if not match:
        raise ValueError(name)
    begin=source.index("{",match.start()); end=begin+1; depth=1
    while depth:
        depth+=(source[end]=="{")-(source[end]=="}"); end+=1
    return source[match.start():end]


def compile_run(label,code,extra=(),includes=()):
    source=OUT/(label+".c"); exe=OUT/(label+".exe")
    source.write_text(code,encoding="utf-8")
    command=[CC,"-std=c99","-O2","-Wall","-Wextra","-Werror"]
    for include in includes:
        command += ["-I",str(include)]
    subprocess.run(command+[str(source),*map(str,extra),"-lm","-o",str(exe)],check=True)
    result=subprocess.run([exe],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    log=result.stdout.decode("utf-8",errors="replace")
    (OUT/(label+".log")).write_text(log,encoding="utf-8")
    return result.returncode,log


def hardware_model(motor_source,balance_source):
    chip=read(ROOT/"USER/stm32f4xx.h")
    timer_header=read(ROOT/"FWLIB/inc/stm32f4xx_tim.h")
    gpio_header=read(ROOT/"FWLIB/inc/stm32f4xx_gpio.h")
    rcc_header=read(ROOT/"FWLIB/inc/stm32f4xx_rcc.h")
    structures={name:body for body,name in re.findall(r"typedef struct\s*\{(.*?)\}\s*(\w+)\s*;",chip+timer_header+gpio_header,re.S)}
    enums="\n".join("typedef enum {"+body+"} "+name+";" for body,name in re.findall(
        r"typedef enum\s*\{(.*?)\}\s*(\w+)\s*;",gpio_header,re.S))
    constants={}
    for header in (chip,timer_header,gpio_header,rcc_header,read(ROOT/"FWLIB/src/stm32f4xx_tim.c")):
        for name,value in re.findall(r"^#define\s+((?:TIM_|GPIO_|RCC_|CCMR_|CCER_)\w+)\s+([^\n]+)",header,re.M):
            value=value.split("/*",1)[0].split("//",1)[0].strip()
            if "\\" not in value and "(" not in name:
                constants[name]=value
    prefix='''#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef enum {DISABLE=0,ENABLE=1} FunctionalState;
#define __IO volatile
#define assert_param(x) ((void)0)
'''+enums+"\n"
    for name in ("TIM_TypeDef","GPIO_TypeDef","TIM_TimeBaseInitTypeDef","TIM_OCInitTypeDef","GPIO_InitTypeDef"):
        prefix+="typedef struct {"+structures[name]+"} "+name+";\n"
    prefix+="\n".join("#define "+name+" "+value for name,value in constants.items())+"\n"
    prefix+="static TIM_TypeDef timers[15]; static GPIO_TypeDef port_b,port_e,port_d;\n"
    prefix+="\n".join(f"#define TIM{i} (&timers[{i}])" for i in range(1,15))+"\n"
    prefix+='''#define GPIOB (&port_b)
#define GPIOE (&port_e)
#define GPIOD (&port_d)
static unsigned bad_setup;
static void RCC_AHB1PeriphClockCmd(uint32_t mask,FunctionalState state) {(void)mask;(void)state;}
static void RCC_APB2PeriphClockCmd(uint32_t mask,FunctionalState state) {(void)mask;(void)state;}
static void GPIO_ResetBits(GPIO_TypeDef *gpio,uint16_t pins) {gpio->BSRRH=pins;gpio->ODR&=~(uint32_t)pins;}
'''
    vendor=read(ROOT/"FWLIB/src/stm32f4xx_tim.c")
    for name in ("TIM_TimeBaseStructInit","TIM_TimeBaseInit","TIM_OCStructInit","TIM_OC1Init","TIM_OC2Init","TIM_OC3Init","TIM_OC4Init",
                 "TIM_OC1PreloadConfig","TIM_OC2PreloadConfig","TIM_OC3PreloadConfig","TIM_OC4PreloadConfig",
                 "TIM_ARRPreloadConfig","TIM_CtrlPWMOutputs","TIM_Cmd","TIM_SelectOCxM","TIM_CCxCmd"):
        body=function(vendor,name)
        if name=="TIM_SelectOCxM":
            # SPL uses 32-bit peripheral addresses; keep its offset logic on a 64-bit host.
            body=re.sub(r"uint32_t\s+tmp\s*=\s*0;","uintptr_t tmp=0;",body)
            body=body.replace("(uint32_t) TIMx","(uintptr_t) TIMx").replace("(uint32_t)TIMx","(uintptr_t)TIMx")
        if re.fullmatch(r"TIM_OC[1234]Init",name):
            body=body.replace("{","{\n if(TIM_OCInitStruct->TIM_Pulse!=0 || TIM_OCInitStruct->TIM_OutputNState!=TIM_OutputNState_Disable || TIM_OCInitStruct->TIM_OCNPolarity!=TIM_OCNPolarity_High || TIM_OCInitStruct->TIM_OCIdleState!=TIM_OCIdleState_Reset || TIM_OCInitStruct->TIM_OCNIdleState!=TIM_OCNIdleState_Reset) bad_setup++;",1)
        if name=="TIM_TimeBaseInit":
            body=body.replace("{","{\n if(TIM_TimeBaseInitStruct->TIM_RepetitionCounter!=0) bad_setup++;",1)
        prefix+=body+"\n"
    vendor=read(ROOT/"FWLIB/src/stm32f4xx_gpio.c")
    for name in ("GPIO_StructInit","GPIO_Init","GPIO_PinAFConfig"):
        prefix+=function(vendor,name)+"\n"
    actual=read(motor_source).replace('#include "motor.h"',"")
    # Deterministic arbitrary stack contents expose omitted fields; real init code follows unchanged.
    actual=re.sub(r"(TIM_(?:OC|TimeBase)InitTypeDef\s+(\w+);)",
                  lambda m:m[1]+"\n memset(&"+m[2]+",0xA5,sizeof("+m[2]+"));",actual)
    prefix+="#define HAS_GATE "+str(int("void Motor_OutputEnable(" in actual))+"\n"+actual+"\n"
    definitions=read(ROOT/"HARDWARE/motor.h")
    for name,value in re.findall(r"^#define\s+(PWM[ABCD][12]|Servo_PWM)\s+([^\r\n]+)",definitions,re.M):
        prefix+="#define "+name+" "+value+"\n"
    prefix+=function(read(balance_source),"Set_Pwm")+"\n"
    return prefix+HARDWARE_TEST


HARDWARE_TEST=r'''
#define CHECK(c) do {if(!(c)){printf("FAIL %s at %d\n",#c,__LINE__);return 1;}}while(0)
static int pin_level(GPIO_TypeDef *port,unsigned pin,TIM_TypeDef *timer,unsigned ch,unsigned tick) {
 unsigned mode=(port->MODER>>(2*pin))&3U;
 unsigned shift=ch*4,ocmode; uint32_t ccr;
 if(mode==GPIO_Mode_OUT) return (port->ODR>>pin)&1U;
 if(mode!=GPIO_Mode_AF || !(timer->CCER&(1U<<shift))) return 0;
 ocmode=((ch<2?timer->CCMR1:timer->CCMR2)>>((ch%2)*8))&0x70;
 ccr=ch==0?timer->CCR1:ch==1?timer->CCR2:ch==2?timer->CCR3:timer->CCR4;
 return ((ocmode==TIM_ForcedAction_InActive?0:ocmode==TIM_OCMode_PWM1?(tick<ccr):1)^((timer->CCER>>(shift+1))&1U));
}
static int all_low(void) {
 unsigned t;for(t=0;t<16800;t+=137) {
  if(pin_level(GPIOB,8,TIM10,0,t)||pin_level(GPIOB,9,TIM11,0,t)||
     pin_level(GPIOE,5,TIM9,0,t)||pin_level(GPIOE,6,TIM9,1,t)||
     pin_level(GPIOE,9,TIM1,0,t)||pin_level(GPIOE,11,TIM1,1,t)||
     pin_level(GPIOE,13,TIM1,2,t)||pin_level(GPIOE,14,TIM1,3,t)) return 0;
 }return 1;
}
int main(void) {
 (void)all_low; (void)GPIO_ResetBits;
 memset(timers,0,sizeof(timers));memset(&port_b,0,sizeof(port_b));memset(&port_e,0,sizeof(port_e));
 port_b.ODR=port_e.ODR=0xFFFF;
#if HAS_GATE
 Motor_OutputInit();CHECK(all_low());
#endif
 TIM1_PWM_Init(16799,0);TIM9_PWM_Init(16799,0);TIM10_PWM_Init(16799,0);TIM11_PWM_Init(16799,0);
 CHECK(bad_setup==0);CHECK(TIM1->RCR==0);CHECK(TIM11->BDTR==0);
#if HAS_GATE
 CHECK(all_low());Motor_OutputEnable(0);CHECK(all_low());
 Set_Pwm(14000,-13000,12000,-11000,0);CHECK(all_low());
 Motor_OutputEnable(0);CHECK(all_low());
 Set_Pwm(0,0,0,0,0);Motor_OutputEnable(1);
 CHECK(((TIM1->CCMR1)&0x70)==TIM_OCMode_PWM1);
 CHECK((TIM1->CCER&0x1111)==0x1111 && (TIM9->CCER&0x11)==0x11 && (TIM10->CCER&1) && (TIM11->CCER&1));
 Set_Pwm(5000,0,0,0,0);
 CHECK(pin_level(GPIOB,8,TIM10,0,13000)!=pin_level(GPIOB,9,TIM11,0,13000));
 Motor_OutputEnable(0);CHECK(all_low());
 Set_Pwm(-15000,15000,-15000,15000,0);CHECK(all_low());
 puts("PASS actual motor/SPL code: poisoned-stack init, eight pins low before start, explicit PWM enable and stop gate");
#else
 CHECK(0);
#endif
 return 0;
}
'''

START_TEST=r'''
#include <stdio.h>
#include <string.h>
#include "nav_core.h"
#include "nav_button.h"
#define CHECK(c) do {if(!(c)){printf("FAIL %s at %d\n",#c,__LINE__);return 1;}}while(0)
static int scenario(int held) {
 NavCore nav;NavInput in;NavLidar frame;NavButton key;unsigned t,event;
 memset(&in,0,sizeof(in));memset(&frame,0,sizeof(frame));
 Nav_Init(&nav,&nav_calibration,NAV_CALIBRATED);NavButton_Init(&key);
 in.enabled=in.imu_ready=1;
 frame.mm[NAV_FRONT]=(uint16_t)(nav_calibration.lane[0].side_front_mm+400);
 frame.mm[NAV_LEFT]=(uint16_t)nav_calibration.left_stop_mm;
 frame.mm[NAV_RIGHT]=(uint16_t)(nav_calibration.lr_sum_mm-nav_calibration.left_stop_mm);
 for(t=0;t<10000;t+=10) {
  in.now_ms=t;if(t%100==0){frame.sequence++;frame.received_ms=t;}
  event=NavButton_Update(&key,held&&t<1000,Nav_IsActive(&nav),nav.state==NAV_READY,t);
  in.start=event==NAV_KEY_START;Nav_Tick(&nav,&in,&frame);
  CHECK(!in.start && !nav.output.drive_enabled);
  CHECK(nav.output.vx==0 && nav.output.vy==0 && nav.output.wz==0);
 }
 CHECK(nav.state==(NAV_CALIBRATED?NAV_READY:NAV_BOOT));
 if(NAV_CALIBRATED) {
  for(t=10000;t<10200;t+=10) {
   in.now_ms=t;if(t%100==0){frame.sequence++;frame.received_ms=t;}
   event=NavButton_Update(&key,t<10100,Nav_IsActive(&nav),1,t);
   in.start=event==NAV_KEY_START;Nav_Tick(&nav,&in,&frame);
  }
  CHECK(Nav_IsActive(&nav));
  in.start=0;in.stop=1;Nav_Tick(&nav,&in,&frame);CHECK(!nav.output.drive_enabled);
 }
 return 0;
}
int main(void){if(scenario(0)||scenario(1))return 1;printf("PASS CAL=%d: no key/boot-held key cannot start; explicit press/release required\n",NAV_CALIBRATED);return 0;}
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-before",action="store_true")
    args=parser.parse_args()
    if args.check_before:
        before=Path(json.loads((ROOT/"build/motor-startup-baseline.json").read_text(encoding="utf-8"))["backup"])
        code,log=compile_run("before-hardware",hardware_model(before/"HARDWARE/motor.c",before/"BALANCE/balance.c"))
        if code==0:
            raise SystemExit("Expected original uninitialized PWM failure was not reproduced")
        print("PASS original PWM defect reproduced: "+log.strip())
    code,log=compile_run("after-hardware",hardware_model(ROOT/"HARDWARE/motor.c",ROOT/"BALANCE/balance.c"))
    print(log,end="")
    if code:
        return code
    names=("nav_core.c","nav_core.h","nav_button.c","nav_button.h","nav_calibration.c","nav_plan.h","nav_can.h","nav_config.h")
    for calibrated in (0,1):
        test_dir=OUT/f"cal{calibrated}"
        test_dir.mkdir(exist_ok=True)
        for name in names:
            shutil.copy2(ROOT/"USER"/name,test_dir/name)
        config=(test_dir/"nav_config.h").read_text(encoding="utf-8")
        config,count=re.subn(r"(^#define\s+NAV_CALIBRATED\s+)\d+",r"\g<1>"+str(calibrated),config,flags=re.M)
        assert count==1
        (test_dir/"nav_config.h").write_text(config,encoding="utf-8")
        code,log=compile_run(f"cal{calibrated}-startup",START_TEST,
            (test_dir/"nav_core.c",test_dir/"nav_button.c",test_dir/"nav_calibration.c"),(test_dir,))
        print(log,end="")
        if code:
            return code
        board_sources=("nav_stm32.c","nav_can.c","nav_ble_uart.c","nav_vofa.c","remote_control.c")
        for name in (*board_sources,"nav_stm32.h","nav_vofa.h","remote_control.h"):
            shutil.copy2(ROOT/"USER"/name,test_dir/name)
        source=(ROOT/"tests/remote_mode_test.c").read_text(encoding="utf-8")
        code,log=compile_run(f"cal{calibrated}-remote-mode",source,
            tuple(test_dir/name for name in (*board_sources,"nav_core.c","nav_button.c","nav_calibration.c")),
            (ROOT/"tests/fixtures/remote_board",test_dir))
        print(log,end="")
        if code:
            return code
    balance=function(read(ROOT/"BALANCE/balance.c"),"Balance_task")
    inactive=balance[balance.index("if(!command.drive_enabled"):balance.index("was_enabled=1")]
    assert inactive.index("Motor_OutputEnable(0)")<inactive.index("NavBoard_PublishDrive(0)")
    active=balance[balance.index("was_enabled=1"):]
    assert active.index("Set_Pwm(")<active.index("Motor_OutputEnable(1)")<active.index("NavBoard_PublishDrive(1)")
    system=function(read(ROOT/"BALANCE/system.c"),"systemInit")
    assert system.index("Motor_OutputInit();")<system.index("TIM1_PWM_Init(")
    assert system.index("TIM11_PWM_Init(")<system.index("Motor_OutputEnable(0);")
    print("PASS integration: early pin hold, standby gate off, output enabled only after motor command commit")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
