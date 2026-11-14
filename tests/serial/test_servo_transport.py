"""Run S2 through the real compiled UART4 driver, parser and state machine.

Only peripheral readiness, external replies, RTOS clock/scheduler and printing
are modeled. No Servo_* API is intercepted. Build EIDE firmware before running:
  python tests/serial/test_servo_transport.py
"""
from pathlib import Path
import runpy, struct
from unicorn import UC_HOOK_CODE
from unicorn.arm_const import *
ROOT=Path(__file__).resolve().parents[2]
support=runpy.run_path(str(ROOT / 'tests/serial/test_task_commands.py'))
sym=support['symbols']
UART4=0x40004c00

def simulate(mode=2, ignore_outward=0, extra_ack=False, inject_error=False,
             noisy=False, short_mode=False, miss_mode_queries=0):
    cpu=support['fresh'](); cpu.mem_map(0xe000e000,0x4000)
    tick=0; position=541; sends=0; frame=bytearray(); wire=[]; replies=[]; logs=[]; cr1_checks=0; errors=0
    scheduler_depth=0; mode_queries=0
    hook_names=['xTaskGetTickCount','USART_GetFlagStatus','USART_SendData','USART_Cmd',
                'USART_HalfDuplexCmd','GPIO_ReadInputDataBit','any_printf',
                'xTaskGetSchedulerState','vTaskSuspendAll','xTaskResumeAll']
    addresses={sym[n]&~1:n for n in hook_names}
    def word(addr): return int.from_bytes(cpu.mem_read(addr,4),'little')
    def put(addr,value): cpu.mem_write(addr,struct.pack('<I',value))
    def cstring(addr):
        data=bytearray()
        while True:
            b=bytes(cpu.mem_read(addr+len(data),1))
            if b==b'\0': return data.decode('ascii')
            data.extend(b)
    def hook(uc,address,size,user):
        nonlocal tick,position,sends,frame,cr1_checks,scheduler_depth,mode_queries
        name=addresses.get(address)
        if name is None: return
        a=uc.reg_read(UC_ARM_REG_R0); b=uc.reg_read(UC_ARM_REG_R1); r=0
        if name=='xTaskGetTickCount': r=tick
        elif name=='xTaskGetSchedulerState': r=0 if scheduler_depth else 2
        elif name=='vTaskSuspendAll':
            assert not uc.reg_read(UC_ARM_REG_PRIMASK)
            scheduler_depth+=1
        elif name=='xTaskResumeAll':
            assert scheduler_depth==1 and not uc.reg_read(UC_ARM_REG_PRIMASK)
            assert not (word(UART4+12)&8)
            scheduler_depth-=1
        elif name=='USART_GetFlagStatus':
            assert a==UART4
            r=1 # Model UART ready; TC boundary is covered by the driver unit test.
        elif name=='USART_Cmd':
            put(a+12,(word(a+12)|0x2000) if b else (word(a+12)&~0x2000))
        elif name=='USART_HalfDuplexCmd':
            put(a+20,(word(a+20)|8) if b else (word(a+20)&~8))
        elif name=='GPIO_ReadInputDataBit': r=1
        elif name=='any_printf': logs.append(cstring(b))
        elif name=='USART_SendData':
            assert a==UART4 and (word(a+12)&12)==8, hex(word(a+12))
            assert scheduler_depth==1
            assert uc.reg_read(UC_ARM_REG_PRIMASK)==int(b==ord('!'))
            cr1_checks+=1; frame.append(b&255)
            if b==ord('!'):
                command=frame.decode('ascii'); wire.append((tick,command)); frame=bytearray()
                if command=='#002PMOD!':
                    mode_queries+=1
                    if extra_ack: replies.append('#OK!')
                    if noisy: replies.extend(['#002PMOD!','#003PMOD2!','#002PMOD0!'])
                    if mode is not None and mode_queries>miss_mode_queries:
                        replies.append(f'#002MOD{mode}!' if short_mode else f'#002PMOD{mode}!')
                elif command=='#002PRAD!':
                    if extra_ack: replies.append('#OK!')
                    if noisy: replies.extend(['#002PRAD!','#003P1833!','#002P12x3!','#002P0499!'])
                    replies.append(f'#002P{position:04d}!')
                elif command.startswith('#002P') and 'T' in command:
                    # '#002P1833T0500!' => pulse indices 5..8.
                    target=int(command[5:9])
                    if target in (1833,2500):
                        sends+=1
                        if sends>ignore_outward: position=target
                    else: position=541
                else: raise AssertionError(command)
        uc.reg_write(UC_ARM_REG_R0,r); uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
    cpu.hook_add(UC_HOOK_CODE,hook)
    def call(name):
        cpu.reg_write(UC_ARM_REG_SP,0x2001fff0); cpu.reg_write(UC_ARM_REG_LR,0x1001)
        cpu.emu_start(sym[name]|1,0x1000,count=500000)
    def deliver(reply):
        nonlocal errors
        for index,ch in enumerate(reply):
            if word(UART4+12)&36 !=36: continue
            status=0x20
            if inject_error and not errors and reply.startswith('#002P0') and index==3:
                status|=8; errors+=1
            put(UART4,status); put(UART4+4,ord(ch)); call('UART4_IRQHandler')
    call('Servo_Init')
    support['receive'](cpu,'S2')
    for _ in range(300):
        call('ServoDebug_Process')
        for reply in replies: deliver(reply)
        replies.clear()
        if not support['read'](cpu,'servo_debug_busy'): break
        tick+=50
    else: raise AssertionError('did not finish')
    assert not scheduler_depth
    if mode is None: assert mode_queries==3
    if miss_mode_queries and mode is not None: assert mode_queries==miss_mode_queries+1
    moves=[cmd for _,cmd in wire if 'T' in cmd]
    return moves, logs, tick, cr1_checks

normal=['#002P0500T0500!','#002P1833T0500!','#002P0500T0500!']
m,l,t,c=simulate(); assert m==normal,(m,l)
print('PASS: real S2 move encoding, UART4 direction and scheduler protection for',c,'bytes')
m,l,t,c=simulate(ignore_outward=1); assert m==normal[:2]+normal[1:],m
m,l,t,c=simulate(ignore_outward=99); assert m==normal[:1]+[normal[1]]*4+normal[-1:],m
assert any('POSITION TIMEOUT' in f for f in l)
print('PASS: P0541 retry recovery, retry limit and timeout return using the real UART driver')
m,l,t,c=simulate(mode=None); assert m==[] and any('MODE NOT CONFIRMED' in f for f in l)
m,l,t,c=simulate(miss_mode_queries=2); assert m==normal,m
m,l,t,c=simulate(mode=7); assert m==[] and any('not position mode' in f for f in l)
print('PASS: mode confirmed before motion, three missing-mode reads cancel, later reply recovers')
m,l,t,c=simulate(extra_ack=True); assert m==normal,m
m,l,t,c=simulate(extra_ack=True,noisy=True); assert m==normal,m
m,l,t,c=simulate(extra_ack=True,noisy=True,short_mode=True); assert m==normal,m
print('PASS: back-to-back ACK/echo/wrong-ID/malformed frames cannot hide mode or position replies')
m,l,t,c=simulate(inject_error=True); assert m==normal,m
print('PASS: injected receive overrun recovers on the next position query (10 integration scenarios)')
