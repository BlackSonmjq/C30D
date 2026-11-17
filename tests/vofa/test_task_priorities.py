"""Inspect actual task creation and UART IRQ setup by running the compiled image.

RTOS allocation and board interrupt setup are mocked; task entry functions and
their priority arguments come from the linked firmware, not copied constants.
No physical board is accessed.
"""
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests/camera/.build/python_deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2,
                               UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_SP)

firmware = Path(sys.argv[1]) if len(sys.argv) > 1 else (
    ROOT.parent / 'C30D-develop-EIDE/build/FreeRTOS/WHEELTEC.axf')
with firmware.open('rb') as stream:
    elf = ELFFile(stream)
    segments = [(s.header.p_vaddr, s.data()) for s in elf.iter_segments()
                if s.header.p_type == 'PT_LOAD']
    symbols = {s.name: s.entry.st_value
               for s in elf.get_section_by_name('.symtab').iter_symbols()}

cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
cpu.mem_map(0x08000000, 0x100000)
cpu.mem_map(0x20000000, 0x20000)
cpu.mem_map(0x40000000, 0x30000)
cpu.mem_map(0x1000, 0x1000)
for address, data in segments:
    cpu.mem_write(address, data)
tasks, irqs = {}, {}

def word(address):
    return struct.unpack('<I', cpu.mem_read(address, 4))[0]

def cstring(address):
    data = bytearray()
    while True:
        value = cpu.mem_read(address + len(data), 1)
        if value == b'\0':
            return data.decode('ascii')
        data.extend(value)

names = ('xTaskCreate', 'xQueueGenericCreate', 'vPortEnterCritical',
         'vPortExitCritical', 'vTaskDelete', 'USBH_LL_Init', 'NVIC_Init')
addresses = {symbols[n] & ~1: n for n in names}

def intercept(uc, address, size, user):
    name = addresses.get(address)
    if name is None:
        return
    result = 0
    if name == 'xTaskCreate':
        task_name = cstring(uc.reg_read(UC_ARM_REG_R1))
        stack = uc.reg_read(UC_ARM_REG_SP)
        tasks[task_name] = (word(stack), uc.reg_read(UC_ARM_REG_R0), uc.reg_read(UC_ARM_REG_R2))
        handle = word(stack + 4)
        if handle:
            uc.mem_write(handle, struct.pack('<I', 0x20017000))
        result = 1
    elif name == 'xQueueGenericCreate':
        result = 0x20016000
    elif name == 'NVIC_Init':
        config = bytes(uc.mem_read(uc.reg_read(UC_ARM_REG_R0), 4))
        irqs[config[0]] = config[1]
    elif name == 'vTaskDelete':
        uc.emu_stop()
        return
    uc.reg_write(UC_ARM_REG_R0, result)
    uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))

cpu.hook_add(UC_HOOK_CODE, intercept)

def call(name, *args):
    cpu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
    cpu.reg_write(UC_ARM_REG_LR, 0x1001)
    for register, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2), args):
        cpu.reg_write(register, value)
    cpu.emu_start(symbols[name] | 1, 0x1000, count=300000)

cpu.mem_write(symbols['Check'], b'\0')
cpu.mem_write(symbols['SysVal'], b'\x02')  # V1_1: USB hand controller and ICM20948.
call('start_task', 0)
expected = {'Balance_task': 12, 'vision_vofa': 11, 'PSTWO_task': 12,
            'route_demo': 8, 'distance_task': 7, 'stepper_task': 6,
            'servo_task': 5, 'DATA_task': 3, 'show_task': 2, 'led_task': 1}
for name, priority in expected.items():
    assert tasks[name][0] == priority, (name, tasks[name], priority)
assert tasks['vision_vofa'][1] == symbols['RouteVofa_Task']
assert tasks['vision_vofa'][2] >= 256
assert tasks['IMU_task'][0] == tasks['Balance_task'][0]
assert tasks['IMU_task'][1] == symbols['ICM20948_task']

# The actual USB middleware passes this value to native xTaskCreate.
call('USBH_Init', 0x20018000, 0, 0)
assert tasks['USBH_Queue'][0] == tasks['PSTWO_task'][0] == tasks['Balance_task'][0]
call('uart2_init', 9600)
call('uart3_init', 115200)
assert irqs[38] == 6 and irqs[39] == 0, irqs
print('PASS: compiled input/chassis equal priorities, IMU group, USB native priority and visual IRQ priority')
