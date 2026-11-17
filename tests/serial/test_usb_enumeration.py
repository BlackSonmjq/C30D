"""Run the linked USB enumeration state machine without a USB device.

A busy receiver must not reset the robot after 500 polling calls. Hardware
requests are mocked; the actual enum branches and retry counters are executed.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests/camera/.build/python_deps'))
from elftools.elf.elffile import ELFFile
from elftools.dwarf.dwarf_expr import DWARFExprParser
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_SP

firmware = Path(sys.argv[1]) if len(sys.argv) > 1 else (
    ROOT.parent / 'C30D-develop-EIDE/build/FreeRTOS/WHEELTEC.axf')
with firmware.open('rb') as stream:
    elf = ELFFile(stream)
    segments = [(s.header.p_vaddr, s.data()) for s in elf.iter_segments()
                if s.header.p_type == 'PT_LOAD']
    symbols = {s.name: s.entry.st_value
               for s in elf.get_section_by_name('.symtab').iter_symbols()}
    dwarf = elf.get_dwarf_info()
    enum, host = {}, None
    for unit in dwarf.iter_CUs():
        for die in unit.iter_DIEs():
            name = die.attributes.get('DW_AT_name')
            if not name:
                continue
            name = name.value.decode(errors='replace')
            if die.tag == 'DW_TAG_enumerator':
                enum[name] = die.attributes['DW_AT_const_value'].value
            if name == 'USBH_HandleTypeDef' and host is None:
                host = die
    while host.tag == 'DW_TAG_typedef':
        host = host.get_DIE_from_attribute('DW_AT_type')
    host_size = host.attributes['DW_AT_byte_size'].value
    offsets = {}
    for member in host.iter_children():
        if member.tag != 'DW_TAG_member':
            continue
        value = member.attributes['DW_AT_data_member_location'].value
        if isinstance(value, list):
            value = DWARFExprParser(dwarf.structs).parse_expr(value)[0].args[0]
        offsets[member.attributes['DW_AT_name'].value.decode()] = value

HOST = 0x20018000
cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
for address, size in ((0x08000000, 0x100000), (0x20000000, 0x20000),
                      (0x1000, 0x1000), (0xE000E000, 0x4000)):
    cpu.mem_map(address, size)
for address, data in segments:
    cpu.mem_write(address, data)
status, requests = enum['USBH_BUSY'], 0
addresses = {symbols[n] & ~1: n for n in ('USBH_Get_DevDesc', 'USBH_OpenPipe', 'USBH_FreePipe')}

def intercept(uc, address, size, user):
    global requests
    name = addresses.get(address)
    if name is None:
        return
    result = enum['USBH_OK']
    if name == 'USBH_Get_DevDesc':
        requests += 1
        result = status
    uc.reg_write(UC_ARM_REG_R0, result)
    uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))

def no_reset(uc, access, address, size, value, user):
    if value & 4:
        raise AssertionError('USB enumeration requested an MCU reset while the receiver was busy')

cpu.hook_add(UC_HOOK_CODE, intercept)
cpu.hook_add(UC_HOOK_MEM_WRITE, no_reset, begin=0xE000ED0C, end=0xE000ED0F)

def initialize():
    cpu.mem_write(HOST, bytes(host_size))
    cpu.mem_write(HOST + offsets['gState'], bytes([enum['HOST_ENUMERATION']]))

def call():
    cpu.reg_write(UC_ARM_REG_R0, HOST)
    cpu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
    cpu.reg_write(UC_ARM_REG_LR, 0x1001)
    cpu.emu_start(symbols['USBH_HandleEnum'] | 1, 0x1000, count=10000)

initialize()
for _ in range(600):
    call()
assert requests == 600
assert cpu.mem_read(HOST + offsets['gState'], 1)[0] == enum['HOST_ENUMERATION']
status = enum['USBH_OK']
call()
assert cpu.mem_read(HOST + offsets['EnumState'], 1)[0] == enum['ENUM_GET_FULL_DEV_DESC']

initialize()
status = enum['USBH_NOT_SUPPORTED']
for _ in range(4):
    call()
assert cpu.mem_read(HOST + offsets['gState'], 1)[0] == enum['HOST_ABORT_STATE']
print('PASS: USB enum survives 600 busy polls, recovers on descriptor and bounds failures without MCU reset')
