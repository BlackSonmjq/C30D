"""Exercise the compiled VOFA sender, busy-frame integrity and 400 ms cadence."""
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests/camera/.build/python_deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_LR, UC_ARM_REG_SP, UC_ARM_REG_PC, UC_ARM_REG_PRIMASK

image = ROOT.parent / 'C30D-develop-EIDE/build/FreeRTOS/WHEELTEC.axf'
cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
for address, size in ((0x08000000, 0x100000), (0x20000000, 0x20000),
                      (0x40000000, 0x30000), (0x1000, 0x1000)):
    cpu.mem_map(address, size)
with image.open('rb') as stream:
    elf = ELFFile(stream)
    for segment in elf.iter_segments():
        if segment.header.p_type == 'PT_LOAD':
            cpu.mem_write(segment.header.p_vaddr, segment.data())
    symbols = {s.name: s.entry.st_value for s in elf.get_section_by_name('.symtab').iter_symbols()}

UART2 = 0x40004400
tick = 0
packets = []
sent = bytearray()
cpu.mem_write(UART2, struct.pack('<I', 0x80))  # TXE remains ready between calls.
cpu.mem_write(UART2 + 12, struct.pack('<I', 0x200C))


def interrupt_enabled():
    return bool(struct.unpack('<I', cpu.mem_read(UART2 + 12, 4))[0] & 0x80)


def on_code(uc, address, size, _):
    if address == symbols['xTaskGetTickCount'] & ~1:
        uc.reg_write(UC_ARM_REG_R0, tick)
    elif address == symbols['RouteDebug_BuildPacket'] & ~1:
        assert not interrupt_enabled(), 'building a frame while TX is active'
        packet = struct.pack('<60f', *([float(len(packets) + 1)] * 60)) + bytes.fromhex('0000807f')
        assert uc.reg_read(UC_ARM_REG_R1) == len(packet)
        uc.mem_write(uc.reg_read(UC_ARM_REG_R0), packet)
        packets.append(packet)
        uc.reg_write(UC_ARM_REG_R0, len(packet))
    else:
        return
    uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))


def on_write(uc, access, address, size, value, _):
    if address == UART2 + 4:
        sent.append(value & 0xFF)


cpu.hook_add(UC_HOOK_CODE, on_code)
cpu.hook_add(UC_HOOK_MEM_WRITE, on_write, begin=UART2 + 4, end=UART2 + 4)


def invoke(name):
    cpu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
    cpu.reg_write(UC_ARM_REG_LR, 0x1001)
    cpu.emu_start(symbols[name] | 1, 0x1000, count=10000)
    assert cpu.reg_read(UC_ARM_REG_PC) == 0x1000


invoke('RouteVofa_Poll')
assert interrupt_enabled() and len(packets) == 1
for _ in range(20):
    invoke('RouteVofa_Uart2TxIRQ')
tick = 1000  # Even an overdue poll must preserve a frame still in flight.
invoke('RouteVofa_Poll')
assert len(packets) == 1
for _ in range(225):
    invoke('RouteVofa_Uart2TxIRQ')
assert not interrupt_enabled() and bytes(sent) == packets[0]
cpu.reg_write(UC_ARM_REG_PRIMASK, 1)
invoke('RouteVofa_Poll')
assert cpu.reg_read(UC_ARM_REG_PRIMASK) == 1
assert interrupt_enabled() and len(packets) == 2
cpu.reg_write(UC_ARM_REG_PRIMASK, 0)
for _ in range(245):
    invoke('RouteVofa_Uart2TxIRQ')
assert not interrupt_enabled() and bytes(sent) == b''.join(packets)
tick = 1399
invoke('RouteVofa_Poll')
assert not interrupt_enabled() and len(packets) == 2
tick = 1400
invoke('RouteVofa_Poll')
assert interrupt_enabled() and len(packets) == 3
for _ in range(245):
    invoke('RouteVofa_Uart2TxIRQ')
assert not interrupt_enabled() and bytes(sent) == b''.join(packets)
print('PASS: compiled 60-channel TX preserves busy frames, restores IRQ mask and waits 400 ms')
