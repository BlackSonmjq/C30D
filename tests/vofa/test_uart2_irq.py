"""Exercise the compiled VOFA USART2 ISR with an emulated STM32 receive register.

No board is connected. The DR read hook models the hardware's SR/DR flag clear.
Build the EIDE image first, then run this file with unicorn and pyelftools.
"""
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests/camera/.build/python_deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_READ
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_SP

firmware = Path(sys.argv[1]) if len(sys.argv) > 1 else (
    ROOT.parent / 'C30D-develop-EIDE/build/FreeRTOS/WHEELTEC.axf')
with firmware.open('rb') as stream:
    elf = ELFFile(stream)
    segments = [(s.header.p_vaddr, s.data()) for s in elf.iter_segments()
                if s.header.p_type == 'PT_LOAD']
    symbols = {s.name: s.entry.st_value
               for s in elf.get_section_by_name('.symtab').iter_symbols()}

UART2 = 0x40004400
RX_FLAGS = 0x20 | 0x08 | 0x04 | 0x02 | 0x01
checks = 0
for flags in (0x20, 0x28, 0x08, 0x02, 0x04, 0x01, RX_FLAGS):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.mem_map(0x08000000, 0x100000)
    cpu.mem_map(0x20000000, 0x20000)
    cpu.mem_map(0x40000000, 0x30000)
    cpu.mem_map(0x1000, 0x1000)
    for address, data in segments:
        cpu.mem_write(address, data)
    # UE, TE, RE, RXNEIE enabled, TXEIE off: the receive interrupt is isolated.
    cpu.mem_write(UART2, struct.pack('<III', flags, ord('A'), 0))
    cpu.mem_write(UART2 + 12, struct.pack('<I', 0x202C))
    cpu.mem_write(symbols['APP_ON_Flag'], b'\0')
    reads = []

    def on_read(uc, access, address, size, value, user):
        if address == UART2:
            reads.append('SR')
        elif address == UART2 + 4:
            assert reads and reads[-1] == 'SR', 'DR must follow an SR read'
            reads.append('DR')
            status, = struct.unpack('<I', uc.mem_read(UART2, 4))
            uc.mem_write(UART2, struct.pack('<I', status & ~RX_FLAGS))

    cpu.hook_add(UC_HOOK_MEM_READ, on_read, begin=UART2, end=UART2 + 4)

    def invoke():
        cpu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
        cpu.reg_write(UC_ARM_REG_LR, 0x1001)
        cpu.emu_start(symbols['USART2_IRQHandler'] | 1, 0x1000, count=10000)

    invoke()
    assert reads.count('DR') == 1, f'flags {flags:#x}: receive data was not drained'
    assert struct.unpack('<I', cpu.mem_read(UART2, 4))[0] & RX_FLAGS == 0
    assert cpu.mem_read(symbols['APP_ON_Flag'], 1) == b'\0', 'APP parser must remain disabled'
    invoke()
    assert reads.count('DR') == 1, 'no repeated drain when no receive flag is pending'
    checks += 1
print(f'PASS: compiled USART2 ISR drains RX/errors without APP parsing ({checks} cases)')
