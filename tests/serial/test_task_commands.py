"""Exercise the compiled USART1 ISR with emulated UART registers, no hardware.

Build the EIDE firmware first, then run with unicorn and pyelftools installed:
  python tests/serial/test_task_commands.py [path/to/WHEELTEC.axf]
"""
from pathlib import Path
import struct
import sys

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_SP

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(sys.argv[1]) if len(sys.argv) > 1 else (
    ROOT.parent.parent / "C30D_EIDE/build/FreeRTOS/WHEELTEC.axf"
)
UART1 = 0x40011000
with FIRMWARE.open("rb") as stream:
    elf = ELFFile(stream)
    segments = [(s.header.p_vaddr, s.data()) for s in elf.iter_segments()
                if s.header.p_type == "PT_LOAD"]
    symbols = {s.name: s.entry.st_value
               for s in elf.get_section_by_name(".symtab").iter_symbols()}


def fresh():
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.mem_map(0x08000000, 0x100000)
    cpu.mem_map(0x20000000, 0x20000)
    cpu.mem_map(0x40000000, 0x30000)
    cpu.mem_map(0x1000, 0x1000)  # Return sentinel, no startup/RTOS execution.
    for address, data in segments:
        cpu.mem_write(address, data)
    return cpu


def read(cpu, name, size=1):
    return int.from_bytes(cpu.mem_read(symbols[name], size), "little")


def receive(cpu, data, status=0x20):
    for byte in data.encode("ascii"):
        cpu.mem_write(UART1, struct.pack("<II", status, byte))
        cpu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
        cpu.reg_write(UC_ARM_REG_LR, 0x1001)
        cpu.emu_start(symbols["USART1_IRQHandler"] | 1, 0x1000, count=10000)


checks = 0


def check(condition):
    global checks
    checks += 1
    if not condition:
        raise AssertionError(f"check {checks} failed")


for number in range(1, 5):
    cpu = fresh()
    receive(cpu, "K")
    check(read(cpu, "job_busy") == 0)
    receive(cpu, str(number) + "\r\n")
    check(read(cpu, "job_busy") == 1)
    check(read(cpu, "pending_mango") == number)
    check(read(cpu, "pending_laser") == 0)
    check("usart3_forward_write" not in symbols)  # USART3 belongs to the camera.

cpu = fresh()
receive(cpu, "K1K2K3K4")
check(read(cpu, "pending_mango") == 1)
check(read(cpu, "rejected_command") == 1)
check(read(cpu, "pending_laser") == 0)

cpu = fresh()
receive(cpu, "K0K5K9K?\r\n")
check(read(cpu, "job_busy") == 0)
check(read(cpu, "pending_laser") == 0)
receive(cpu, "KK3")
check(read(cpu, "pending_mango") == 3)

cpu = fresh()
receive(cpu, "K")
receive(cpu, "1", status=0x28)  # RXNE plus ORE discards pending K.
receive(cpu, "2")
check(read(cpu, "pending_mango") == 0)
check(read(cpu, "pending_laser") == 2)
receive(cpu, "K4")
check(read(cpu, "pending_mango") == 4)

cpu = fresh()
receive(cpu, "1")
check(read(cpu, "job_busy") == 0)
check(read(cpu, "pending_laser") == 1)
receive(cpu, "S2")
check(read(cpu, "servo_debug_pending") == 2)

cpu = fresh()
receive(cpu, "M3")
check(read(cpu, "servo_debug_pending") == 8)
check(read(cpu, "job_busy") == 0)

cpu = fresh()
receive(cpu, "B0")
check(read(cpu,"servo_debug_pending")==10)
check(read(cpu,"servo_debug_busy")==1)
receive(cpu,"S2K1")
check(read(cpu,"servo_debug_pending")==10)
check(read(cpu,"job_busy")==0)

cpu = fresh()
receive(cpu,"K1B0")
check(read(cpu,"job_busy")==1)
check(read(cpu,"servo_debug_pending")==0)

cpu = fresh()
receive(cpu,"B?")
check(read(cpu,"servo_debug_pending")==11)
check(read(cpu,"servo_debug_busy")==1)

print(f"PASS: compiled USART1 K1-K4 commands, busy/error recovery, existing controls ({checks} checks)")
