"""Compile the production task tests with ARMCC and execute with Unicorn.

From the repository root:
  uv run --with unicorn --with pyelftools python tests/mango/run_tests.py
Hardware drivers, FreeRTOS scheduling and sensor readings are mocked.
"""
from pathlib import Path
import os
import struct
import subprocess

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_INTR
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_PC, UC_ARM_REG_SP

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "tests/mango/.build"
BIN = Path(os.environ.get("ARMCC_BIN", "D:/Keil_v5/ARM/ARMCC/bin"))
BUILD.mkdir(exist_ok=True)
subprocess.run([
    str(BIN / "armcc.exe"), "--cpu", "Cortex-M4", "--c99", "-O0", "-g",
    "-Itests/mango/stubs", "-c", "tests/mango/test_mango.c",
    "-o", str(BUILD / "test_mango.o"),
], cwd=ROOT, check=True)
subprocess.run([
    str(BIN / "armlink.exe"), "--cpu", "Cortex-M4", "--ro_base", "0x0",
    "--rw_base", "0x20000000", "--entry", "__main",
    str(BUILD / "test_mango.o"), "-o", str(BUILD / "test_mango.axf"),
], cwd=ROOT, check=True)

cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
cpu.mem_map(0x0, 0x400000)
cpu.mem_map(0x20000000, 0x200000)
with (BUILD / "test_mango.axf").open("rb") as stream:
    elf = ELFFile(stream)
    entry = elf.header.e_entry
    for segment in elf.iter_segments():
        if segment.header.p_type == "PT_LOAD":
            cpu.mem_write(segment.header.p_vaddr, segment.data())

def words(address, count):
    return struct.unpack("<" + "I" * count, cpu.mem_read(address, 4 * count))

def cstring(address):
    data = bytearray()
    while True:
        value = cpu.mem_read(address + len(data), 1)
        if value == b"\0":
            return bytes(data)
        data.extend(value)

output = []
exited = False

def semihosting(uc, number, _):
    global exited
    pc = uc.reg_read(UC_ARM_REG_PC)
    if bytes(uc.mem_read(pc, 2)) == b"\xab\xbe":
        uc.reg_write(UC_ARM_REG_PC, (pc + 2) | 1)
    elif bytes(uc.mem_read(pc - 2, 2)) != b"\xab\xbe":
        raise RuntimeError(f"Unexpected exception {number} at {pc:#x}")
    operation = uc.reg_read(UC_ARM_REG_R0)
    argument = uc.reg_read(UC_ARM_REG_R1)
    result = 0
    if operation == 1:  # SYS_OPEN (:tt)
        result = 1
    elif operation == 3:  # SYS_WRITEC
        output.append(bytes(uc.mem_read(argument, 1)))
    elif operation == 4:  # SYS_WRITE0
        output.append(cstring(argument))
    elif operation == 5:  # SYS_WRITE
        _, address, length = words(argument, 3)
        output.append(bytes(uc.mem_read(address, length)))
    elif operation == 9:  # SYS_ISTTY
        result = 1
    elif operation == 0x15:  # SYS_GET_CMDLINE
        address, _ = words(argument, 2)
        uc.mem_write(address, b"\0")
        uc.mem_write(argument + 4, struct.pack("<I", 0))
    elif operation == 0x16:  # SYS_HEAPINFO
        address, = words(argument, 1)
        uc.mem_write(address, struct.pack("<IIII", 0x20080000, 0x20100000, 0x201ff000, 0x20100000))
    elif operation in (0x18, 0x20):  # SYS_EXIT / SYS_EXIT_EXTENDED
        exited = True
        uc.emu_stop()
    elif operation not in (2, 0x13):  # SYS_CLOSE / SYS_ERRNO
        raise RuntimeError(f"Unsupported semihosting call {operation:#x}")
    uc.reg_write(UC_ARM_REG_R0, result)

cpu.hook_add(UC_HOOK_INTR, semihosting)
cpu.reg_write(UC_ARM_REG_SP, 0x201ff000)
cpu.emu_start(entry | 1, 0xffffffff, timeout=30_000_000, count=20_000_000)
message = b"".join(output).decode("utf-8", errors="replace")
print(message, end="")
if not exited or "PASS:" not in message or "FAIL" in message:
    raise SystemExit("State machine tests failed or did not finish")
