"""Exercise the compiled USART3 ISR with emulated UART registers, no hardware.

Build the EIDE firmware first, then run with unicorn and pyelftools installed:
  python tests/serial/test_task_commands.py [path/to/WHEELTEC.axf]
"""
from pathlib import Path
import struct
import sys
import re

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_SP, UC_ARM_REG_PC, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(sys.argv[1]) if len(sys.argv) > 1 else (
    ROOT.parent.parent / "C30D_EIDE/build/FreeRTOS/WHEELTEC.axf"
)
UART3 = 0x40004800
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
        cpu.mem_write(UART3, struct.pack("<II", status, byte))
        cpu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
        cpu.reg_write(UC_ARM_REG_LR, 0x1001)
        cpu.emu_start(symbols["USART3_IRQHandler"] | 1, 0x1000, count=10000)


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
    check("usart3_forward_write" not in symbols)  # The legacy transparent bridge is absent.

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

cpu = fresh()
receive(cpu,"V2")
check(read(cpu,"servo_debug_pending")==14)
check(read(cpu,"servo_debug_busy")==1)
receive(cpu,"K1S1B0")
check(read(cpu,"servo_debug_pending")==14)
check(read(cpu,"job_busy")==0)

cpu = fresh()
receive(cpu,"V0")
check(read(cpu,"servo_debug_pending")==12)

cpu = fresh()
receive(cpu, "S")
check(read(cpu, "camera_report_pending") == 0)
receive(cpu, "J\r\n")
check(read(cpu, "camera_report_pending") == 1)
check(read(cpu, "servo_debug_pending") == 0 and read(cpu, "job_busy") == 0)

cpu = fresh()
receive(cpu, "K3SJ")
check(read(cpu, "camera_report_pending") == 1 and read(cpu, "pending_mango") == 3)
check(read(cpu, "pending_laser") == 0 and read(cpu, "servo_debug_pending") == 0)

cpu = fresh()
receive(cpu, "S")
receive(cpu, "J", status=0x28)
receive(cpu, "J")
check(read(cpu, "camera_report_pending") == 0)
receive(cpu, "SJ")
check(read(cpu, "camera_report_pending") == 1)

cpu = fresh()
receive(cpu, "JG")
check(read(cpu, "camera_report_pending") == 0)
receive(cpu, "SSJ")
check(read(cpu, "camera_report_pending") == 0)  # Existing malformed-S recovery.
receive(cpu, "SJ")
check(read(cpu, "camera_report_pending") == 1)


def invoke(cpu, name):
    cpu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
    cpu.reg_write(UC_ARM_REG_LR, 0x1001)
    cpu.emu_start(symbols[name] | 1, 0x1000, count=1000000)
    check(cpu.reg_read(UC_ARM_REG_PC) == 0x1000)


def camera_bytes(cpu, data):
    for byte in data:
        cpu.mem_write(0x40011000, struct.pack("<II", 0x20, byte))
        invoke(cpu, "USART1_IRQHandler")


cpu = fresh()
reports = []


def capture_report(uc, address, size, context):
    if address != symbols["any_printf"] & ~1:
        return
    ptr = uc.reg_read(UC_ARM_REG_R1)
    text = bytearray()
    while bytes(uc.mem_read(ptr + len(text), 1)) != b"\0":
        text.extend(uc.mem_read(ptr + len(text), 1))
    fmt = text.decode("ascii")
    specifiers = re.findall(r"%(?:l)?[ud]", fmt)
    values = [uc.reg_read(UC_ARM_REG_R2), uc.reg_read(UC_ARM_REG_R3)]
    if len(specifiers) > 2:
        values.extend(struct.unpack("<" + "I" * (len(specifiers) - 2),
                                    uc.mem_read(uc.reg_read(UC_ARM_REG_SP), 4 * (len(specifiers) - 2))))
    values = values[:len(specifiers)]
    for i, spec in enumerate(specifiers):
        if spec.endswith("d") and values[i] >= 0x80000000:
            values[i] -= 0x100000000
    reports.append(fmt.replace("%lu", "%u") % tuple(values))
    uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))


cpu.hook_add(UC_HOOK_CODE, capture_report)
receive(cpu, "SJ")
invoke(cpu, "CameraReport_Process")
check(reports == ["CMD SJ: NO DATA R=0 E=0 U=0\r\n"])
check(read(cpu, "camera_report_pending") == 0)
invoke(cpu, "CameraReport_Process")
check(len(reports) == 1)  # One snapshot per request, no unsolicited stream.

for offset in (-20, -128, 127):
    camera_bytes(cpu, bytes([0x63, 1, offset & 0xFF, 1, 3, 0x66]))
    receive(cpu, "SJ")
    invoke(cpu, "CameraReport_Process")
    check(f"CMD SJ: B=1 DX={offset} G=1 REGION=3 F=" in reports[-1])
    check(read(cpu, "job_busy") == 0 and read(cpu, "servo_debug_pending") == 0)

camera_bytes(cpu, bytes([0x63, 0, 1, 1, 2, 0x66]))  # Invalid candidate preserves verified data.
receive(cpu, "SJ")
invoke(cpu, "CameraReport_Process")
check("B=1 DX=127 G=1 REGION=3 F=3 R=24 E=1 U=0" in reports[-1])

# Camera payload bytes can match command characters, but use a separate IRQ.
cpu = fresh()
camera_bytes(cpu, bytes([0x63, 1, ord("1"), 1, 3, 0x66]))
check(read(cpu, "pending_laser") == 0 and read(cpu, "job_busy") == 0)


# USART1 accepts the same commands between frames, with independent prefixes.
cpu = fresh()
camera_bytes(cpu, b"S1")
check(read(cpu, "servo_debug_pending") == 1)
check(read(cpu, "pending_laser") == 0)
cpu = fresh()
camera_bytes(cpu, b"1")
check(read(cpu, "pending_laser") == 1)
cpu = fresh()
camera_bytes(cpu, b"S")
receive(cpu, "2")
check(read(cpu, "pending_laser") == 2 and read(cpu, "servo_debug_pending") == 0)
camera_bytes(cpu, b"J")
check(read(cpu, "camera_report_pending") == 1)
cpu = fresh()
camera_bytes(cpu, b"S" + bytes([0x63, 1, ord("1"), 1, 3, 0x66]) + b"2")
check(read(cpu, "pending_laser") == 2 and read(cpu, "servo_debug_pending") == 0)
cpu = fresh()
camera_bytes(cpu, b"cS1K4?")
check(read(cpu, "pending_laser") == 0 and read(cpu, "job_busy") == 0)
check(read(cpu, "servo_debug_pending") == 0)

# Also execute the real formatter and UART sender, capturing peripheral writes.
cpu = fresh()
wire = bytearray()
cpu.mem_write(UART3 + 0x0C, struct.pack("<I", 0x2008))  # UART enabled, TX enabled.


def capture_wire(uc, access, address, size, value, context):
    if address == UART3 + 4:
        wire.append(value & 0xFF)


cpu.hook_add(UC_HOOK_MEM_WRITE, capture_wire)
for frame in (None, bytes([0x63, 1, 0xEC, 1, 3, 0x66])):
    if frame:
        camera_bytes(cpu, frame)
    receive(cpu, "SJ")
    cpu.mem_write(UART3, struct.pack("<I", 0xC0))  # TXE and TC remain ready.
    invoke(cpu, "CameraReport_Process")
check(wire == b"CMD SJ: NO DATA R=0 E=0 U=0\r\n"
              b"CMD SJ: B=1 DX=-20 G=1 REGION=3 F=1 R=6 E=0 U=0\r\n")

# The actual PA9 TX path acknowledges accepted and busy servo commands.
for number in range(1, 5):
    cpu = fresh()
    wire = bytearray()
    uart = 0x40011000

    def capture_ack(uc, access, address, size, value, context):
        if address == uart + 4:
            wire.append(value & 0xFF)

    cpu.hook_add(UC_HOOK_MEM_WRITE, capture_ack)
    command = f"S{number}".encode()
    camera_bytes(cpu, command + command)
    check(read(cpu, "servo_debug_pending") == number)
    cpu.mem_write(uart + 0x0C, struct.pack("<I", 0x2008))
    cpu.mem_write(uart, struct.pack("<I", 0xC0))
    invoke(cpu, "USART3_CommandLogProcess")
    check(wire == f"CMD S{number} ACCEPTED\r\nCMD S{number} BUSY\r\n".encode())

    # The task consumes the request and reports the UART4 query on the same port.
    wire.clear()
    invoke(cpu, "ServoDebug_Process")
    check(read(cpu, "servo_debug_pending") == 0)
    check(f"SERVO {number} mode query baud=".encode() in wire)

print(f"PASS: compiled USART3 K1-K4/SJ commands, vision snapshots, real UART bytes, busy/error recovery ({checks} checks)")
