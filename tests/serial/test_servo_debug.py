"""Run the compiled manual servo state machine with simulated servo replies.

Requires a freshly built EIDE WHEELTEC.axf, Unicorn and pyelftools.
The firmware executes its real parsing, phase transitions and timeout logic.
Only UART transport, tick count and diagnostic printing are intercepted.
"""
from pathlib import Path
import runpy
import struct

from unicorn import UC_HOOK_CODE
from unicorn.arm_const import (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2,
                               UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_SP)

ROOT = Path(__file__).resolve().parents[2]
support = runpy.run_path(str(ROOT / "tests/serial/test_task_commands.py"))
symbols = support["symbols"]


def simulate(number, mode, fail_move=False, fail_read=False, position_offset=0,
             stuck_outward=0, stuck_position=541):
    cpu = support["fresh"]()
    tick, pending, position = 0, None, 500
    moves, stops = [], []
    move_ticks = []
    outward_sends = 0
    mode_queries = 0
    names = ["xTaskGetTickCount", "Servo_ClearRx", "Servo_SendCommand",
             "Servo_MovePulse", "Servo_ReadPosition", "Servo_GetFrame",
             "Servo_Stop", "any_printf"]
    addresses = {symbols[name] & ~1: name for name in names}

    def intercept(uc, address, size, user):
        nonlocal pending, position, outward_sends, mode_queries
        name = addresses.get(address)
        if name is None:
            return
        a = uc.reg_read(UC_ARM_REG_R0)
        b = uc.reg_read(UC_ARM_REG_R1)
        c = uc.reg_read(UC_ARM_REG_R2)
        result = 0
        if name == "xTaskGetTickCount":
            result = tick
        elif name == "Servo_ClearRx":
            pending = None
        elif name == "Servo_SendCommand":
            command = bytes(uc.mem_read(a, 11)).split(b"\0")[0]
            assert command == f"#{number:03d}PMOD!".encode()
            mode_queries += 1
            pending = None if mode is None else f"#{number:03d}PMOD{mode}!".encode()
        elif name == "Servo_MovePulse":
            assert a == number and c == 500
            moves.append(b)
            move_ticks.append(tick)
            if b in (1833, 2500):
                outward_sends += 1
            if fail_move and b == 1833:
                result = 4  # SERVO_TX_TIMEOUT
            else:
                position = b
        elif name == "Servo_ReadPosition":
            assert a == number
            if fail_read:
                result = 4
            else:
                feedback = position + position_offset
                if position in (1833, 2500) and outward_sends <= stuck_outward:
                    feedback = stuck_position
                pending = f"#{number:03d}P{feedback:04d}!".encode()
        elif name == "Servo_GetFrame":
            if pending is None:
                result = 5  # SERVO_NO_FRAME
            else:
                assert len(pending) + 1 <= b
                uc.mem_write(a, pending + b"\0")
                uc.mem_write(c, struct.pack("<H", len(pending)))
                pending = None
        elif name == "Servo_Stop":
            stops.append(a)
        uc.reg_write(UC_ARM_REG_R0, result)
        uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))

    cpu.hook_add(UC_HOOK_CODE, intercept)
    support["receive"](cpu, f"S{number}")
    assert support["read"](cpu, "servo_debug_busy")
    for _ in range(300):
        cpu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
        cpu.reg_write(UC_ARM_REG_LR, 0x1001)
        cpu.emu_start(symbols["ServoDebug_Process"] | 1, 0x1000, count=100000)
        if not support["read"](cpu, "servo_debug_busy"):
            if mode is None:
                assert mode_queries == 3 and not moves
            if stuck_outward:
                outward_ticks = [t for p, t in zip(moves, move_ticks) if p in (1833, 2500)]
                assert len(outward_ticks) <= 1
                assert all(b-a >= 500 for a, b in zip(outward_ticks, outward_ticks[1:]))
                if outward_sends <= stuck_outward:
                    assert 3000 <= tick - outward_ticks[0] <= 3100
            return moves, stops
        tick += 50
    raise AssertionError("manual servo sequence did not terminate")


for number, mode in [(1, 2), (2, 2), (3, 1), (4, 1)]:
    assert simulate(number, mode) == ([500, 1833, 500], [])
assert simulate(2, 3) == ([500, 2500, 500], [])
for number in (1,2,3,4):
    assert simulate(number, None) == ([], [])
    assert simulate(number, 7) == ([], [])
    assert simulate(number, 2, fail_move=True) == ([500,1833], [number])
    assert simulate(number, 2, fail_read=True) == ([500], [number])
    assert simulate(number, 2, stuck_outward=99) == ([500,1833], [number])
assert simulate(2, 2, position_offset=100) == ([500,1833,500], [])
assert simulate(2, 2, position_offset=101) == ([500], [2])
print("PASS: S1-S4 normal feedback, unconfirmed modes cancel, failures stop without retry/return")
