from pathlib import Path
import os
import re
import shutil
import subprocess
import sys
from host_test_calibration import build_validator

ROOT = Path(__file__).resolve().parents[1]
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
out = ROOT / "OBJ/tests"
out.mkdir(parents=True, exist_ok=True)
(ROOT / "tools/testdata").mkdir(exist_ok=True)
cc = os.environ.get("HOST_CC") or shutil.which("gcc")
if not cc:
    raise SystemExit("GCC required for firmware wire-format tests")
# Check the actual board integration, beyond the portable encoder and queue.
def board_source(name):
    return (ROOT/name).read_bytes().decode("gbk",errors="replace")
usart=board_source("HARDWARE/usartx.c")
uart2=usart[usart.index("void uart2_init("):usart.index("void uart3_init(")]
assert re.search(r"NVIC_IRQChannelPreemptionPriority\s*=\s*6\s*;",uart2)
irq=usart[usart.index("int USART2_IRQHandler("):usart.index("int USART3_IRQHandler(")]
assert "NavBle_TxIRQ();" in irq and "Get_RC" not in irq and "Move_X=" not in irq
task=usart[usart.index("void data_task("):usart.index("void data_transition(")]
assert "NavBle_Poll();" in task and "APP_Send" not in task
assert "uart2_init(NAV_BLE_BAUD);" in board_source("BALANCE/system.c")
assert "if(USARTx==USART2) return;" in board_source("SYSTEM/usart/usart.c")
print("PASS board UART2 telemetry ownership, interrupt priority and disabled receive commands")
for name, sources in (
    ("vofa_packet", ["USER/nav_vofa.c", "tests/vofa_packet_test.c"]),
    ("vofa_observation", ["USER/nav_core.c", "tests/fixtures/nav_calibration_nominal.c", "tests/vofa_observation_test.c"]),
):
    exe=out/(name+".exe")
    cmd=[cc,"-std=c99","-Wall","-Wextra","-Werror","-O2","-I",str(ROOT/"USER"),"-I",str(ROOT/"BALANCE")]
    cmd += [str(ROOT/p) for p in sources] + ["-lm","-o",str(exe)]
    if name=="vofa_observation":
        cmd.insert(-3,str(build_validator(ROOT,cc,out,"vofa-calibration-validator")))
    subprocess.run(cmd,check=True,cwd=ROOT)
    args=[exe]
    if name=="vofa_packet":
        args.append("tools/testdata/nav48-frame.bin")
    subprocess.run(args,check=True,cwd=ROOT)
environment=os.environ.copy()
environment["PYTHONIOENCODING"]="utf-8"
result=subprocess.run([sys.executable,"-m","unittest","discover","-s","tests","-p","test_ble_vofa.py","-v"],
                      cwd=ROOT,env=environment,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
log=result.stdout.decode("utf-8",errors="replace")
(out/"ble-tests.log").write_text(log,encoding="utf-8")
print(log)
raise SystemExit(result.returncode)
