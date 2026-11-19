from pathlib import Path
import os
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"tools"))
from host_test_calibration import build_validator
compiler = os.environ.get("HOST_CC") or shutil.which("gcc")
if not compiler:
    raise SystemExit("Set HOST_CC to a native GCC compiler.")
subprocess.run([sys.executable,str(ROOT / "tools/run_can_filter_test.py")],check=True)
subprocess.run([sys.executable,str(ROOT / "tools/run_motor_startup_tests.py")],check=True)
output = ROOT / "OBJ/tests"
output.mkdir(parents=True, exist_ok=True)
logs = []
for window in (1, 3, 5):
    validation=build_validator(ROOT,compiler,output,f"calibration-validator-{window}",window)
    exe = output / f"navigation_tests_{window}.exe"
    command = [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-O2",
               f"-DNAV_FILTER_FRAMES={window}", "-I", str(ROOT / "USER"), "-I", str(ROOT / "BALANCE")]
    command += [str(ROOT / p) for p in (
        "USER/nav_can.c", "tests/fixtures/nav_calibration_nominal.c", "USER/nav_core.c", "USER/nav_button.c",
        "BALANCE/wheel_control.c", "tests/navigation_tests.c")]
    command += [str(validation),"-lm", "-o", str(exe)]
    subprocess.run(command, cwd=ROOT, check=True)
    result = subprocess.run([exe], cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log = f"FILTER WINDOW {window}\n" + result.stdout.decode("utf-8", errors="replace").replace("\r\n", "\n")
    logs.append(log)
    print(log, end="")
    (output / "host-tests.log").write_text("\n".join(logs), encoding="utf-8")
    if result.returncode:
        raise SystemExit(result.returncode)

# Check that the tested core actually owns the firmware motor path.
def source(path):
    return (ROOT / path).read_bytes().decode("gbk", errors="replace")

balance = source("BALANCE/balance.c")
body = balance[balance.index("void Balance_task("):balance.index("void Set_Pwm(")]
assert "NavBoard_ControlStep(&command)" in body and "command.reset_pi" in body
assert "auto_pwm_clear();" not in body and "Key();" not in body
assert "Get_RC();" not in body and "Remote_Control();" not in body
system = source("BALANCE/system.c")
assert "MX_USB_HOST_Init();" not in system and "TIM8_Cap_Init(9999,168-1);" not in system
assert "FlashParam_Read();" not in system
main = source("USER/main.c")
assert "xTaskCreate(Check_task" not in main and "xTaskCreate(pstwo_task" not in main
assert main.index("taskEXIT_CRITICAL();") < main.index("if (SysVal.HardWare_Ver==V1_1) MX_USB_HOST_Init();")
usb = "HARDWARE/USB"
assert "NVIC_SystemReset" not in source(f"{usb}/STM32_USB_Host_Library/Core/Src/usbh_core.c")
assert "USBH_PROCESS_PRIO          1U" in source(f"{usb}/USB_HOST/Target/usbh_conf.h")
assert "NavBoard_Tick1ms();" in source("SYSTEM/delay/delay.c")
assert "#define DATA_TASK_PRIO  2" in source("HARDWARE/usartx.h")
integration = "PASS firmware integration ownership, disabled legacy command paths and ISR timebase\n"
print(integration, end="")
(output / "host-tests.log").write_text("\n".join(logs) + "\n" + integration, encoding="utf-8")
