from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26"
HERE = Path(__file__).resolve().parent
compiler = os.environ.get("ROUTE_HOST_CC") or shutil.which("gcc")
if not compiler:
    raise SystemExit("Set ROUTE_HOST_CC to a C11 GCC compiler.")
with tempfile.TemporaryDirectory(prefix="mango-route-tests-") as temporary:
    build = Path(temporary).resolve()
    assert build.parent == Path(tempfile.gettempdir()).resolve()
    for name in ("route_config.h", "route_io.h", "route_io.c", "route_plan.h", "route_plan.c",
                 "route_nav.h", "route_nav.c", "my_chassis_task.h", "my_chassis_task.c",
                 "chassis_odometry.h", "chassis_odometry.c", "route_debug.h",
                 "route_debug.c", "route_vofa.c"):
        shutil.copyfile(FW / "USER" / name, build / name)
    for name in ("heading_control.h", "heading_control.c"):
        shutil.copyfile(FW / "BALANCE" / name, build / name)
    config = (build / "route_config.h").read_text(encoding="utf-8")

    def extract_function(source, signature):
        start = source.index(signature)
        opening = source.index("{", start)
        depth, end = 1, opening + 1
        while depth:
            depth += (source[end] == "{") - (source[end] == "}")
            end += 1
        return source[start:end]

    serial_source = (FW / "SYSTEM/usart/usart.c").read_bytes().decode("utf-8",errors="replace")
    uart_source = (FW / "HARDWARE/usartx.c").read_bytes().decode("utf-8",errors="replace")
    legacy = '#include <stdarg.h>\nstatic uint8_t Ux_TxBuff[256];\n'
    legacy += '#define USART1_SERVO_LOG_ONLY 1\n#define USART_TX_POLL_LIMIT 1000U\nvolatile unsigned usart_tx_timeout_count,usart_tx_disabled_count;\n'
    legacy += extract_function(serial_source,"static int USART_WaitFlag(")+"\n"
    legacy += extract_function(serial_source,"int USART_WriteByte(")+"\n"
    for source, signature, old, new in (
        (serial_source, "int fputc(", "fputc", "route_test_fputc"),
        (serial_source, "void any_printf(", "any_printf", "route_test_any_printf"),
        (uart_source, "void usart2_send(", "usart2_send", "route_test_usart2_send"),
        (uart_source, "int USART2_IRQHandler(", "USART2_IRQHandler", "route_test_usart2_irq"),
    ):
        legacy += extract_function(source, signature).replace(old + "(", new + "(", 1) + "\n"
    system_source = (FW / "BALANCE/system.c").read_bytes().decode("utf-8",errors="replace")
    init_calls = re.findall(r"uart2_init\([^;]*\);", system_source)
    assert len(init_calls) == 1
    legacy += "static unsigned initialized_uart2_baud;\n"
    legacy += "static void uart2_init(unsigned baud){initialized_uart2_baud=baud;}\n"
    legacy += "static void route_test_init_uart2(void){" + init_calls[0] + "}\n"
    (build / "legacy_usart2.c").write_text(legacy, encoding="utf-8")

    for debug, calibrated, vofa, production in ((1, 1, 1, True), (1, 1, 1, False),
                                              (0, 1, 1, False), (1, 1, 0, False),
                                              (1, 0, 1, False)):
        settings = {
            "ROUTE_DEBUG_MODE": str(debug),
            "ROUTE_CALIBRATION_READY": str(calibrated),
            "ROUTE_VOFA_ENABLE": str(vofa),
        }
        if not production:
            settings.update({"ROUTE_ENTRY_LENGTH_MM": "700.0f",
                             "ROUTE_EXIT_LENGTH_MM": "700.0f",
                             "Front_Lidar_Center": "80U", "Behind_Lidar_Center": "75U"})
        active = config
        for name, value in settings.items():
            active, count = re.subn(r"(?m)^(#define\s+" + re.escape(name) + r"\s+)\S+",
                                   lambda m: m.group(1) + value, active)
            assert count == 1, name
        (build / "route_config.h").write_text(active, encoding="utf-8")
        executable = build / ("route-test.exe" if os.name == "nt" else "route-test")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I", str(build), "-I", str(HERE / "stubs"),
                        str(HERE / "test_route.c"), str(build / "route_plan.c"),
                        str(build / "chassis_odometry.c"), str(build / "heading_control.c"),
                        str(build / "route_debug.c"), str(build / "route_vofa.c"),
                        "-lm", "-o", str(executable)], check=True)
        print(f"DEBUG={debug} CALIBRATED={calibrated} VOFA={vofa} PRODUCTION_CALIBRATION={production}", flush=True)
        subprocess.run([str(executable)], check=True, timeout=30)

    # 检查真正的底盘换算与调用，防止只修复里程函数却把调用单位仍写成速度。
    balance_source = (FW / "BALANCE/balance.c").read_bytes().decode("utf-8",errors="replace")

    def function(signature):
        start = balance_source.index(signature)
        opening = balance_source.index("{", start)
        depth, end = 1, opening + 1
        while depth:
            depth += (balance_source[end] == "{") - (balance_source[end] == "}")
            end += 1
        return balance_source[start:end]

    start = balance_source.index("ChassisOdom_UpdateDelta(")
    ending = balance_source.index(";", start) + 1
    bridge = (HERE / "test_encoder_bridge.c").read_text(encoding="utf-8")
    bridge += "\n" + function("static float wheelCoefficient(")
    bridge += "\n" + function("void Get_Velocity_Form_Encoder(void)")
    bridge += "\nstatic void production_odom_call(void){\n" + balance_source[start:ending] + "\n}\n"
    (build / "encoder_bridge.c").write_text(bridge, encoding="utf-8")
    subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(build), "-I", str(HERE / "stubs"),
                    str(build / "encoder_bridge.c"), str(build / "chassis_odometry.c"),
                    "-lm", "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True, timeout=30)
