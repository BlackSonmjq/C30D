"""Run the real PS2 mapping, alignment wrapper and IMU controller together."""
from pathlib import Path
import os
import subprocess
import sys
ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26'
BUILD = ROOT / 'tests/mango/.build/vision_align_integration'
BUILD.mkdir(parents=True, exist_ok=True)
source = (FW / 'BALANCE/balance.c').read_bytes().decode('gbk')
serial = (FW / 'HARDWARE/usartx.c').read_text(encoding='utf-8')
def extract(text, signature):
    start = text.index(signature)
    opening = text.index('{', start)
    while 0 <= text.find(';', start, opening) < opening:
        start = text.index(signature, start + len(signature))
        opening = text.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end] + '\n'
logic = extract(source, 'static uint8_t VisionAlign_ControlAllowed(')
logic += extract(source, 'static void ManualMecanum_Drive(')
logic += extract(source, 'static void VisionAlign_HardStop(')
logic += extract(source, 'static void VisionAlign_HoldYaw(')
logic += extract(source, 'static void VisionAlign_Drive(')
logic += extract(source, 'static void VisionAlign_ReleaseControl(')
logic += extract(source, 'uint8_t VisionAlign_GetPhase(')
logic += extract(source, 'uint8_t VisionAlign_ControlOwned(')
logic += extract(source, 'void Chassis_NavigationDrive(')
logic += extract(source, 'void VisionAlign_GetDiagnostics(')
logic += 'static void PS2_control(void) {' + extract(source[source.index('void PS2_control(void)'):], 'if (Car_Mode == Mec_Car') + '}\n'
# Execute the actual no-controller dispatch and the actual control-release guards.
reset_start = source.index('wheel_output_probe_assist_pwm = 0.0f;', source.index('void Balance_task('))
reset_end = source.index('if (vision_pick_phase == PICK_WORK || vision_pick_phase == PICK_REPORT)', reset_start)
start = source.index('if      (APP_ON_Flag)', reset_end)
end = source.index('}', source.index('chassis_vz = Move_Z;', start)) + 1
logic += 'static void production_cycle(void) { float chassis_vz; uint8_t heading_control_active=0;\n'
force_start = source.index('if (vision_pick_phase == PICK_WORK || vision_pick_phase == PICK_REPORT)', source.index('void Balance_task('))
force_end = source.index('else if (RouteIO_ControlOwned())', force_start)
logic += source[reset_start:reset_end] + source[force_start:force_end]
logic += 'else ' + extract(source, 'if (RouteIO_ControlOwned())')
logic += 'else {\n' + source[start:end] + '\n}\n}\n'
logic += extract(serial, 'void VisionStrike_Process(void)')
(BUILD / 'production_logic.inc').write_text(logic, encoding='utf-8')
for directory, name in [('USER','vision_align.h'), ('USER','vision_strike.h'),
                        ('HARDWARE','camera_protocol.h'), ('BALANCE','heading_control.c'),
                        ('BALANCE','heading_control.h')]:
    (BUILD / name).write_bytes((FW / directory / name).read_bytes())
(BUILD / 'sys.h').write_bytes((ROOT / 'tests/chassis/stubs/sys.h').read_bytes())
(BUILD / 'align_fixture.inc').write_bytes((ROOT / 'tests/chassis/test_vision_align_integration.c').read_bytes())
test_file = Path(sys.argv[1]) if len(sys.argv)>1 else ROOT / 'tests/chassis/test_vision_align_integration.c'
if test_file.name == 'test_vision_navigation_merge.c':
    route_io = (FW / 'USER/route_io.c').read_text(encoding='utf-8')
    signatures = (
        'uint32_t RouteIO_Millis(', 'void RouteIO_ReceiveCan(', 'void RouteIO_PublishLidar(',
        'void RouteIO_ReadNavigationLidar(', 'void RouteIO_ReadLidar(',
        'static uint8_t take_request(', 'uint8_t RouteIO_TakeStartRequest(',
        'uint8_t RouteIO_TakeStopRequest(', 'void RouteIO_SetRunning(',
        'void RouteIO_SetHeading(', 'void RouteIO_SetVelocity(', 'float RouteIO_ApplyDrive(',
        'uint8_t RouteIO_QueueWorkFromISR(', 'uint8_t RouteIO_TakeWorkRequest(',
        'void RouteIO_AllowWork(', 'void RouteIO_CompleteWork(')
    (BUILD / 'navigation_io.inc').write_text(
        ''.join(extract(route_io, signature) for signature in signatures), encoding='utf-8')
    navigation = (FW / 'USER/my_chassis_task.c').read_text(encoding='utf-8')
    opening = navigation.index('{', navigation.index('for (;;)'))
    loop = navigation[opening+1:navigation.index('vTaskDelayUntil', opening)]
    (BUILD / 'navigation_control.inc').write_text(loop.rstrip()+'\n', encoding='utf-8')
(BUILD / 'test.c').write_bytes(test_file.read_bytes())
environment = os.environ.copy()
environment['PYTHONPATH'] = str(ROOT / 'tests/camera/.build/python_deps') + os.pathsep + environment.get('PYTHONPATH','')
subprocess.run([sys.executable, str(ROOT / 'tests/mango/run_tests.py'), str(BUILD / 'test.c')],
                cwd=ROOT, env=environment, check=True)
