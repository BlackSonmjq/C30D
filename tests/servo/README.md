# ZP10S 标准库接口

驱动：`HARDWARE/servo.c`、`HARDWARE/servo.h`。使用 UART4、PC10 单线半双工，9600、8N1；系统启动调用 `Servo_Init()`，只初始化通信，不发送动作或修改配置。四个舵机 ID 使用 1、2、3、4，与原推杆编号对应。第 4 路激光 XSHUT 已从 PC10 迁至 PC11，包括上电初始化、测距任务和 OLED 显示。

自动采摘协调任务通过 `ServoTask_*` 接口调用舵机：确认零位、转到 180°、恢复零位，位置回读确认后再进行步进返程。每段使用代码中的 `SERVO_MOVE_MS`（当前 500 ms），超时为 3 秒。舵机需预先配置为 270° 位置模式：1/2 号模式 2，3/4 号模式 1；自动任务不改写配置。串口裸字符 `1～4` 目前仍仅选激光，不触发自动任务。自动任务期间拒绝手动舵机动作和接收诊断，手动序列占用总线时自动舵机请求失败并沿用原返程处理。

## 接线与一次性配置

- 四个舵机的 S 信号接 PC10 总线，舵机电源按型号要求单独供电，与主控共地；PC10 为开漏复用，需保证总线上拉和电平与 STM32 兼容，勿用 MCU GPIO 给舵机供电。
- 激光 4 的 XSHUT 实物改接 PC11，其余三路仍为 PA2、PA3、PA6。
- 出厂 ID 通常为 0：一次只接一只舵机，依次调用 `Servo_SetID(0, 1)` 至 `Servo_SetID(0, 4)`。确认回包 `#001P!` 等，再接下一只，最后并联总线。
- 按当前机构方向配置：1/2 号使用模式 2，3/4 号使用模式 1。可分别发送 `M1`～`M4` 设置，并等待 `CONFIRMED` 回读确认。不要每次启动重复写 ID 或模式。实际安装方向先空载核对。

## Baud discovery and recovery (baud-probe-7)

UART4 starts at 115200 for recovery. Upper USART1 stays at 115200 8N1.
Send B? first: it only queries modes at 9600/115200, prints raw replies,
and records the confirmed rate for each ID in MCU RAM. Example:
mask9600=0x00 mask115200=0x0F means all four responded at 115200.
No ID, mode or baud configuration is written by B?. S1-S4 also try both
rates for the selected ID and retain the successful rate. For automatic K
jobs with mixed rates, run B? after boot before the first job.

B0 still attempts migration: only an ID confirmed at 115200 gets PBD0.
It now captures any old-rate ACK before switching, verifies at 9600, then
re-checks 115200 after a failed verification. The final log reports both
masks. Failure no longer leaves all communication forced to 9600. Typed
motion, position query and stop APIs select each ID's confirmed rate.
No repeated writes occur for IDs already confirmed at 9600.

The prior fault latch and travel guard remain removed as requested.
Original feedback/retry behavior remains: +/-100 units, 500 ms moves,
two valid arrival samples, bounded motion retries and return attempt.
DMA/TC interrupt turnaround avoids slow-byte waits with IRQs masked.

## 动作含义

| 接口 | 含义 |
| --- | --- |
| `Servo_Home(id, time_ms)` | 绝对位置 P0500，作为机构零位 |
| `Servo_Twist(id, time_ms)` | 绝对位置 P1833，约为零位起转 180° |
| `Servo_MoveAngle(id, degrees, time_ms)` | 270° 模式下的绝对角度，0～270° |
| `Servo_MovePulse(id, pulse, time_ms)` | 原始位置参数 500～2500 |
| `Servo_Stop(id)` | PDST，停止当前运动 |
| `Servo_ReadPosition(id)` | 请求位置回读，使用 GetFrame 获取 `#001P1833!` 等回复 |
| `Servo_GetFrame(buffer, capacity, &length)` | 获取一帧 ASCII 回包，自动补 NUL |

270° 模式换算：`P = 500 + round(角度 × 2000 / 270)`，因此半圈为 P1833；P1500 是约 135°，P2500 是约 270°。位置换算是名义值，实际零位和转角需要机构联调。

`time_ms` 范围 0～9999；当前常量 `SERVO_MOVE_MS=500` 供自动动作及调试使用。下例假设 ID 和模式已配置并核对，运行在单一控制任务中：

```c
#include "servo.h"

/* 例：让 1 号从零位转半圈。Servo_Init 已由 systemInit 调用。 */
if (Servo_Home(1, SERVO_MOVE_MS) == SERVO_OK) {
    vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_MS + 100U));
    if (Servo_Twist(1, SERVO_MOVE_MS) == SERVO_OK) {
        vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_MS + 100U));
        /* 后续可读取位置确认；返程/复位由上层流程安排。 */
    }
}
```

Twist 是绝对位置接口：连续调用不会每次再增加半圈。每轮应先回零，等回零完成后再绞取。需要回零复位时，在确认脱果且机构允许后再次调用 Home。

## 返回值和接收约定

发送接口的 `SERVO_OK` 只表示串口发送完成，不代表舵机已确认、已到位或已脱果。需要确认时由同一任务串行发送请求、轮询回包并处理超时；切换模式后先消费确认，再发下一条指令。自动任务已加入位置回读和超时判断，正常流程确认回零后才返程；故障仍按原协调流程记录错误并尝试步进返程。

Startup resets UART4 and leaves RX disabled. DMA sends frames with interrupts enabled; UART TC completion immediately switches to RX. The task scheduler guard spans the send, but there is no global interrupt-masked wait for slow bytes. Finite polling handles peripheral errors. Send APIs remain task-context only and requests/replies must be serialized.

接收邮箱容量64字节。普通动作、设置指令及接收诊断保留首帧接收规则。PRAD位置查询只接受同编号的合法 `#002P1833!` 类位置帧；PMOD模式查询接受同编号的 `#002MOD2!` 或 `#002PMOD2!`。查询期间忽略ACK、指令回显、错误编号和格式错误的帧，继续接收，匹配帧到达后才关闭接收。缓冲区不足返回 BUFFER_TOO_SMALL 并保留原帧，`length` 不含NUL。每次发送前需先消费或清除上一回包。串口错误、帧缓冲区溢出或单次累计输入达到128字节时仍关闭接收，包括连续无效帧，避免持续异常中断；下次成功发送可重新开启。没有回包时由调用方计时，超时调用 `Servo_ClearRx()` 取消接收并清空邮箱。

## 串口1接收诊断

重新构建烧录后，串口1设置为 115200、8N1、文本模式，发送小写 `s`。任务开启一次 500 ms 的 UART4 接收窗口，不发送任何舵机指令。收到完整帧、串口错误或输入达到字节上限会提前停止接收；窗口结束时仍会统一打印结果、关闭接收并清空诊断期间的帧。正在发送、等待正常回包或有未读帧时拒绝诊断，正在诊断时拒绝发送。

输出 `servo probe irq=... bytes=... errors=... flags=0x... sr=0x... rx=... pc10=...`：`irq` 为本次中断次数，`bytes` 为无串口错误的接收字节数，`errors` 为错误中断次数，`flags` 为累计错误位（PE=0x01、FE=0x02、NE=0x04、ORE=0x08），`sr` 为最后一次中断入口状态，`rx` 为窗口结束前接收是否仍开启，`pc10` 为此刻引脚采样电平。`rx=1` 表示到窗口结束前未被提前关闭，报告后会关闭。正常未接设备且线路保持空闲时，预期 `irq=0 bytes=0 errors=0 pc10=1`；单次引脚采样不能排除短暂毛刺。

请同时记录 OLED 是否保持显示，并保留开始和结果两行原始输出，以便区分线路电平、帧错误及中断触发情况。若发生死机或复位导致没有结果行，先记录最后一行输出。

## 串口日志

M1～M4 在模式查询超时时输出 `mode rx attempt=... irq=... bytes=... errors=... flags=... receiving=... pc10=...`，计数为本次查询窗口的增量（flags 为累计错误位）。没有完整帧时，`bytes=0` 表示没有接收字节；`bytes>0` 表示收到字节但未得到可识别的完整模式回复，需结合 `errors` 和已有 `mode raw` 日志判断。该日志不改变动作流程。

串口1只输出 `SERVO ` 或 `servo ` 开头的舵机日志，关闭激光周期数据、APP `{A…}/{B…}` 和启动信息。自动任务记录目标、位置反馈、错误和完成状态；`phase=0/1/2` 分别表示初始回零、转动180度、返回零位。串口接收指令、测距和OLED显示保持不变。

## 仿真验证

在仓库根目录运行（与 mango 测试共用输出目录，顺序执行）：

```powershell
uv run --with unicorn --with pyelftools python tests/mango/run_tests.py tests/servo/test_servo.c
uv run --with unicorn --with pyelftools python tests/mango/run_tests.py
uv run --with unicorn --with pyelftools python tests/serial/test_servo_debug.py
uv run --with unicorn --with pyelftools python tests/serial/test_servo_transport.py
```

测试编译生产驱动并模拟 UART/GPIO，覆盖四个 ID、角度换算、命令格式、TC 等待、发送超时、并发拒绝、启动关闭接收、旧外设状态复位、回包容量、连续噪声、溢出、串口错误关闭接收及下一请求恢复。不能替代电气及实物运动验证。

## Firmware version query (baud-version-8)

The supplied Zhongling PWM servo manual, revision 250508, includes ZP10S
in the bus-servo table (p12). Pages 25-27 define PVER, MOD2 and PBD0:
MOD2 is 270-degree reverse position control; PBD0 requests 9600 with a
PBD9600 response. The relevant command descriptions do not state a separate
save/restart requirement. Observed generic OK plus continued 115200 mode
readback differs from this description; do not infer a firmware version.

V0 reads versions of IDs 1-4; V1-V4 read one. These commands send only
#002PVER! (matching ID), trying known and alternate rates. They never move
a servo or write ID/mode/baud. A valid version looks like #002PV0.97!.
Raw replies are logged; OK, echoes and wrong IDs do not count as a version.
The receiver continues listening for a trailing version after an ACK.
