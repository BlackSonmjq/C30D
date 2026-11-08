# ZP10S 标准库接口

驱动：`HARDWARE/servo.c`、`HARDWARE/servo.h`。使用 UART4、PC10 单线半双工，115200、8N1；系统启动调用 `Servo_Init()`，只初始化通信，不发送动作或修改配置。四个舵机 ID 使用 1、2、3、4，与原推杆编号对应。第 4 路激光 XSHUT 已从 PC10 迁至 PA7，包括上电初始化、测距任务和 OLED 显示。

本次先提供接口，自动采摘协调任务仍使用原推杆流程，尚未调用舵机动作。不要把现有串口采摘命令当作舵机测试命令。

## 接线与一次性配置

- 四个舵机的 S 信号接 PC10 总线，舵机电源按型号要求单独供电，与主控共地；PC10 为开漏复用，需保证总线上拉和电平与 STM32 兼容，勿用 MCU GPIO 给舵机供电。
- 激光 4 的 XSHUT 实物改接 PA7，其余三路仍为 PA2、PA3、PA6。
- 出厂 ID 通常为 0：一次只接一只舵机，依次调用 `Servo_SetID(0, 1)` 至 `Servo_SetID(0, 4)`。确认回包 `#001P!` 等，再接下一只，最后并联总线。
- 对每个 ID 显式调用 `Servo_SetMode(id, 1)`，并确认 `#OK!` 后再发动作。模式 1 是 270° 舵机模式；不要每次启动重复写 ID 或模式。实际安装方向先空载核对，模式 2 为反向的 270° 模式。

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

`time_ms` 范围 0～9999；默认常量 `SERVO_MOVE_MS=1500` 仅供初始空载调试。下例假设 ID 和模式已配置并核对，运行在单一控制任务中：

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

发送接口的 `SERVO_OK` 只表示串口发送完成，不代表舵机已确认、已到位或已脱果。需要确认时由同一任务串行发送请求、轮询回包并处理超时；切换模式后先消费确认，再发下一条指令。自动流程后续应加到位和故障判断，不能仅凭发送成功返程。

驱动等待 TC 后切回接收，最后一个字节及收发切换期间短暂屏蔽中断，正常约 87 微秒；硬件异常使用有限轮询退出。发送仅限任务上下文，并发发送返回 BUSY；请求/回包事务由调用方串行管理。所有常用动作接口只接受 ID 1～4，原始 SendCommand 由调用方负责内容。

接收邮箱容量 64 字节，支持 `#OK!` 和带 ID 的帧。未读帧保留时后续帧丢弃；缓冲区不足返回 BUFFER_TOO_SMALL 并保留原帧。`length` 不含 NUL。ClearRx 可在开始新事务前清除旧帧，调用前需确认上一事务结束。RXNE 中断持续接收，溢出或串口错误后重新寻找帧头。

## 仿真验证

在仓库根目录运行（与 mango 测试共用输出目录，顺序执行）：

```powershell
uv run --with unicorn --with pyelftools python tests/mango/run_tests.py tests/servo/test_servo.c
uv run --with unicorn --with pyelftools python tests/mango/run_tests.py
```

测试编译生产驱动并模拟 UART/GPIO，覆盖四个 ID、角度换算、命令格式、TC 等待、超时恢复、并发拒绝、回包容量、噪声、溢出与错误恢复。不能替代电气及实物运动验证。
