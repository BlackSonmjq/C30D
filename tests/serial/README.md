Send B0 once after flashing to migrate servo IDs 1-4 from 115200 to 9600; wait for confirmed_mask=0x0F. Upper USART1 stays 115200. Repeating B0 resumes partial migrations without rewriting confirmed devices. Fault latch/travel guard are removed; the previous bounded feedback/retry flow is restored.

> Current console: USART1 RX (PA10) accepts local task/servo commands; USART3 PD8/PD9 is reserved for the camera protocol. USART1 TX (PA9) prints servo logs only at 115200 8N1. Laser, APP and boot prints are disabled. The distance/rx3 examples below describe earlier configurations.

# 串口1任务指令

串口1接收端 PA10、发送端 PA9，115200、8N1，使用文本发送模式：

| 指令 | 动作 |
| --- | --- |
| `K1` / `K2` | 执行 1 / 2 号完整任务，使用左步进及同编号激光、舵机 |
| `K3` / `K4` | 执行 3 / 4 号完整任务，使用右步进及同编号激光、舵机 |

击打距离分别为：`K1` 小于 108 mm，`K2` 小于 60 mm，`K3` 小于 45 mm，`K4` 小于 35 mm。

流程为测距、靠近至有效距离小于对应编号阈值、舵机正转 180° 并回零、步进等脉冲返程、关闭激光。等于阈值时继续靠近。指令无需换行，可追加 CR/LF，也可分两次发送 `K` 和编号。只识别大写 `K` 和编号 `1～4`；忙碌时不切换目标、不排队。连续发送 `K1K2` 时，首个任务立即占用，第二个被拒绝。指令在本地消费，不通过 USART3 透明转发。UART 接收错误会清除未完成的指令前缀。

裸数字 `1～4` 仍仅选激光，`S0`～`S4`、`M1`～`M4` 保持现有舵机调试功能。

S1-S4 and K1-K4 use feedback-confirmed home -> 180 degrees -> home, 500 ms, +/-100. Missing replies preserve valid arrival counts; stalled targets retry at most twice within 3 seconds. K2 no longer uses timed completion.

S1-S4 require a confirmed position mode (1-4); after three unanswered mode queries, motion is cancelled and the busy flag is released. UART4 query reception filters ACKs, echoes, wrong IDs and malformed frames until a matching reply or the existing limits. Whole-frame TX suspends task scheduling while keeping IRQs enabled except for the final direction transition. UART4 IRQ preemption priority remains 2.

During S2 outward motion, a valid position of 541 triggers up to three resends of the current absolute target, spaced at least 500 ms apart. Each resend allows 500 ms of motion before querying again; the original phase deadline is retained. Other positions, homing phases, other IDs and K2 do not trigger this retry.

构建 EIDE 固件后，可仿真执行编译后的 USART1 中断入口：

```powershell
uv run --with unicorn --with pyelftools python tests/serial/test_task_commands.py
```

测试覆盖四个编号、分次发送、CR/LF、连续请求的忙碌拒绝、无效编号、重复前缀、UART 错误恢复及原数字/舵机调试指令。

# 串口3测距接收协议

本板是接收端。正常运行模式（`Check == 0`）下，USART3 使用 **115200、8N1**，接收外部设备发送的 **11 字节二进制帧**。四路距离为16位无符号数，单位 mm，高字节在前。图中标注的 `1Bit` 按一个字节（8 bit）实现。

| 字节序号（从1开始） | 内容 |
| --- | --- |
| 1 | 帧头，固定 `0x7B` |
| 2、3 | 接口1距离，高8位、低8位 |
| 4、5 | 接口2距离，高8位、低8位 |
| 6、7 | 接口3距离，高8位、低8位 |
| 8、9 | 接口4距离，高8位、低8位 |
| 10 | 校验：第1～9字节逐字节异或（包含帧头） |
| 11 | 帧尾，固定 `0x7D` |

例如外部发送接口1～4分别为1000、2000、3000、4000 mm：

```text
7B 03 E8 07 D0 0B B8 0F A0 5B 7D
```

串口助手必须使用十六进制发送模式，不要发送上述数字的 ASCII 文本。发送周期由外部设备决定；本板不限制接收为500 ms一次，也不等待开机10秒才开始接收。

只有完整帧通过帧头、帧尾和异或校验后，才同时发布四路距离。支持逐字节分批到达、连续多帧、忽略帧前杂字节，以及错帧后重新寻找帧头。数据和校验字节也可能等于 `7B` 或 `7D`，不会因此提前截帧。UART硬件错误时清除当前未完成帧，保留上一份已校验数据。图片中的 RCC 按文字说明实现为异或校验，不是 CRC8。

图片没有规定无效值，接收端保留全部原始数值，包括 `0000` 和 `FFFF`；不根据本地激光是否开启改写外部数据。报文有效只表示通信校验通过，不表示传感器测量有效。

## 任务中读取

```c
#include "usartx.h"

DistanceProtocol_Data data;
USART3_GetDistanceData(&data);
if (data.received) {
    /* data.distance_mm[0..3]：外部接口1～4的距离，单位mm。
     * data.frame_count：累计接收成功帧数，每成功一帧加1。 */
}
```

读取接口通过保存、恢复 PRIMASK 取得一致快照，适用于任务上下文。`received` 表示至少接收过一帧，不代表数据仍然新鲜；应用可观察 `frame_count` 是否变化，自行判断超时。读取不会清除数据，也不会消费帧计数。

解析器位于 `HARDWARE/distance_protocol.h`，接收入口为 `HARDWARE/usartx.c` 的 `USART3_IRQHandler()`。串口3不再周期发送测距帧，也不再将收到的11字节解释为原ROS底盘、回充或IP控制帧；这些帧与新协议缺少可靠的区分标识。串口3原有自检文本应答及串口1小写 `s` 舵机诊断保留。

## 串口任务输出

`data_task()` 已使用串口3接收的四路快照，每500 ms通过串口1输出一次（115200、8N1、文本显示）。收到上述示例报文后，输出类似：

```text
rx3 d1=1000 d2=2000 d3=3000 d4=4000 mm frames=1 age_ms=0
```

`frames` 为累计校验通过帧数；`age_ms` 是串口任务观察到最近一帧后的毫秒数，包含50 ms任务轮询带来的观测误差。断开外部设备后，保留最后收到的数值及帧数，`age_ms` 持续增加，不会将旧数据标记为新数据。未收到过完整合法帧时输出：

```text
rx3 waiting bytes=0 err=0 flags=0x00 xor=0 tail=0 mode=0 pd9=1 raw=
```

四路同时显示，不需要串口1输入数字选通；串口任务不再响应原 `0～4` 文本命令控制本地 XSHUT。串口1原有二进制底盘控制接收逻辑和 `s` 诊断仍保留。

等待期间的诊断：`bytes` 为串口3读取的累计接收/错误事件数，`err` 为UART错误事件数，`flags` 为累计错误位（PE=1、FE=2、NE=4、ORE=8）；`xor`/`tail` 为解析窗口校验/帧尾失败次数；`mode=0` 为正常接收，`mode=1` 为自检；`pd9` 为串口3 RX引脚的瞬时电平；`raw` 为最近最多16个字节的十六进制记录，按接收顺序显示。错误窗口可能包含杂字节，不等于外部发送帧数。

`bytes` 一直为0时优先检查外部TX是否接主控 **PD9（USART3_RX）**、共地及外部是否实际输出；字节增长且UART错误增加时核对115200/8N1及电平；有字节但 `xor`/`tail` 增长时保留 `raw` 原始记录检查报文格式。单次电平采样不能排除短暂毛刺或替代波形测量。

本地 `distance_task`、OLED 和电机任务仍使用原有本地测距路径，未自动切换为外部数据。

## 仿真验证

在仓库根目录运行（与其它仿真共用构建目录，顺序执行）：

```powershell
uv run --with unicorn --with pyelftools python tests/mango/run_tests.py tests/serial/test_distance_protocol.c
```

测试覆盖四路距离、零值和无符号高位、校验和帧尾拒绝、半帧、连续帧、字节丢失、杂字节、连续假帧头、包含帧头/尾的数据，以及错误后恢复。串口电气通信仍需实机确认。
