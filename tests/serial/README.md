> Current wiring (supersedes the configuration below): distance input is USART1 RX (PA10); raw bytes are forwarded on USART3 TX (PD8). USART1 TX (PA9) prints rx1 distance/waiting diagnostics every 500 ms at 115200 8N1, even without input. Connect adapter RX to PA9 and share GND. At startup it prints boot: detecting IMU. The USART3 input and rx3 examples below describe the earlier configuration.

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
