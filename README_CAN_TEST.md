# S21C -> C30D CAN 测试

## 协议

S21C 每 100 ms 发送一帧 CAN 标准数据帧：

- CAN1，Normal 模式
- 标准 ID `0x301`
- `DLC = 8`
- `data[0..3]`：小端 `uint32_t counter`，从 1 开始递增
- `data[4..7]`：`12 34 56 78`

C30D 只将 ID `0x301`、标准数据帧、DLC 8 且固定字节正确的帧计入测试统计。接收、丢帧统计在 CAN RX0 中断中完成，在线超时和调试观察在普通任务中完成。

## 当前 CAN 配置

| 工程 | MCU | CAN | TX / RX | CAN 时钟 | SJW / BS1 / BS2 / Prescaler | 波特率 | 模式 | FIFO / IRQ |
|---|---|---|---|---:|---|---:|---|---|
| C30D | STM32F407ZG | CAN1 | PD1 / PD0 | 42 MHz APB1 | 1 / 6 / 7 / 3 | 1 Mbps | Normal | FIFO0 / CAN1_RX0_IRQHandler |
| S21C | STM32F103HD | CAN1 | PA12 / PA11 | 36 MHz APB1 | 1 / 3 / 2 / 6 | 1 Mbps | Normal | FIFO0 / USB_LP_CAN1_RX0_IRQHandler |

两边波特率一致，计算分别为 `42 MHz / (3 * (1 + 6 + 7))` 和 `36 MHz / (6 * (1 + 3 + 2))`。C30D 的既有底盘 CAN 配置为 1 Mbps，因此只将 S21C 活跃的 `CAN_Comm_Init` 参数从原 500 kbps 调整为同样的 1 Mbps；S21C 的旧 `CAN1_Mode_Init` 参数本来也为 1 Mbps。C30D 当前过滤器允许所有帧进入 FIFO0，S21C 活跃驱动保留原有精确 0x302 接收过滤器；C30D 的 0x301 分发已加入现有 RX0 入口。

## 硬件连接

```text
C30D CAN_H <----> S21C CAN_H
C30D CAN_L <----> S21C CAN_L
C30D GND   <----> S21C GND
```

两块板可以独立供电。不能将 CAN_H 与 CAN_L 互接，必须共地。总线两端各使用 120 ohm 终端电阻。只有两个节点时断电测量 CAN_H 与 CAN_L 应约为 60 ohm；约 120 ohm 表示缺少一个终端，约 40 ohm 可能存在三个终端。

## 观察变量

C30D：

- `g_can_test_rx_count`
- `g_can_test_last_counter`
- `g_can_test_lost_count`
- `g_can_test_error_count`
- `g_can_test_online`

S21C：

- `g_can_test_tx_ok`
- `g_can_test_tx_error`

上电运行至少 30 秒。正常情况下 S21C 发送成功计数持续增加，C30D 收到约 300 帧、`ONLINE = 1`、固定字节错误为 0，理想情况下丢帧为 0。

## 文件

C30D 新增 `HARDWARE/can_test.c`、`HARDWARE/can_test.h`，并接入 `USER/main.c`、`USER/WHEELTEC.uvprojx` 与现有 `HARDWARE/can.c`。

S21C 新增 `HARDWARE/can_test.c`、`HARDWARE/can_test.h`，并接入 `USER/main.c`、`USER/UART1.uvprojx`；S21C 的 CAN 驱动继续使用现有 `HARDWARE/can_comm.c`。本阶段未接入正式四路激光协议。

后续切换正式协议时，只需保留 CAN 初始化、邮箱超时/完成统计和 RX 分发框架，将 S21C `CAN_Test_Send` 的 8 字节 payload 替换为四个 `uint16_t` 距离，并在 C30D 的 0x301 解析分支替换测试固定字节校验为距离字段校验。

## 串口1调试输出

S21C 串口1每秒输出：

```text
CAN TEST TX OK=123 TX ERROR=0
```

C30D USART1 每秒输出：

```text
CAN TEST RX=123 LAST=123 LOST=0 ERROR=0 ONLINE=1
```

测试期间已停止原有 USART1 二进制周期发送，避免与英文 CAN 状态行混流。
