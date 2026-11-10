# S21C 实际距离输出

完整协议、接线、方向命名、字节处理入口及烧录说明已整合到[总 README](../../README.md#s21c-四向测距通信)，以总 README 为准。

实际工程：S21C_EIDE/S21C_Source；接收端：C30D-develop-EIDE。

两块板均需烧录新版固件。旧测试帧与新距离帧使用相同 ID，不能混用。

- CAN：标准数据帧，ID `0x301`，DLC 8，1 Mbps，每 100 ms 发送。
- 数据：四路无符号 16 位距离，单位 mm，每路高字节在前。
- USART1：115200、8N1，每 100 ms 发送 11 字节；`7B D1H D1L D2H D2L D3H D3L D4H D4L XOR 7D`。XOR 为前 9 字节逐字节异或。
- CAN 与串口使用同一份距离快照。S21C 串口不再混入启动文字或测试统计；助手使用十六进制显示。
- C30D 当前串口每 500 ms 显示本地上层激光 `UPPER VL53L0X laser=... distance=... mm valid=... status=... age_ms=...`；CAN 四向距离仍接收，但不再周期打印。

例：1000、2000、3000、4000 mm，CAN 数据为 `03 E8 07 D0 0B B8 0F A0`，串口为 `7B 03 E8 07 D0 0B B8 0F A0 5B 7D`。

距离沿用现有传感器解析结果。当前协议不携带各路有效标志或更新时间；尚未收到测量时可能为 0，断开传感器后也可能保留旧值。`ONLINE` 只表示 CAN 报文近期到达，不保证各传感器读数有效。当前只更新 CAN 接收与显示，不将外部距离接入电机控制。

## Direction mapping

| Channel | Name | CAN bytes (0-based) | UART bytes (0-based) |
|---|---|---|---|
| D1 | Left | 0, 1 | 1, 2 |
| D2 | Front | 2, 3 | 3, 4 |
| D3 | Right | 4, 5 | 5, 6 |
| D4 | Behind | 6, 7 | 7, 8 |

Names only; the wire format and channel order are unchanged.
