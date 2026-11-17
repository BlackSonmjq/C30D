# 原蓝牙模块连接 VOFA

当前固件发送 32 通道：0～7 为原视觉数据，8～19 为对中诊断，20～31 为四轮目标速度、实际速度及 PWM。接收工具另兼容历史 8/20/48/64/72 通道固件。
运行时自动在 EXE 同目录的 `logs` 文件夹创建带时间戳的 CSV，包含电脑接收时间和实际收到的全部遥测字段，每帧刷新到磁盘。断线重连且通道数相同时仍写入本次文件；若烧录后通道数改变，工具结束，需重新启动以创建新表头。旧日志不改写。终端显示路径点、状态及故障原因变化。该文件可在程序运行期间读取分析；关闭旧版转发器后再运行新版，避免争用 BLE 和 UDP 9001。

## Windows 独立可执行文件

新版打包输出为 [WHEELTEC-BLE-VOFA-Load.exe](../../C30D-develop-EIDE/build/vofa-load/WHEELTEC-BLE-VOFA-Load.exe)。先关闭旧转发器，再启动 Load 版本或当前 Python 源码；旧 Align/Vision EXE 不支持新 32 通道布局。
双击即可扫描、连接并转发，无需安装 Python 或 bleak；可单独复制 EXE 到其他 Windows x64 电脑。
运行前断开手机连接，开启电脑蓝牙。保留黑色状态窗口，Ctrl+C 停止。
VOFA 使用 JustFloat、UDP，本地端口 9000；远端如需填写为 127.0.0.1:9001。
命令行参数与 Python 版相同，`--self-test` 可检查内置依赖、分片重组及 UDP 收发，不连接实车。

当前实测手机可从 WHEELTEC-IOS 的 FFE0 / FFE1 Notify 收到字节。
底盘 USART2 串口侧保持 9600、8N1，当前输出 32 通道 JustFloat，共 132 字节。最小尝试间隔仍为 271 ms，结合 50 ms 调度，实际预计约 300 ms 一帧。
电脑通过 BLE 订阅通知，重组为完整帧，再发往本机 UDP；旧 8/20/48/64/72 通道固件也可使用同一工具。CSV 以 `vision-` 开头，包含 `received_at` 和实际通道字段，终端显示 B/DX、对中状态、LY/RX、底盘指令及四轮 PWM。
无需蓝牙 COM 口，也不需要修改模块配置。当前转发仅接收遥测，不支持在线调参。

## 启动

先在手机 nRF Connect 点击 Disconnect，关闭 WHEELTEC APP，开启电脑蓝牙。
在源码仓库根目录打开终端，使用 Python 3.10 或更新版本：

```powershell
python -m pip install bleak
python tools/ble_vofa_bridge.py
```

默认设备名 WHEELTEC-IOS；有多个同名设备时用 `--address 蓝牙地址` 指定。
工具扫描后自动连接、订阅，断线后重新扫描。Ctrl+C 退出。
每次连接先用连续两帧确认通道数，确认后两帧一起转发；这会增加约一帧的起始等待，避免从新帧中途接入时误识别为旧 48/64 通道。之后逐帧转发。
依赖 API 见 [Bleak 官方文档](https://bleak.readthedocs.io/en/stable/api/client.html)。

## VOFA 配置

- 数据引擎：JustFloat。
- 数据接口：UDP（监听/服务器模式，如版本提供模式选项）。
- 本地监听地址：127.0.0.1 或 0.0.0.0；本地端口：9000。
- 若界面要求远端地址和端口，填写 127.0.0.1、9001。
- 打开接收。无需设置串口波特率，也无需发送启动指令。
- 当前 32 通道的波形时间步长可先设为 300 ms；以电脑实际接收时间为准。

转发器绑定本机 9001，发送到 VOFA 本机 9000；不能把两个程序都绑定 9000。
需要换端口时运行 `python tools/ble_vofa_bridge.py --port 其他VOFA端口 --local-port 其他源端口`。
当前 32 通道及单位见 [VOFA 输出](VOFA跑图诊断.md)。对中观察 8/9（状态/自动控制）、15/16（横移/IMU）；负载补偿观察 20～23（目标）、24～27（实际轮速）、28～31（PWM），均按 A/B/C/D 排列。

## 排查

- 找不到设备：检查模块供电、电脑蓝牙，并释放手机或其他电脑的连接。
- RX=0：已订阅但没有通知，检查烧录版本、模块 UART 速率及 PD5 接线。
- RX 增长而 frames=0：没有解析到有效 8/48/64/72 通道完整帧，检查串口速率、丢包及所用固件。
- frames 持续增加而 VOFA 无曲线：检查 UDP 本地监听端口和 JustFloat 引擎。
- discarded 持续增加：存在非遥测字节或丢失片段；工具只转发通过帧长度及字段检查的数据。

自动测试：`python tests/vofa/test_ble_bridge.py`。该测试验证分片、噪声、缺失片段、连续帧和本机 UDP，不替代实际 BLE 稳定性测试。
