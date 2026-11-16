# 原蓝牙模块连接 VOFA

路径联调记录版位于 `D:\Keil_Project\C30D_EIDE\dist\BLE-VOFA-Logger\WHEELTEC-BLE-VOFA.exe`。
运行时自动在 EXE 同目录的 `logs` 文件夹创建带时间戳的 CSV，包含电脑接收时间和全部 64 个遥测字段，每帧刷新到磁盘。断线重连仍写入本次文件，终端显示路径点、状态及故障原因变化。该文件可在程序运行期间读取分析；关闭旧版转发器后再运行记录版，避免争用 BLE 和 UDP 9001。

## Windows 独立可执行文件

已打包到 `D:\Keil_Project\C30D_EIDE\dist\BLE-VOFA\WHEELTEC-BLE-VOFA.exe`。
双击即可扫描、连接并转发，无需安装 Python 或 bleak；可单独复制 EXE 到其他 Windows x64 电脑。
运行前断开手机连接，开启电脑蓝牙。保留黑色状态窗口，Ctrl+C 停止。
VOFA 使用 JustFloat、UDP，本地端口 9000；远端如需填写为 127.0.0.1:9001。
命令行参数与 Python 版相同，`--self-test` 可检查内置依赖、分片重组及 UDP 收发，不连接实车。

当前实测手机可从 WHEELTEC-IOS 的 FFE0 / FFE1 Notify 收到字节。
底盘 USART2 串口侧保持 9600、8N1，每 271 ms 输出 64 通道 JustFloat。
电脑通过 BLE 订阅通知，重组为完整 260 字节帧，再发往本机 UDP。
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
依赖 API 见 [Bleak 官方文档](https://bleak.readthedocs.io/en/stable/api/client.html)。

## VOFA 配置

- 数据引擎：JustFloat。
- 数据接口：UDP（监听/服务器模式，如版本提供模式选项）。
- 本地监听地址：127.0.0.1 或 0.0.0.0；本地端口：9000。
- 若界面要求远端地址和端口，填写 127.0.0.1、9001。
- 打开接收。无需设置串口波特率，也无需发送启动指令。
- 波形时间步长设为 271 ms，与固件发送周期一致。

转发器绑定本机 9001，发送到 VOFA 本机 9000；不能把两个程序都绑定 9000。
需要换端口时运行 `python tools/ble_vofa_bridge.py --port 其他VOFA端口 --local-port 其他源端口`。
64 个通道编号见原 VOFA 跑图诊断文档；6～9 仍是实际车体前后左右激光。

## 排查

- 找不到设备：检查模块供电、电脑蓝牙，并释放手机或其他电脑的连接。
- RX=0：已订阅但没有通知，检查烧录版本、模块 UART 速率及 PD5 接线。
- RX 增长而 frames=0：没有解析到有效 64 通道完整帧，检查串口速率、丢包及所用固件。
- frames 持续增加而 VOFA 无曲线：检查 UDP 本地监听端口和 JustFloat 引擎。
- discarded 持续增加：存在非遥测字节或丢失片段；工具只转发通过帧长度及字段检查的数据。

自动测试：`python tests/vofa/test_ble_bridge.py`。该测试验证分片、噪声、缺失片段、连续帧和本机 UDP，不替代实际 BLE 稳定性测试。
