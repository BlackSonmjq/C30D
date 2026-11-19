# 蓝牙激光遥测与 VOFA 使用说明

本工程用蓝牙发送四向激光、导航进度、轮速和车辆状态，共 **48 通道**。电脑接收后把每个完整 JustFloat 帧转发给 VOFA，并逐帧保存 CSV。工具只接收遥测，不能启动车辆或改变速度。

链路：`C30D USART2 → 小车 BLE 模块 → 电脑 BLE Notify → 帧重组 → 本机 UDP → VOFA`。

## 1. 文件在哪里

以下路径相对于 `C30D_rebuild` 工程根目录：

| 文件 | 用途 |
| --- | --- |
| `build/vofa-align/WHEELTEC-BLE-VOFA-Align.exe` | Windows x64 双击运行，已包含 Python、bleak 和 WinRT 依赖 |
| `build/vofa-align/VOFA通道表.md` | 索引 0～47、单位、状态及原因编码 |
| `build/vofa-align/logs/` | 双击 EXE 后自动创建，保存每次运行的 CSV |
| `OBJ/C30D_rebuild.hex`、`.bin` | 本次编译的 C30D 固件，需自行烧录后才会发送新协议 |
| `USER/nav_vofa.c` | 48 通道数值及小端编码 |
| `USER/nav_ble_uart.c` | USART2 中断发送，不等待整帧发送完成 |
| `tools/ble_vofa_bridge.py` | 电脑端入口 |
| `tools/vofa_protocol.py`、`vofa_legacy.py` | 帧重组与新旧通道布局 |

原先建议的 `C30D-develop-EIDE` 目录不在当前工程中，因此本次统一输出至当前工程的 `build/vofa-align/`。保留队友约定的 EXE 名称，默认内容改为本车的 `nav48`，没有发送队友的相机对中 20 通道。

## 2. 先检查小车

1. 将本次生成的固件烧录到 C30D；本次开发没有连接下载器或烧录。
2. 模块与 C30D 间仍为 **USART2，9600、8N1**，TX=PD5、RX=PD6。电脑通过 BLE 接收，不打开蓝牙 COM 口，也不用在 VOFA 中配置串口波特率。
3. 默认 BLE 名称 `WHEELTEC-IOS`，服务 `0000ffe0-0000-1000-8000-00805f9b34fb`，Notify 特征 `0000ffe1-0000-1000-8000-00805f9b34fb`。实际模块须支持这些服务。
4. **断开手机 APP 和其他电脑对该模块的连接**，再启用本机蓝牙、给小车供电。
5. 当前 `NAV_CALIBRATED=0` 保持原值，仍然可以查看遥测，按键不会启动跑图。现场标定仍在 `USER/nav_calibration.c`；完成标定、确认后再按原流程启用跑图。

新协议每帧为 48 个小端 float32，随后是 `00 00 80 7F`，合计 **196 字节**，约 **300 ms/帧**。9600、8N1 下单帧线传输约 204 ms。BLE 通知可能拆成多个片段，软件会重组，不能按通知次数计算帧数。USART1 原有 115200、10 Hz 的文本 CSV 诊断保留。

## 3. 启动 EXE 和 VOFA

1. 打开 `build/vofa-align/`，双击 `WHEELTEC-BLE-VOFA-Align.exe`。
2. 窗口先显示 CSV 路径，随后显示扫描、连接、订阅和状态变化。默认自动寻找 `WHEELTEC-IOS`，断线会自动重新扫描连接。
3. 在 VOFA 中选择 **JustFloat** 数据解析引擎和 **UDP** 设备，监听本地 `127.0.0.1:9000`。远端若必须填写，填 `127.0.0.1:9001`。启动接收。
4. 收到帧后，在 VOFA 中为曲线绑定下表列出的通道编号；有些界面从 1 开始显示，应以第一列原始前距对应通道 0 为准。
5. 正常结束时，在 EXE 窗口按 **Ctrl+C**。程序断开 BLE、结束记录线程、关闭 UDP 和 CSV。直接关窗口或断电可能丢失尚未处理的片段。

每个完整帧对应一个 UDP 数据报；UDP 源固定绑定 `127.0.0.1:9001`，不会向小车发送控制数据。BLE 订阅所需的 Notify/CCCD 配置由 bleak 完成。

## 4. 先绑定哪些曲线

| 现场要看什么 | 通道 |
| --- | --- |
| 前激光：原始 / 滤波 / 当前目标 | `0 / 4 / 8`，mm |
| 后激光：原始 / 滤波 / 当前目标 | `1 / 5 / 9`，mm |
| 左激光：原始 / 滤波 / 当前目标 | `2 / 6 / 10`，mm |
| 右激光：原始 / 滤波 / 当前目标 | `3 / 7 / 11`，mm |
| 激光是否可用 / 是否真的参与定位 / CAN 数据龄 | `12 / 13 / 14` |
| 导航状态 / 停车原因 / 当前目标点 / 已确认点 | `15 / 16 / 17 / 18` |
| 主方向误差 / 横向纠偏误差 / 航向误差 | `19 / 20 / 21`，mm / mm / 度 |
| 车体速度命令 vx / vy / wz | `22 / 23 / 24`，m/s / m/s / rad/s |
| A 轮目标 / 实际轮速 / PWM | `25 / 29 / 33` |
| B 轮目标 / 实际轮速 / PWM | `26 / 30 / 34` |
| C 轮目标 / 实际轮速 / PWM | `27 / 31 / 35` |
| D 轮目标 / 实际轮速 / PWM | `28 / 32 / 36` |
| 电池电压 / 电机控制允许 / 硬件 EN / 软件停用 | `37 / 38 / 39 / 40` |
| IMU 就绪 / CAN 新鲜 / 机构暂停已停稳 | `41 / 42 / 43` |
| 本轮起跑前距 / CAN 序号 / 遥测序号 / 单片机时间 | `44 / 45 / 46 / 47` |

先把四路原始值分别遮挡检查方向，再叠加滤波和目标。**目标为 0 表示当前未使用该方向作定位参考，不是要求车靠墙至 0 mm。** 左侧道后激光即使有合法读数，也会因照向场外而不参与定位；短垄区某一侧近壁不可用时，对应目标也可能为 0。

通道 12、13 是位掩码：前=1、后=2、左=4、右=8，按加法组合。15 表示四路，13 表示前/左/右。有效读数不一定适合作当前段定位；应同时看 13。完整解释见 `VOFA通道表.md`。

轮速反馈来自编码器，车体 22～24 是控制命令。不能把车体命令当成车辆实际地面速度；打滑时编码器也不能代表真实位移。约 3.3 Hz 遥测适合看导航和趋势，无法完整反映 100 Hz 轮速环的瞬态。

## 5. CSV 和窗口状态

默认日志在 **EXE 同目录的 `logs` 文件夹**，与启动时的工作目录无关。文件示例：`telemetry-nav48-20261118-143000-123456.csv`。每次运行创建新文件，不覆盖已有文件；重新连接仍写入该次运行的同一个文件，并增加 `connection_id`。

前两列为 `received_at`（电脑收到组成该帧的最后片段时的时间，带 `+08:00` 时区）和 `connection_id`，后面按通道表给出 48 个具名字段。每帧执行刷新和文件同步；可在运行中复制 CSV 分析。它记录的是电脑接收时间，不是精确的激光采样时间。单片机快照时间在 `mcu_time_ms`。

窗口显示累计接收字节、合法帧、丢弃字节、UDP 错误、队列丢弃和接收空闲时间。状态变化会输出导航状态、原因、点号、当前参考和关键允许位。缺少数据、协议不符、端口或日志写入错误会给出提示。

`discarded` 是无法组成合法帧、重连清空或队列溢出的字节数，不是真实无线丢包数。可结合 `telemetry_sequence` 观察间断；序号在 16777215 后回到 1，单片机重启也会重新从 1 开始。`connection_id` 用于识别重连。

JustFloat 无校验和、帧头或协议版本。当前程序使用固定帧长、帧尾和数值约束恢复同步，已测试噪声和丢失片段后的恢复，但不能保证识别所有恰好仍为合法数值的损坏。每次重连都创建新缓冲，防止跨连接拼帧。

## 6. 参数与离线自检

在 EXE 所在目录打开 PowerShell：

```powershell
# 不扫描蓝牙、不连接实车，检查 WinRT 依赖、重组、回环 UDP 和 CSV
.\WHEELTEC-BLE-VOFA-Align.exe --self-test

# 指定实际名称或地址（二选一；指定地址时优先使用地址）
.\WHEELTEC-BLE-VOFA-Align.exe --name WHEELTEC-IOS
.\WHEELTEC-BLE-VOFA-Align.exe --address AA:BB:CC:DD:EE:FF

# 修改 VOFA 监听端口和本工具源端口，VOFA 配置须同步
.\WHEELTEC-BLE-VOFA-Align.exe --port 9100 --local-port 9101

# 保存到其他可写目录
.\WHEELTEC-BLE-VOFA-Align.exe --log-dir 'D:\robot-logs'

.\WHEELTEC-BLE-VOFA-Align.exe --help
```

自检使用临时回环端口，生成带 `selftest-` 前缀的模拟数据 CSV；不是实车数据，也不要求 VOFA 正在运行。默认正常运行会独占绑定源端口，端口被占用时直接报错；目标端口是否真的有 VOFA 监听，UDP 本身不能确认。

旧固件兼容通过明确布局选择，不自动混猜：

| 参数 | 数据布局 |
| --- | --- |
| 无参数，或 `--profile nav48` | 当前激光导航 48 通道 |
| `--profile align20` | 队友提供的相机/对中 20 通道 |
| `--profile legacy48` / `legacy64` | 项目旧跑图 48 / 64 通道 |
| `--profile legacy80` / `legacy104` / `legacy112` / `legacy128` | 保留已有旧协议解析 |
| `--profile legacy8` / `legacy72` | 8 / 72 通道 JustFloat，字段用 ch00…通用名 |

未找到旧 8/72 通道的语义表，因此保留收帧能力，不给它们附会本工程字段。`--channels 48` 等同 `nav48`，旧 48 通道必须用 `--profile legacy48`；`--channels 20` 等同 `align20`。不要同时传 `--channels` 和 `--profile`。

## 7. 常见问题

| 现象 | 检查方法 |
| --- | --- |
| 找不到设备或反复连接失败 | 断开手机 APP/其他电脑，确认模块供电、蓝牙开启和设备名；必要时指定地址。模块必须支持 FFE0/FFE1 的 BLE Notify，经典蓝牙串口模块不能用此 BLE 流程 |
| 已连接但接收字节一直为 0 | 检查是否烧录本次固件、模块 UART 是否 9600/8N1、PD5 到模块 RX 和共地；只改电脑端无法让旧固件新增遥测 |
| 有字节但没有合法帧，丢弃持续增加 | 确认默认 nav48 和新固件配套；检查波特率、UART 线路、是否有其他代码向 USART2 混入文本；队友 20 通道要另选 align20 |
| RX 有合法帧，VOFA 没波形 | VOFA 选择 JustFloat、UDP 并启动接收，监听 9000，不要把它监听成 9001；检查通道绑定和被其他程序占用的目标端口 |
| 无法绑定 9001 | 关闭另一个转发器，或修改 `--local-port`，同时修改 VOFA 远端配置；不能与目标监听端口相同 |
| 无法创建 CSV、写入失败 | EXE 所在目录须可写，避开只读目录，或用 `--log-dir`；检查磁盘空间。写盘失败会结束程序，避免静默漏记 |
| 后距有效但后目标为 0 | 左侧道后激光指向场外时会主动不用它；看 reference_mask 和当前点，不把 0 当测距目标 |
| 原始测距不动，CAN 新鲜仍为 1 | CAN 新鲜只证明接收了新帧。若底板一直重发某一路旧值，此协议无法识别，需检查底板是否按无效约定上报 |
| 状态为 CAL / 原因为“需要标定” | 当前 NAV_CALIBRATED 仍为 0 或表不合法；先采集标定数据，不为看波形而绕过标定门槛 |
| 断线后偶尔少几帧 | 重连会清空半帧；模块缓存、无线吞吐和距离需实测。查看序号和接收/丢弃统计 |

## 8. 开发与验证

源码要求 Python 3.10+，电脑端使用 bleak；已构建版本使用 Windows x64 Python 3.14.3、bleak 3.1.0、PyInstaller 6.22.3。EXE 内已包含这些运行依赖，目标电脑不用安装 Python。完整依赖版本见输出目录的 `requirements-lock.txt`；实际验证环境为 Windows 11 x64，另一台电脑的蓝牙适配器/系统仍需验证。

在工程根目录执行：

```powershell
# 现有项目私有构建环境，不影响全局 Python
.\build\ble-venv\Scripts\python.exe tools\ble_vofa_bridge.py --self-test
python tools\run_ble_tests.py
python tests\run_tests.py
python tools\run_control_regression.py
.\build\ble-venv\Scripts\python.exe tools\package_ble.py
python tools\build.py --toolchain 'G:/keil5/ARM/ARMCC/bin'
```

换电脑重新构建时先用 Windows x64 Python 创建虚拟环境，再按 `tools/requirements-ble-lock.txt` 安装依赖。控制回归脚本需要 `build/ble-baseline.json` 指向本次修改前的备份，因此它属于本工程工作区验证，不是可脱离备份运行的通用仿真工具。

已完成的自动验证记录在 `OBJ/tests/ble-tests.log`、`OBJ/tests/host-tests.log`、`OBJ/tests/control-regression/result.json`、`OBJ/build-manifest.json` 和 `build/vofa-align/self-test.log`。打包脚本会复制 EXE 到单独目录，移除 PATH 中的 Python，以不同工作目录运行自检，验证依赖内置、UDP/CSV 正常和日志落在 EXE 旁。该检查不是一台完全无 Python 的干净虚拟机测试。

本次通过 2 组 C 遥测/参考选择测试、16 项电脑端自动测试、1/3/5 帧滤波下的导航测试与修改前后控制轨迹比对。ARMCC 编译 70 个单元，0 错误、0 警告。EXE 自检成功导入 11 个 WinRT 相关模块，并实际执行本机 WinRT DataWriter/DataReader 缓冲往返；没有扫描蓝牙。源码/产物一致性核验见 `OBJ/tests/ble-delivery-check.json`。

**尚需实车验证**：实际模块的名称/UUID/Notify、9600 UART 到 BLE 吞吐、长时间接收/断线重连、四向对应关系、车轮反馈和符号、VOFA 界面显示、另一台 Windows 电脑运行，以及启用现场标定后实际行驶中的实时影响。没有用主机测试替代实车验收。

实现参考：[bleak 连接/通知 API](https://bleak.readthedocs.io/en/stable/api/client.html)、[Windows 线程模型排查](https://bleak.readthedocs.io/en/latest/troubleshooting.html)、[PyInstaller 运行要求](https://pyinstaller.org/en/stable/requirements.html)。
