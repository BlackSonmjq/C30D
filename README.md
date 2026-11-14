# C30D 视觉与梳果任务联调

基于STM32F407和FreeRTOS。当前执行机构为四只ZP10S舵机、两路A4988步进和四路VL53L0X，视觉设备为MaixCAM Pro。

**旧GPIO推杆持续输出5秒的方案已弃用，梳果全部使用同编号舵机。** 历史pushrod.c/.h及开机拉低PC0～PC3的初始化仍保留，但不参与任务；不再按旧推杆方案接线或操作。结果码名MANGO_PUSHROD_FAILED仅为兼容旧编号保留，当前表示舵机失败。

源码：[固件目录](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/)。

## 已验证状态与范围

- 相机USB Host接底盘串口3、OLED视觉显示已由用户验证正常。
- 四只舵机均在115200回复，1/2号模式2，3/4号模式1，固件版本ZL-ZServo_AD_CBM V2.1.16STG。
- 用户调整2号接线后反馈动作正常；具体改线细节未记录，不据此推导其他接线也可靠。
- 9600降速未验证成功：PBD0后普通ACK不能代替新速率回读，现场四只仍在115200回复。当前按115200使用。
- 软件有32个梳果协调仿真场景，以及相机、串口、舵机驱动、速率探测和版本查询测试；不代表整车全流程实机验收完成。

**视觉目前只接收、解析和显示，不会自动触发梳果。** 完整任务由上位机K1～K4启动；视觉到任务编号的映射、目标时效和底盘停车协调仍需整车上层接入。

## 接线与资源

| 功能 | MCU资源 | 用途 |
| --- | --- | --- |
| 上位机 | USART1，PA9 TX / PA10 RX | 115200、8N1，ASCII命令和日志 |
| 相机 | USART3，PD8 TX / PD9 RX | 115200、8N1，六字节二进制接收 |
| 舵机 | UART4，PC10 | 单线半双工、开漏复用；启动115200，支持按ID选择已确认速率 |
| 左步进 | PB14 DIR / PB15 STEP | K1、K2 |
| 右步进 | PC7 DIR / PC8 STEP | K3、K4 |
| 激光I²C | PA8 SCL / PC9 SDA | 共用地址0x29，互斥选通 |
| XSHUT | PA2 / PA3 / PA6 / PC11 | 激光1～4，4号已迁至PC11 |
| CAN1 | PD1 TX / PD0 RX | 经收发器连接CANH/CANL |

舵机独立供电并与主控共地，PC10连接信号总线；电平和上拉需与主控兼容。ID与模式已经配置，启动不自动改写。不要使用恢复出厂命令破坏已有配置。

## 视觉通信与OLED

实际链路：MaixCAM Pro USB Host → 底盘“串口3”USB → 板载CH9102F → USART3 PD8/PD9。相机应用为/maixapp/apps/mgou/main.py，曾确认USB串口设备为/dev/ttyACM0，速率115200。UART0是相机板载UART选择，不能直接当作USB设备名。

相机终端可用lsusb、ls -l /dev/ttyACM* /dev/ttyUSB*、dmesg | tail -40检查枚举。设备应为字符设备；名称相似的普通文件不代表USB串口。没有外设枚举时先查线缆、供电和Host状态，不让两个应用同时占用串口。

每帧6字节，发送二进制值而非ASCII十六进制文本：

```text
63 B DX G REGION 66
```

| 字段 | 含义 |
| --- | --- |
| 63、66 | 固定帧头0x63、帧尾0x66 |
| B | 蓝杆标志，0或1 |
| DX | int8有符号偏移，-128～127；负值表示杆在图像中心左侧 |
| G | 绿色目标标志，0或1 |
| REGION | B与G均为1时取1～4，否则为0 |

B=0时DX必须为0。REGION相对杆的横向中心和顶部参考位置划分：1左上、2左下、3右下、4右上，不是按图像中心划分。示例63 01 EC 01 03 66表示B=1、DX=-20、G=1、REGION=3。协议没有序号、校验和或ACK，滑动重同步不能保证每帧无误码。

USART3专用于视觉，USART1到USART3的原始转发关闭，旧11字节串口距离格式不是当前相机输入。

OLED按10Hz刷新，默认页索引3。长按用户键循环切换：0底盘、1回充、2本地激光、3梳果任务、4相机。任务页残留的4/4文字不代表只有四页。

相机页显示B、DX、G、REGION及F有效帧数、R字节数、E无效候选帧、U串口错误，计数按显示宽度取模。LINK OK表示约1秒内观察到有效帧；WAIT RX表示上电无接收；STALE表示近期无数据；BAD DATA表示有数据却无近期有效帧。过期目标隐藏，显示状态不能代替整车控制层时效判断。

## 上位机命令

USART1使用115200、8N1、文本发送，区分大小写，可追加CR/LF。初次联调关闭循环发送，一条结束后再发下一条。

| 命令 | 功能 |
| --- | --- |
| 1～4 | 仅选对应激光，不启动梳果 |
| K1～K4 | 完整梳果任务 |
| S1～S4 | 单只舵机回零→约180°→回零 |
| S0 | 四只依次回零，是运动命令而非急停 |
| M1、M2 | 显式设置1/2号模式2并回读 |
| M3、M4 | 显式设置3/4号模式1并回读 |
| B? | 只读扫描9600/115200，记录各ID有效速率 |
| B0 | 尝试PBD0降至9600，失败重查115200并保留有效速率 |
| V0 | 只读查询全部版本 |
| V1～V4 | 只读查询指定舵机版本 |

自动任务与手动舵机/诊断互斥，忙时不排队。上位机没有任意舵机原始命令透明转发，裸PBD0不是上位机命令。B/V不会启动梳果。

B?输出mask9600=0x00、mask115200=0x0F表示四只仍在115200。各ID速率缓存在主控RAM；混合速率设备重启后，K任务前先B?扫描。S也会尝试两种速率。tx=0只表示主控发送完成，不表示舵机执行或改速成功。

默认SYSTEM/usart/usart.h的USART1_SERVO_LOG_ONLY=1，只输出SERVO/servo前缀；历史APP、激光周期和协调任务mango日志被过滤。**K任务进度与结果主要看OLED任务页。** 置0可恢复其他日志，也可能恢复历史APP刷屏。

## 梳果流程与参数

| 命令 | 激光XSHUT | 步进 | 舵机ID | 严格小于此距离才动作 |
| --- | --- | --- | --- | --- |
| K1 | PA2 | 左 | 1 | 108 mm |
| K2 | PA3 | 左 | 2 | 60 mm |
| K3 | PA6 | 右 | 3 | 45 mm |
| K4 | PC11 | 右 | 4 | 35 mm |

```text
接收K编号 → 启用同编号激光 → 等有效距离
→ 对应步进靠近 → 进入距离阈值，停止并保存前进脉冲
→ 同编号舵机回零 → 约180° → 回零并确认
→ 步进反向等脉冲返程 → 关闭激光 → 释放忙状态
```

初始已低于阈值则不前进，直接舵机动作，返程脉冲为0。等于阈值时继续靠近。四只激光共用默认地址，仅一路XSHUT为高。

以实际代码常量为准：左DIR低前进、高返程；右DIR高前进、低返程。ARR均为199，计数频率1MHz，名义巡航5000脉冲/秒，保留加减速，实际受中断开销影响。

首次测距等候上限5秒；距离500ms未更新或通道不符视为无效。前进达到26000个STEP脉冲优先跳过舵机并返程，即使同轮已进入阈值。前进/返程单段上限420秒，返程不依赖测距。

返程按指令脉冲计数，无原点开关或编码器闭环；机械失步、打滑不能由等脉冲返回排除。返程启动失败、提前停止或计数不符会保持FAULT / LOCKED，需检查原点。**梳果忙标志不会自动使底盘停车**，整车上层需协调。

### 舵机动作

自动K任务沿用已配置的270°模式：1/2号模式2，3/4号模式1，不写模式。零位P0500，约180°为P1833。手动S读取模式，若为180°模式3/4则使用P2500。

所有动作时间500ms、到位容差±100位置单位，需要两次有效位置确认；漏包保留计数，未到位反馈清零。阶段超时3秒，停滞目标在原截止时间内最多额外发送两次；一次未到位反馈后持续漏包也可约1秒后重发。手动回零还可再尝试一轮，500ms不是整轮耗时。K2已取消固定等待1.5秒、不读位置的旧特例。

按用户要求，额外舵机故障锁存与行程保护已取消；普通超时和有限重发仍保留。去程失败仍尝试回零，最终回零不代表去程成功。

OLED任务阶段包括WAIT RANGE、APPROACH、SERVO 180->0、RETURNING、LASER OFF、DONE / READY、ABORT / READY、FAULT / LOCKED。P为当前阶段计数，OUT为保存的前进计数，RES为结果。

| 结果码 | 含义 |
| --- | --- |
| 0 | 正常完成 |
| 1 | 首次测距超时 |
| 2 | 前进中测距失效 |
| 3 | 前进脉冲上限或提前停止 |
| 4 | 前进启动失败 |
| 5 | 舵机请求、动作或等待失败，枚举旧名MANGO_PUSHROD_FAILED |
| 6 | 前进运动超时 |
| 7 | 返程失败 |

## 整车联调顺序

1. 检查供电、共地和接线，先B?确认速率、V0确认版本，不改既有模式。
2. 单次S1～S4验证转角及返回。2号已通过改线恢复，后续优先结合线路与日志定位，不直接归因于任务阻塞。
3. 用裸1～4逐路验证激光和有效距离。
4. 在OLED任务页逐个运行K1～K4，核对左右方向、距离阈值、舵机编号、往返计数和RES。
5. 相机单独检查LINK OK、DX正负、REGION和断流显示，再与K任务并行。
6. 整车上层另行实现视觉选择→任务入口、目标时效、底盘停车、忙碌处理和结果反馈。当前没有该自动闭环。

## 源码入口

以下路径相对源码目录。

| 文件 | 用途 |
| --- | --- |
| HARDWARE/usartx.c | 上位机命令、手动S/B/V流程、视觉接收 |
| HARDWARE/camera_protocol.h | 六字节解析及视觉快照 |
| HARDWARE/servo.c、servo.h | UART4 DMA/TC中断半双工、各ID速率和动作常量 |
| BALANCE/servo_task.c、servo_task.h | K任务的反馈动作及超时 |
| BALANCE/stepper_task.c、stepper_task.h | 梳果协调、方向、距离阈值、脉冲与时间上限 |
| BALANCE/distance_task.c、distance_task.h | 激光选通、测距和500ms时效 |
| HARDWARE/stepper_motor.c、stepper_motor.h | 步进定时器及脉冲计数 |
| BALANCE/show.c、show.h | OLED页面 |
| BALANCE/heading_control.c、heading_control.h | 保留的底盘IMU航向控制 |
| HARDWARE/can_test.c、can_test.h | S21C四向距离与在线快照 |

ISR接收/提交请求，各任务独占对应事务。应用读取快照，避免其他任务直接抢占激光I²C或舵机总线。

S21C距离通过CAN标准数据帧0x301、DLC=8接收，四个uint16毫米值高字节在前，顺序左、前、右、后；500ms未更新标记离线。它与梳果本地VL53L0X数据分开，没有自动替换K任务测距源。

## 编译、烧录与验证

在D:/Keil_Project/C30D_EIDE执行python tools/build_normal.py，产物build/FreeRTOS/WHEELTEC.hex，不再另建output包。脚本引用本仓库源码并适配servo_task。Keil工程在源码目录USER/WHEELTEC.uvprojx。

当前为普通有线烧录：Flash起址0x08000000、向量偏移0。FlyMCU若自动复位握手失败，曾验证先读芯片并按RST进入Bootloader，再选“不使用RTS和DTR”开始编程；连接失败不等于HEX损坏。

仓库根目录按顺序运行，前两个测试共用构建目录，不能并行：

```powershell
uv run --with unicorn --with pyelftools python tests/mango/run_tests.py
uv run --with unicorn --with pyelftools python tests/mango/run_tests.py tests/servo/test_servo.c
uv run --with unicorn --with pyelftools python tests/servo_debug/run_tests.py
uv run --with unicorn --with pyelftools python tests/serial/test_task_commands.py
uv run --with unicorn --with pyelftools python tests/serial/test_servo_debug.py
uv run --with unicorn --with pyelftools python tests/serial/test_servo_transport.py
uv run --with unicorn --with pyelftools python tests/camera/run_tests.py
```

固件串口仿真先构建AXF再运行。仿真不能验证实际线缆、供电、机械失步或整车原点。补充记录：[舵机](tests/servo/README.md)、[梳果测试](tests/mango/README.md)、[串口测试](tests/serial/README.md)。
