# C30D 底盘 IMU 航向控制固件

本工程在 C30D STM32 底盘中加入了基于车载 IMU 的航向保持控制。上位机只需发送普通速度指令；车辆直行时由底盘在 100 Hz 控制周期内完成航向估计和纠偏，不再依赖上位机侧 IMU 控制节点。

**当前烧录配置为步进电机独立测试模式**：[`USER/main.c`](USER/main.c) 中 `A4988_TEST_MODE` 默认为 `1`。此模式上电后运行两路 A4988 测试，不启动原底盘、IMU/I²C、航向控制或串口反馈任务。要恢复以下底盘功能，须将该宏改为 `0` 并重新编译、烧录。

## 已实现功能

- 使用底盘 IMU 的 Z 轴陀螺仪进行航向积分和 PD 纠偏。
- 上电前约 10 秒采集静止 IMU 样本，以平均值校准陀螺仪零偏。
- IMU 校准完成后，PA8 有源蜂鸣器短响一次。
- 仅在车辆平移且上位机未请求转向时启用航向保持。
- 停车或主动转向时复位航向控制器，避免停车后反复纠偏。
- 主动转向指令直接传递给底盘，不会被航向保持覆盖。
- 加入陀螺低通、积分死区、底盘死区补偿和输出斜率限制。
- 切换到 APP、遥控器、PS2、CAN、USART1、USART5 或自动回充模式时自动复位航向控制器。
- 新增两路 A4988 步进电机库，支持按正负脉冲数运动、加减速及停止 STEP 脉冲输出。

## 两路 A4988 步进电机

独立库为 [`HARDWARE/stepper_motor.h`](HARDWARE/stepper_motor.h) 和 [`HARDWARE/stepper_motor.c`](HARDWARE/stepper_motor.c)；底层 DIR/STEP 引脚定义见 [`HARDWARE/a4988.h`](HARDWARE/a4988.h)。左路 DIR=PB14、STEP=PB15/TIM12_CH2，右路 DIR=PC7、STEP=PC8/TIM8_CH3。两路 EN 均由外部接线控制，PC6、PC9 不参与步进驱动。TIM12 和 TIM8 已被 STEP 输出占用，不能同时用作原舵机或遥控输入。

```c
StepperMotor_Init();                              // 上电初始化一次；测试程序已调用
StepperMotor_SetARR(A4988_MOTOR_Left, 4999);      // 目标约 200 脉冲/秒
StepperMotor_MovePulses(A4988_MOTOR_Left, 800);  // DIR 高，走 800 个脉冲
StepperMotor_MovePulses(A4988_MOTOR_Left, -800); // DIR 低，走 800 个脉冲
StepperMotor_Disable(A4988_MOTOR_Left);          // 停止 STEP 脉冲并拉低 STEP
```

`StepperMotor_SetARR(motor, arr)` 可分别调整两路目标转速；1 MHz 计数时脉冲频率约为 `1000000 / (arr + 1)` Hz，允许 `arr=1999～19999`（约 500～50 脉冲/秒），默认 `4999`。运行中改值会从后续脉冲逐步生效。`StepperMotor_MovePulses()` 为阻塞式函数：正数和负数分别选择两个 DIR 电平，`0` 不运动；返回完成的脉冲周期数。每段最多用前后各 200 个脉冲加减速，默认从约 50 脉冲/秒加速到约 200 脉冲/秒，再减速；到达目标会自动停止脉冲。**停止脉冲不等于 A4988 真正失能**：外部 EN 若仍有效，电机可能保持力矩。脉冲数也不等于毫米，实际行程取决于电机整步数、细分档位和传动机构。完整接线与时序见 [`HARDWARE/README_A4988.md`](HARDWARE/README_A4988.md)。

当前独立测试程序为 [`USER/stepper_test.c`](USER/stepper_test.c)：上电后左、右电机依次正向 800 脉冲、暂停 0.5 秒、反向 800 脉冲、暂停 0.5 秒，两路测完暂停 1 秒后循环。测试前应架空车轮，并确认电源、共地、限流及两个 EN 的外部使能接线正确。

## 当前控制参数

参数定义在 [`BALANCE/heading_control.h`](BALANCE/heading_control.h)：

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `HEADING_CONTROL_KP` | 1.05 | 航向误差比例增益 |
| `HEADING_CONTROL_KI` | 0.0 | 积分关闭，避免长期累积后摆动 |
| `HEADING_CONTROL_KD` | 0.26 | 使用低通后的陀螺角速度进行阻尼 |
| `HEADING_CONTROL_MAX_ANGULAR_VELOCITY` | 0.60 rad/s | 最大自动纠偏角速度 |
| `HEADING_CONTROL_MIN_ANGULAR_VELOCITY` | 0.04 rad/s | 底盘死区补偿 |
| `HEADING_CONTROL_OUTPUT_SLEW_RATE` | 0.40 rad/s² | 纠偏输出最大变化速度 |
| `HEADING_CONTROL_GYRO_FILTER_ALPHA` | 0.25 | 陀螺一阶低通系数 |
| `HEADING_CONTROL_YAW_TOLERANCE` | 0.015 rad | 航向误差容差 |
| `HEADING_CONTROL_GYRO_TOLERANCE` | 0.025 rad/s | 静止角速度容差 |
| `HEADING_CONTROL_GYRO_INTEGRATION_DEADZONE` | 0.002 rad/s | 航向积分角速度死区 |

## 执行逻辑

1. 上电后保持车辆静止，固件采集约 10 秒 IMU 数据并计算平均零偏。
2. 校准完成后蜂鸣器短响，表示底盘可以接收控制指令。
3. 当 `|Move_X| > 0.02 m/s` 或 `|Move_Y| > 0.02 m/s`，并且 `|Move_Z| < 0.01 rad/s` 时，锁定当前航向并启用自动纠偏。
4. 上位机请求转向时，固件复位航向状态并直接执行 `Move_Z`。
5. 车辆停止平移时，固件立即复位航向状态，不在原地继续来回纠偏。
6. 再次开始直行时，以当时的实际方向作为新的目标航向。

主要实现文件：

- [`BALANCE/heading_control.c`](BALANCE/heading_control.c)：航向估计、PID、滤波、死区与斜率限制。
- [`BALANCE/heading_control.h`](BALANCE/heading_control.h)：控制参数。
- [`BALANCE/balance.c`](BALANCE/balance.c)：控制模式判断、启停条件和电机指令接入。
- [`BALANCE/imu_task.c`](BALANCE/imu_task.c)：启动阶段 IMU 平均零偏校准。
- [`BALANCE/README_heading_control.md`](BALANCE/README_heading_control.md)：航向控制专项说明和首次实车数据。

## 串口协议

当前实车识别为 CH9102 `COM4`，波特率为 `115200`。Windows 端口号可能随电脑和 USB 接口变化，应以设备管理器中的 `USB-Enhanced-SERIAL CH9102` 为准。

### 上位机控制帧

控制帧长度为 11 字节：

| 字节 | 内容 |
| ---: | --- |
| 0 | 帧头 `0x7B` |
| 1 | 普通速度控制为 `0x00` |
| 2 | 保留，填 `0x00` |
| 3–4 | X 轴速度，带符号大端整数，单位为 0.001 m/s |
| 5–6 | Y 轴速度，带符号大端整数，单位为 0.001 m/s |
| 7–8 | Z 轴角速度，带符号大端整数，单位为 0.001 rad/s |
| 9 | 字节 0–8 的异或校验 |
| 10 | 帧尾 `0x7D` |

### 底盘反馈帧

反馈帧长度为 24 字节，包含软件失能标志、编码器解算速度、三轴加速度、三轴陀螺仪和电池电压。字节 22 为字节 0–21 的异或校验，字节 23 为帧尾 `0x7D`。

## 编译与烧录

Keil 工程：[`USER/WHEELTEC.uvprojx`](USER/WHEELTEC.uvprojx)

当前工程已经使用 ARMCC 5.06 编译通过：

```text
0 Error(s), 0 Warning(s)
```

生成的烧录文件：[`OBJ/WHEELTEC.hex`](OBJ/WHEELTEC.hex)。当前 HEX 对应上面的步进电机测试模式；切换 `A4988_TEST_MODE` 后需重新编译，不能只改源码就直接烧录旧 HEX。

烧录后如果使用串口 ISP，请将 `BOOT0` 或下载开关拨回 `0/RUN`，退出烧录软件，再断电重启。若停留在下载模式，CH9102 串口虽然存在，但不会输出底盘反馈帧。

## 上电检查

以下步骤仅适用于将 `A4988_TEST_MODE` 设为 `0` 并重新烧录后的正常底盘模式。默认步进测试模式不会蜂鸣提示 IMU 校准，也不会输出底盘串口反馈帧。

1. 将车辆放平并保持静止。
2. 上电后等待约 10 秒，不要移动或碰撞车辆。
3. 听到一次短蜂鸣，表示 IMU 零偏校准完成。
4. 检查串口能否持续收到帧头 `0x7B`、帧尾 `0x7D` 且异或校验正确的 24 字节反馈帧。
5. 未听到蜂鸣或串口无数据时，先检查 `BOOT0/RUN` 状态和供电，不要发送运动指令。

## 实车调参记录

首次串口测试中，旧参数在停车阶段出现约 `-0.037～+0.039 rad/s` 的正反角速度。原因不是积分项，而是 `0.08 rad/s` 固定死区补偿形成开关式纠偏，同时停车后航向环仍然工作。

本次修改将死区补偿降为 `0.04 rad/s`，提高角速度阻尼，加入低通和输出斜率限制，并修正停车/主动转向逻辑。新固件已经完成编译、烧录、启动蜂鸣和静态串口反馈检查；低速动态复测应在空旷场地完成后再继续微调参数。

## 安全注意事项

- 首次动态测试必须在空旷平地进行，并安排人员随时断电。
- 每次测试结束都应连续发送零速度帧，不能只依赖上位机进程正常退出。
- 修改 PID 后先使用低速、短时间指令，确认方向和停止行为正确后再提高速度。
- 调参期间不要同时启用多个控制来源。

