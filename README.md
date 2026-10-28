# C30D 底盘与外设固件

本项目基于 STM32F407 和 FreeRTOS，包含底盘控制、IMU 航向保持、两路 A4988 步进电机、四路推杆控制和 VL53L0X 激光测距。本文统一维护工程和各零部件说明。

源码目录：[R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/)。

## 目录

- [双路激光与串口命令](#双路激光与串口命令)
- [函数速查](#函数速查)
- [当前启动行为](#当前启动行为)
- [引脚与资源分配](#引脚与资源分配)
- [两路 A4988 步进电机](#两路-a4988-步进电机)
- [推杆控制](#推杆控制)
- [VL53L0X 激光测距](#vl53l0x-激光测距)
- [IMU 航向控制](#imu-航向控制)
- [串口协议](#串口协议)
- [编译与上电检查](#编译与上电检查)

## 双路激光与串口命令

当前串口 1 使用激光选择命令，替换此前步进启动、返回和推杆启动命令。串口助手设置 115200、8N1，使用文本发送，可附带 CRLF。

| 输入 | PA2 / 激光 1 XSHUT | PA3 / 激光 2 XSHUT | OLED 标题 |
| --- | --- | --- | --- |
| 上电，未发送命令 | 低，关闭 | 低，关闭 | Lasers OFF |
| 文本 1（ASCII 0x31） | 高，使能 | 低，关闭 | Laser 1 PA2 |
| 文本 2（ASCII 0x32） | 低，关闭 | 高，使能 | Laser 2 PA3 |

两路共用 PA8=SCL、PC9=SDA 和默认地址 0x29，XSHUT 必须分别接 PA2、PA3。同一时刻只使能一路；切换时先关闭两路、再启用选中一路，等待启动并重新初始化驱动。切换后清除旧距离，仅发布当前传感器的新有效数据。初始化失败后重试，重复选择当前通道不会反复复位传感器。OLED 同时保留距离、Steps 和基本测距状态。

DistanceTask_InitShutdownPins() 在板级初始化早期将 PA2/PA3 配置为低电平推挽输出；DistanceTask_SelectLaserFromISR(laser_id) 只锁存 1/2 选择，测距任务独占 I²C 并完成切换；DistanceTask_Data.laser_id 为实际启用通道（0=全关、1=PA2、2=PA3）。

下文步进和推杆接口保留供应用调用；当前串口字符 1/2 不再触发步进，字符 3 不再启动推杆，因此上电步进与推杆保持等待。历史定位算法参数仍为距离 130 mm、70000 脉冲上限。

## 函数速查

### 步进电机

声明：[stepper_motor.h](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/stepper_motor.h)；实现：[stepper_motor.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/stepper_motor.c)。

motor 取 A4988_MOTOR_Left（1）或 A4988_MOTOR_Right（2）。

| 函数 | 参数与用途 | 返回值 |
| --- | --- | --- |
| StepperMotor_Init() | 初始化两路 GPIO 和 STEP 定时器；运动前调用 | void |
| StepperMotor_SetSpeed(motor, arr) | 设置目标周期；arr 为非零 16 位 ARR，目标频率约 1000000/(arr+1) Hz | u8：成功 1，电机编号无效或 arr=0 时为 0 |
| StepperMotor_MovePulses(motor, signed_pulses) | 阻塞运动；正数 DIR 高，负数 DIR 低，0 不运动；完成后停止 STEP | u32：实际完成的脉冲周期数，无效输入为 0 |
| StepperMotor_Disable(motor) | 停止 STEP 并保持低电平；外部 EN 不变，可能仍有保持力矩 | void |
| StepperMotor_StartContinuous(motor, forward) | 非阻塞连续运动并清零计数；forward=1 DIR 高，0 DIR 低 | u8：成功 1，无效电机编号 0 |
| StepperMotor_GetCompletedPulses(motor) | 读取本次运动已计数的脉冲；先 Disable 再读最终值 | u32：STEP 脉冲数，无效编号 0 |
| StepperMotor_UpdateIRQ(motor) | 内部定时器中断分发，处理计数和下一脉冲；应用层无需调用 | void |

MovePulses 必须在开中断、非临界区的任务或裸机主循环调用；禁止在 ISR 中调用。同一电机不能并发运动，其他任务可用 Disable 停止运动。UpdateIRQ 中不得调用 FreeRTOS API。

底层声明：[a4988.h](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/a4988.h)；实现：[a4988.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/a4988.c)。

| 函数 | 参数与用途 | 返回值 |
| --- | --- | --- |
| A4988_Init() | 初始化两路 DIR/STEP 为低电平 GPIO 输出 | void |
| A4988_SetDir(motor, high) | 设置 DIR，high=0 为低电平，非零为高电平 | void |
| A4988_Step(motor) | GPIO 阻塞输出一次 STEP 脉冲；仅在对应定时器输出已停止时使用 | void |

### 推杆

声明：[pushrod.h](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/pushrod.h)；实现：[pushrod.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/pushrod.c)。

| 函数 | 参数与用途 | 返回值 |
| --- | --- | --- |
| Pushrod_Init() | 初始化 PC0～PC3，全部输出低电平 | void |
| Pushrod_Start(id) | id=1～4，对应输出置高 | Pushrod_Result |
| Pushrod_Stop(id) | id=1～4，对应输出置低 | Pushrod_Result |

Pushrod_Result：PUSHROD_OK 表示成功；PUSHROD_INVALID_ARGUMENT 表示编号无效；PUSHROD_NOT_INITIALIZED 表示尚未初始化。

### 激光测距驱动

声明：[vl53l0x.h](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/vl53l0x.h)；实现：[vl53l0x.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/vl53l0x.c)。

| 函数 | 参数与用途 | 返回值 |
| --- | --- | --- |
| VL53L0X_Init() | 初始化传感器、SPAD、默认配置和参考校准；先完成 delay_init(168)，确保 XSHUT 为高 | VL53L0X_Status |
| VL53L0X_ReadSingle(result) | 执行一次测距；result 指向 VL53L0X_Result | VL53L0X_Status |
| VL53L0X_StartContinuous() | 启动连续测距 | VL53L0X_Status |
| VL53L0X_ReadContinuous(result) | 查询就绪状态并读取结果，不等待新测量完成 | VL53L0X_Status；未就绪为 VL53L0X_NOT_READY |
| VL53L0X_StopContinuous() | 停止连续测距 | VL53L0X_Status |

结果字段：distance_mm 为毫米距离；raw_range_status 为设备原始状态。只有返回 VL53L0X_OK 时才使用距离；未就绪、通信或参数错误不会更新结果，不能重复使用旧数据作为新测量。

| VL53L0X_Status | 含义 |
| --- | --- |
| VL53L0X_OK | 成功 |
| VL53L0X_NOT_READY | 暂无新测量 |
| VL53L0X_ERROR_I2C | I²C 通信错误 |
| VL53L0X_ERROR_TIMEOUT | 等待超时 |
| VL53L0X_ERROR_ID | 器件识别失败 |
| VL53L0X_ERROR_ARGUMENT | 参数无效 |
| VL53L0X_ERROR_STATE | 调用状态不正确 |
| VL53L0X_ERROR_RANGE | 本次测距无效 |
| VL53L0X_ERROR_CONFIG | 配置校验失败 |

驱动由一个任务独占，不得并发或从 ISR 调用。当前默认 distance_task 已使用驱动；普通应用应通过下方快照接口读取，独立调用底层前须停止默认测距任务。

### 测距快照与显示

声明：[distance_task.h](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/distance_task.h)；实现：[distance_task.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/distance_task.c)。

| 函数 | 参数与用途 | 返回值 |
| --- | --- | --- |
| distance_task(pvParameters) | FreeRTOS 任务入口，默认启动传感器、采样和失败重试；由启动任务创建 | void，任务循环运行 |
| DistanceTask_GetData(data) | data 指向 DistanceTask_Data，读取线程安全快照；有效数据达到 500 ms 未更新时标记超时 | void，通过 data 输出 |
| DistanceTask_DrawOLED() | 绘制距离与状态到帧缓冲；由 show_task 统一调用和刷新 | void |

DistanceTask_Data 包含 distance_mm、valid、status、update_tick、laser_id。必须先检查 valid；无效或过期时距离为 0，不能当作有效读数。

### IMU 航向控制

声明：[heading_control.h](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/heading_control.h)；实现：[heading_control.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/heading_control.c)。

| 函数 | 参数与用途 | 返回值 |
| --- | --- | --- |
| HeadingControl_Reset() | 清空航向积分、目标和控制器状态 | void |
| HeadingControl_Update(gyro_z_raw) | 输入已扣除启动零偏的 Z 轴陀螺原始 short 值；由 Balance_task 按 100 Hz 调用，勿传入 rad/s 浮点值 | float：角速度修正量，单位 rad/s |

### 步进测距运动状态

声明：[stepper_task.h](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/stepper_task.h)。

| 函数 | 参数与用途 | 返回值 |
| --- | --- | --- |
| StepperTask_GetData(data) | 任务上下文读取 StepperTask_Data 快照，包含 pulses 和 state | void，通过 data 输出 |

### 其他任务入口

这些函数由 FreeRTOS 调度，不作为一次性普通函数调用。

| 函数 | 默认行为 | 源码 |
| --- | --- | --- |
| stepper_Task(void *pvParameters) | 左路前进至距离 ≤130 mm；达到 70000 脉冲时自动回退 | [stepper_task.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/stepper_task.c) |
| pushrod_task(void *pvParameters) | 等待应用请求；当前串口未绑定推杆命令 | [pushrod_task.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/pushrod_task.c) |

## 当前启动行为

[USER/main.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/USER/main.c) 完成 systemInit() 后启动 FreeRTOS，创建底盘控制或检查任务、对应硬件版本的 IMU 任务、显示、LED、PS2、通信、推杆、步进和测距任务。当前源码没有 A4988_TEST_MODE 独立测试开关。

- 步进任务保留历史定位算法，当前串口不触发该任务，电机保持停止。
- 推杆任务上电保持 PC0～PC3 低电平，当前串口未绑定推杆启动命令。
- 上电 PA2/PA3 全部拉低，收到文本 1/2 后测距任务才启用对应激光并开始测距。
- IMU 上电约 10 秒采集静止样本校准；蜂鸣器已禁用。
- OLED 测距页新增 Steps 累计 STEP 脉冲数，运动时刷新，停止后保留最终计数（包含细分）。OLED 默认显示测距页，长按 PE0 用户键约 0.6 秒循环切换三页。

上电前须确认步进电机与推杆的命令动作符合预期；如需改变启动行为，修改对应任务并重新编译烧录。

## 引脚与资源分配

| 部件 | 引脚 / 资源 | 说明 |
| --- | --- | --- |
| A4988 Left | DIR=PB14，STEP=PB15/TIM12_CH2 | EN 外部控制 |
| A4988 Right | DIR=PC7，STEP=PC8/TIM8_CH3 | EN 外部控制 |
| 推杆 1～4 | PC0～PC3 | 高电平启动，低电平停止 |
| 两路 VL53L0X | SCL=PA8，SDA=PC9，激光 1 XSHUT=PA2，激光 2 XSHUT=PA3 | 独立软件 I²C，互斥使能 |
| 原 IMU 总线 | PB10/PB11 | 与测距总线分离 |
| 蜂鸣器 | 原 PA8 | 已禁用，须断开原蜂鸣器及驱动电路 |

TIM8/TIM12 已用于步进，不能同时用于原舵机或遥控输入；PC9 已用于测距，PC6 不参与步进驱动。


## 两路 A4988 步进电机

独立库为 [`HARDWARE/stepper_motor.h`](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/stepper_motor.h) 和 [`HARDWARE/stepper_motor.c`](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/stepper_motor.c)；底层 DIR/STEP 引脚定义见 [`HARDWARE/a4988.h`](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/a4988.h)。左路 DIR=PB14、STEP=PB15/TIM12_CH2，右路 DIR=PC7、STEP=PC8/TIM8_CH3。两路 EN 均由外部接线控制，PC6、PC9 不参与步进驱动。TIM12 和 TIM8 已被 STEP 输出占用，不能同时用作原舵机或遥控输入。

```c
StepperMotor_Init();                              // 上电初始化一次；测试程序已调用
StepperMotor_SetSpeed(A4988_MOTOR_Left, 4999);    // 目标约 200 脉冲/秒（参数仍为 ARR）
StepperMotor_MovePulses(A4988_MOTOR_Left, 800);  // DIR 高，走 800 个脉冲
StepperMotor_MovePulses(A4988_MOTOR_Left, -800); // DIR 低，走 800 个脉冲
StepperMotor_Disable(A4988_MOTOR_Left);          // 停止 STEP 脉冲并拉低 STEP
```

`StepperMotor_SetSpeed(motor, arr)` 可分别调整两路目标转速；参数仍是 ARR 值，1 MHz 计数时目标脉冲频率约为 `1000000 / (arr + 1)` Hz，默认 `4999`。已移除软件最高、最低速度限制，参数仍受 16 位 ARR 范围约束；有效 STEP 脉冲需要非零 ARR。`arr=999` 对应约 1 kHz，`arr=99` 对应约 10 kHz，实际频率还受单脉冲的软件重启时间影响。运行中改值会从后续脉冲逐步生效。`StepperMotor_MovePulses()` 为阻塞式函数：正数和负数分别选择两个 DIR 电平，`0` 不运动；返回完成的脉冲周期数。每段最多用前后各 200 个脉冲加减速，默认从约 50 脉冲/秒加速到约 200 脉冲/秒，再减速；目标速度低于 50 脉冲/秒时直接以目标周期启动；到达目标会自动停止脉冲。**停止脉冲不等于 A4988 真正失能**：外部 EN 若仍有效，电机可能保持力矩。脉冲数也不等于毫米，实际行程取决于电机整步数、细分档位和传动机构。

当前任务见 [BALANCE/stepper_task.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/stepper_task.c)：历史逻辑等待应用调用 StepperTask_RequestStartFromISR() 后等待有效测距（当前串口 1/2 已改作激光选择），距离大于 130 mm 时让左路连续前进，距离小于等于 130 mm 时停止。左路负脉冲方向为前进，DIR 低（STEPPER_FORWARD=0），ARR=99（目标约 10000 脉冲/秒），每 10 ms 检查一次测距快照；最多前进 70000 个 STEP 脉冲，达到上限后自动停止并反向返回已完成的实际脉冲数。传感器本身约 100 ms 测量预算加 50 ms 轮询间隔，实际停止位置会受采样延迟和机械惯性影响，不能保证恰好 130 mm。

初始距离已 ≤130 mm 时不运动；运动中测距无效或过期立即停止。达到目标距离后保持停止，等待应用请求返回后按实际前进脉冲数返回（当前文本 2 仅切换激光）；达到 70000 脉冲上限时自动返回，不等待字符 2。返回完成后进入 RETURNED，返回过程中异常停止则进入 RETURN_STOP；运动中及完成后重复发送 1 不重新启动。串口助手不要勾选十六进制发送（十六进制方式须发送 31/32，不能发 01/02），可附带 CRLF。到达目标后，通信任务在 USART1（115200，PA9=TX、PA10=RX）发送一次 ASCII 行：stepper position ready <累计脉冲数>，末尾为 CRLF；例如 stepper position ready 12345。自动回退完成或故障停车不发送 ready。初始距离已达标时发送计数 0。该通知由现有通信任务发送。USART1_BINARY_FEEDBACK_ENABLED 默认设为 0，串口 1 不再周期发送二进制底盘反馈，避免文本显示乱码；串口 3 的底盘反馈继续发送。需要恢复串口 1 的二进制反馈时，可在 HARDWARE/usartx.c 将该宏设为 1。方向可通过 STEPPER_FORWARD 修改，阈值通过 STEPPER_TARGET_MM 修改，最大脉冲数通过 STEPPER_MAX_PULSES 修改。其他任务不得同时控制该电机。

~~~c
#include "stepper_task.h"

StepperTask_Data data;
StepperTask_GetData(&data);
/* data.pulses：累计 STEP 脉冲，包含细分，不是编码器实测步数。
   data.state：WAITING_DISTANCE / MOVING / TARGET_REACHED / SENSOR_STOP /
               LIMIT_REACHED / RETURNING / RETURNED / RETURN_STOP，
   对应枚举名称均带 STEPPER_ 前缀。停止后保留最终计数。 */
~~~

如电机每圈 200 整步、16 细分，则整步等效数为 pulses/16，圈数为 pulses/3200；行程还需结合传动机构换算。u32 计数达到 2^32 会回绕。旧的 USER/stepper_test.c、.h 和独立测试模式已移除。

### A4988 接口定义与接线

DIR 使用 3.3 V GPIO 推挽输出；STEP 空闲时为低电平，需要运动时由定时器输出脉冲。系统启动时会优先调用 `A4988_Init()`，先将 DIR/STEP 配置为低电平推挽输出，避免 STEP 引脚在其他外设初始化期间悬空；随后由 `StepperMotor_Init()` 完成定时器初始化。两路 STEP 不会自动产生脉冲。

| 驱动器 | DIR | STEP | 定时器通道 | EN |
| --- | --- | --- | --- | --- |
| Left | PB14 | PB15 | TIM12_CH2 | 外部接线控制 |
| Right | PC7 | PC8 | TIM8_CH3 | 外部接线控制 |

初始化后两路 DIR 和 STEP 均为低电平。两路 EN 不由 STM32 控制；A4988 的 EN 为低电平有效，必须通过外部接线提供确定电平，不能悬空。PC6、PC9 不参与步进驱动。TIM8 和 TIM12 已被 STEP 输出占用，不能同时用于舵机或遥控输入。

底层接口定义在 [`HARDWARE/a4988.h`](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/a4988.h)，步进电机接口定义在 [`HARDWARE/stepper_motor.h`](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/stepper_motor.h)：

```c
A4988_SetDir(A4988_MOTOR_Left, 1);                 // DIR 输出高电平，正反向取决于接线
A4988_Step(A4988_MOTOR_Left);                      // GPIO 阻塞式输出一个 STEP 上升沿
StepperMotor_SetSpeed(A4988_MOTOR_Left, 4999);    // 目标约 200 脉冲/秒
StepperMotor_MovePulses(A4988_MOTOR_Left, 800);   // DIR 高，输出 800 个脉冲
StepperMotor_MovePulses(A4988_MOTOR_Left, -800);  // DIR 低，输出 800 个脉冲
StepperMotor_Disable(A4988_MOTOR_Left);           // 停止 STEP，不切断保持电流
```

`A4988_Step()` 只能在对应电机的 PWM 停止时使用。`StepperMotor_MovePulses()` 使用定时器单脉冲模式，CCR 为当前周期的一半，约为 50% 占空比；运动完成后自动停止并将 STEP 保持低电平。该函数必须从开中断、非临界区的任务或裸机主循环调用，禁止在 ISR 或关中断状态下调用。每个电机同一时刻只能有一个运动调用；另一任务可以调用 `StepperMotor_Disable()` 停止当前运动。需要两路同步运动时，应用层还需自行协调。

### A4988 电源与控制引脚

两块 A4988 的 VDD 接 3.3 V，GND 与 STM32 共地，VMOT 接适合电机的独立电源，并按驱动板要求设置限流和电源去耦。RST、SLP、MS1～MS3 按所用 A4988 模块和细分需求连接；本接口没有占用这些引脚。细分档位由 MS1～MS3 的硬件接线决定，脉冲数不直接等于毫米，实际行程还取决于电机步距角和传动机构。

### FreeRTOS 步进任务与脉冲时序

当前 `main.c` 创建 `stepper_Task`。任务调用运动接口时，TIM12/TIM8 更新中断负责脉冲计数、加减速和启动下一脉冲，任务每次休眠 1 tick 等待运动结束。单脉冲模式可避免中断延迟时产生未计数的连续脉冲，但中断重启仍会产生少量软件开销；长时间全局关中断或更高、同优先级中断可能拉长脉冲间隔。

两个步进更新中断的抢占优先级为 4，高于 FreeRTOS 系统调用阈值 5；中断中不得调用 FreeRTOS API。`TIM8_BRK_TIM12` 和 `TIM8_UP_TIM13` 为共享向量，不能直接再启用其中的 TIM8 Break 或 TIM13 等其他中断源。

烧录后建议先架空或空载验证：使用逻辑分析仪检查 PB15/PC8 的脉冲数量、加减速段、匀速段间隔和停止后的低电平时间，再进行带载测试。测试前确认 EN、电源、共地、限流和驱动器细分设置正确。



## 推杆控制

驱动见 [HARDWARE/pushrod.h](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/pushrod.h) 和 [HARDWARE/pushrod.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/pushrod.c)，任务见 [BALANCE/pushrod_task.c](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/pushrod_task.c)。

四路编号 1～4，依次对应 PC0～PC3。初始化后输出低电平；控制对象的另一输入按驱动约定接 GND。GPIO 为控制信号，推杆电源与驱动电路须按硬件要求接线。

~~~c
Pushrod_Init();
Pushrod_Result result = Pushrod_Start(1); // PC0 输出高电平
Pushrod_Stop(1);                         // PC0 输出低电平
~~~

接口返回 PUSHROD_OK、PUSHROD_INVALID_ARGUMENT 或 PUSHROD_NOT_INITIALIZED。当前任务初始化后保持 PC0～PC3 低电平。保留的 PushrodTask_RequestStartFromISR() 只在 STEPPER_TARGET_REACHED 时接受启动请求；当前串口接收已移除字符 3 的推杆触发。任务每 10 ms 检查请求并再次确认到位，调用 Pushrod_Stop(1) 将 PC0 保持低电平，PC1～PC3 不变；不再自动循环或延时拉低。未到位、返回中、故障或仅达到步数上限时收到 3 会被忽略；请求处理前已开始返回也不执行。高电平由 Pushrod_Stop(1) 或复位清除，激光选择命令不会操作 PC0。


## VL53L0X 激光测距

`vl53l0x.c` / `vl53l0x.h` 已加入 Keil 工程 HARDWARE 分组，使用 STM32F407 标准外设库。独立开漏软件 I²C：**PA8=SCL，PC9=SDA**，不使用原有 PB10/PB11 IMU 总线，也不占用定时器。

### 接线

| 模块 | STM32 |
| --- | --- |
| SCL | PA8 |
| SDA | PC9 |
| GND | GND |
| VCC | 按模块规格供电，优先使用 3.3V |
| 激光 1 XSHUT | PA2，上电低，文本 1 使能 |
| 激光 2 XSHUT | PA3，上电低，文本 2 使能 |
| GPIO1 | 不连接，驱动轮询结果就绪寄存器 |

SCL/SDA 需要外部上拉或模块自带上拉；GPIO 内部弱上拉不能替代 I²C 总线上拉。确认信号电平适合 STM32 与模块，不能将总线上拉至 5V。裸芯片的 AVDD 范围为 2.6～3.5V，2V8 I/O 模式的高电平应与 AVDD 匹配。

PA8 原为蜂鸣器引脚，现在 `LED.H` 中 `BUZZER_ENABLED=0`，蜂鸣器初始化和所有控制写入均禁用。**接线时必须断开 PA8 上原有蜂鸣器及其驱动电路**，防止负载影响 SCL。PC9 不能再同时用于原 TIM8 四通道舵机或遥控输入；当前步进模块只使用 PC8。

### 单次测距

当前工程由 distance_task 调用驱动。以下为独立使用示例，使用前须停止默认测距任务，避免并发访问。先完成系统时钟、`delay_init(168)` 和板级初始化，再初始化传感器；不要在中断里调用。以下代码放在一个应用任务中，其他任务不要同时访问驱动：

```c
#include "vl53l0x.h"
#include "delay.h"

VL53L0X_Result result;
VL53L0X_Status status;

status = VL53L0X_Init();
if (status == VL53L0X_OK) {
    status = VL53L0X_ReadSingle(&result);
    if (status == VL53L0X_OK) {
        /* result.distance_mm 为有效距离，单位 mm */
    }
}
```

### 连续测距

```c
status = VL53L0X_Init();
if (status == VL53L0X_OK)
    status = VL53L0X_StartContinuous();

/* 在同一个任务中周期调用，建议每次间隔 10ms 或更长 */
status = VL53L0X_ReadContinuous(&result);
if (status == VL53L0X_OK) {
    /* 使用 result.distance_mm */
} else if (status == VL53L0X_NOT_READY) {
    /* 本次没有新结果，稍后再读取 */
}

/* 不再测距时调用 */
status = VL53L0X_StopContinuous();
```

默认 7 位地址为 `0x29`，对应手册中的写地址 `0x52`、读地址 `0x53`。初始化包含 `0xC0=0xEE` 器件识别、2V8 I/O、出厂 SPAD 映射配置、默认调谐、100ms 测量预算、VHV 与相位参考校准；不包含用户盖板串扰和距离偏移校准。

`VL53L0X_ERROR_RANGE` 表示当前距离不可作为有效值使用，结果结构保留原始状态用于诊断。`raw_range_status` 是设备寄存器的原始状态，不是 ST API 的 `RangeStatus`；本驱动仅接受原始状态 11。通信错误、参数错误或未就绪时不更新输出结构，不能使用旧值冒充新测量。

单次测距和初始化等待使用有限次数轮询，每个等待阶段最多 200 次，每次附加约 1ms 延时；总耗时还包括 I²C 通信及任务调度。连续读取不等待测量完成，只进行就绪查询和必要的数据传输。时钟拉伸等待亦有上限，I²C NACK/通信超时后驱动失效，须重新 `VL53L0X_Init()`；若仍失败，检查接线、上拉及 XSHUT，必要时重启传感器电源。

参考用户提供的 `VL53L0X.pdf`（DocID029104 Rev 1，尤其第 8、14～21 页）及同目录 Arduino 示例。默认初始化表来源于随附 Pololu/ST 示例库，许可保留于 `LICENSE_VL53L0X.txt`，并修正示例 SPAD 恢复阶段的括号错误。原始状态判定参照 [ST 驱动源码](https://github.com/stm32duino/VL53L0X/blob/main/src/vl53l0x_class.cpp)。本模块提供基础测距，不等同于完整 ST API 的软件信号质量评估和工厂校准功能。

历史验证记录：测距驱动加入时曾使用 Keil ARMCC 5.06 编译、链接通过；尚无传感器硬件测距或逻辑分析仪时序验证记录。本次仅整合文档，未重新编译固件。

当前驱动直接从 NVM 读取出厂 SPAD 信息，保留配置回读校验和错误处理。临时初始化阶段、SPAD 数量和光信号率诊断接口已移除。两路 XSHUT 分别接 PA2、PA3，由测距任务互斥输出高电平释放；早期 XSHUT 不外接的接线说明已不适用。默认任务使用单次测距；连续接口仅供底层独立使用，勿与测距任务并发调用。

### 命令使能

PA2/PA3 上电都为低电平。收到串口 1 的文本 1 或 2 后，测距任务先将两路拉低，再拉高选中 XSHUT，等待 10 ms 后初始化并开始测距。初始化失败约间隔 1 秒重试，切换请求可中断重试等待。


### 测距任务与 OLED

`distance_task.c` / `distance_task.h` 已加入 Keil 工程，由 `start_task` 直接调用 `xTaskCreate(distance_task, ...)` 创建。使用 PA8=SCL、PC9=SDA；测距任务启动后等待文本 1/2，分别选择 PA2/PA3 控制的传感器。

任务独占传感器总线，使用 100ms 测量预算的单次测距，每次测量前延时 50ms。通信错误或超时后间隔 1 秒重新初始化。无效测量不作为有效距离发布；超过 500ms 未更新的数据快照返回超时。

OLED 测距页新增 Steps 累计 STEP 脉冲数，运动时刷新，停止后保留最终计数（包含细分）。OLED 默认显示测距页，保留距离（mm）和基本状态，删除 Raw、SPAD、信号率和初始化阶段诊断。显示函数由现有 show_task 统一调用和刷新；长按 PE0 用户键约 0.6 秒循环切换三页。测距任务不直接操作 OLED 总线。

头文件提供任务入口 distance_task、任务优先级和栈大小，以及数据读取和显示接口。测距轮询和重试参数封装在 .c 文件中。应用读取方式：

```c
#include "distance_task.h"

DistanceTask_Data data;
DistanceTask_GetData(&data);
if (data.valid) {
    /* data.distance_mm 为有效距离，单位 mm */
}
```

结构体保留 distance_mm、valid、status 和 update_tick。无效或过期时 distance_mm 为 0，必须检查 valid，不能将 0 当成有效测距结果。




## IMU 航向控制

航向保持算法已从 ROS 2 上位机移至 C30D 固件。`Balance_task` 以 100 Hz 调用
`HeadingControl_Update()`，使用完成零偏修正后的 `imu.gyro.z` 计算底盘角速度修正量。

### 生效条件

- C30D 已完成上电 10 秒 IMU 零偏采集；采集期间底盘必须保持静止。
- 当前为默认 USART3/ROS 控制模式。
- 车型不是阿克曼车型。
- 未启用 APP、航模遥控器、PS2、CAN、USART1、USART5 或自动回充控制。

切换到其他控制模式时 PID 状态会清零；重新进入 ROS 控制模式时会以当前方向作为新的零航向。

PA8 已用于测距 SCL，BUZZER_ENABLED=0；当前无蜂鸣提示，不能以是否听到蜂鸣判断校准完成。

### 参数

参数位于 [BALANCE/heading_control.h](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/BALANCE/heading_control.h)，原头文件名为 `heading_control.h`：

- `HEADING_CONTROL_KP`：1.32
- `HEADING_CONTROL_KI`：0.0
- `HEADING_CONTROL_KD`：0.18
- `HEADING_CONTROL_MAX_ANGULAR_VELOCITY`：1.0 rad/s
- `HEADING_CONTROL_MIN_ANGULAR_VELOCITY`：0.08 rad/s
- `HEADING_CONTROL_INTEGRAL_LIMIT`：0.5
- `HEADING_CONTROL_YAW_TOLERANCE`：0.01 rad
- `HEADING_CONTROL_GYRO_TOLERANCE`：0.03 rad/s
- `HEADING_CONTROL_GYRO_INTEGRATION_DEADZONE`：0.002 rad/s

启动阶段使用约 10 秒内的全部静止 IMU 样本计算平均零偏，不再使用单个末次采样。小于 `0.002 rad/s` 的残余静态角速度不参与航向积分，以抑制长时间累计漂移。

微分项直接使用陀螺仪角速度进行阻尼。只有航向误差和旋转角速度同时进入容差范围，控制器才停止输出并清空积分；尚未稳定且 PID 输出小于 `0.08 rad/s` 时，控制器保持同方向的最小角速度，以克服电机和底盘死区。

如需临时关闭底盘侧航向保持，将 `HEADING_CONTROL_ENABLE` 改为 `0` 并重新编译烧录。


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



## 编译与上电检查

Keil 工程：[USER/WHEELTEC.uvprojx](R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/USER/WHEELTEC.uvprojx)，历史使用 ARMCC 5.06 编译。烧录文件输出路径为 OBJ/WHEELTEC.hex；修改源码后须重新编译。本次文档整合未执行编译或硬件测试，不能据历史编译记录判断当前源码或现有 HEX 的状态。

1. 确认步进、推杆的启动动作，空载或架空验证，检查供电、共地、限流、EN 和细分设置。
2. 确认 PA8 的原蜂鸣器电路已断开，测距 SCL/SDA 有合适的上拉，两路 XSHUT 分别接 PA2、PA3。
3. 串口 ISP 烧录后将 BOOT0/下载开关拨回 0/RUN，退出烧录软件，再断电重启。
4. 车辆放平，上电后保持静止约 10 秒，等待 IMU 校准。当前没有蜂鸣提示。
5. 检查 OLED 测距状态及串口 24 字节反馈帧的帧头、帧尾和异或校验；无反馈时先排查供电、运行模式及端口。
6. 步进使用逻辑分析仪检查脉冲数量、加减速和停止电平；测距尚需实物验证。

首次动态测试使用低速、短时间指令，在空旷平地进行，并安排人员随时断电。每次测试结束连续发送零速度帧；调参期间避免多个控制来源同时生效。

串口 1 当前命令以本文“双路激光与串口命令”为准；原步进启动/返回入口保留但未绑定字符命令。

前进行程上限：STEPPER_MAX_PULSES=70000，由定时器中断限制，最多输出 70000 个 STEP 脉冲（含细分）；与距离 ≤130 mm 条件先到先停。达到步数上限置 STEPPER_LIMIT_REACHED，不发送 position ready（未确认距离到位），随后自动按实际前进脉冲数返回。

推杆底层与任务接口保留；当前串口字符 3 不再调用推杆请求。
