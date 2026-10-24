# VL53L0X 激光测距驱动

`vl53l0x.c` / `vl53l0x.h` 已加入 Keil 工程 HARDWARE 分组，使用 STM32F407 标准外设库。独立开漏软件 I²C：**PA8=SCL，PC9=SDA**，不使用原有 PB10/PB11 IMU 总线，也不占用定时器。

## 接线

| 模块 | STM32 |
| --- | --- |
| SCL | PA8 |
| SDA | PC9 |
| GND | GND |
| VCC | 按模块规格供电，优先使用 3.3V |
| XSHUT | PA3，上电后由测距任务拉高，默认开启 |
| GPIO1 | 不连接，驱动轮询结果就绪寄存器 |

SCL/SDA 需要外部上拉或模块自带上拉；GPIO 内部弱上拉不能替代 I²C 总线上拉。确认信号电平适合 STM32 与模块，不能将总线上拉至 5V。裸芯片的 AVDD 范围为 2.6～3.5V，2V8 I/O 模式的高电平应与 AVDD 匹配。

PA8 原为蜂鸣器引脚，现在 `LED.H` 中 `BUZZER_ENABLED=0`，蜂鸣器初始化和所有控制写入均禁用。**接线时必须断开 PA8 上原有蜂鸣器及其驱动电路**，防止负载影响 SCL。PC9 不能再同时用于原 TIM8 四通道舵机或遥控输入；当前步进模块只使用 PC8。

## 单次测距

驱动加入工程后由应用主动调用，不自动创建任务或改变底盘控制。先完成系统时钟、`delay_init(168)` 和板级初始化，再初始化传感器；不要在中断里调用。以下代码放在一个应用任务中，其他任务不要同时访问驱动：

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

## 连续测距

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

验证：使用 Keil ARMCC 5.06 对工程全部 70 个 C/汇编源文件编译并链接通过；尚未连接传感器进行硬件测距或逻辑分析仪时序验证。

当前驱动直接从 NVM 读取出厂 SPAD 信息，保留配置回读校验和错误处理。临时初始化阶段、SPAD 数量和光信号率诊断接口已移除。模块按随附原理图自带 XSHUT 上拉，实际接线保持 XSHUT 不外接，不接 MCU 的 3.3V 或 5V。默认任务使用单次测距；连续接口仅供底层独立使用，勿与测距任务并发调用。

## 默认开启

测距任务启动后立即将 PA3 配置为推挽高电平，释放 VL53L0X 的 XSHUT，等待 10 ms 后初始化并开始测距。初始化失败后间隔 1 秒重试，不需要用户按键。
