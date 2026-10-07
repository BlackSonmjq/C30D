#ifndef PUSHROD_H
#define PUSHROD_H

#include "stm32f4xx.h"

/*
 * 四路直流推杆方向控制（STM32F4 标准外设库，L9110 双输入驱动）。
 * 每个推杆占用一个完整 H 桥通道；双通道模块通常需要两块。
 * GPIO 只接模块 IA/IB，不能直接接推杆电机；模块与主控共地。
 *
 * 推杆编号     IA       IB       C30D-V2.0 原理图 H1 引脚号
 *     1       PC0      PC1             6 / 8
 *     2       PC2      PC3            10 / 12
 *     3       PC5      PC10           14 / 19
 *     4       PC11     PD15           20 / 15
 * 引脚号依据厂商原理图，不代表从任意观察方向数排针的次序。
 * 这些信号在当前工程中未被占用；实物按板上丝印和对应版本原理图接线。
 * 避开了轮子 PWM、编码器、IMU、串口、SWD、USB 和舵机接口。
 *
 * 功能：正转 IA=1/IB=0；反转 IA=0/IB=1；停止 IA=0/IB=0。
 * “正转”是否伸出取决于电机接线，第一次短时试动确认方向。
 * 停止是撤去驱动（L9110SL 为高阻释放），不是位置锁定或紧急制动。
 * 此驱动不含 PWM、位置同步、行程限位、过流检测或运行超时。
 * 启动后会持续运动，调用方必须按限位/运行时间主动调用停止。
 * 四路独立控制，并不保证四根推杆同步；机械联动需额外位置反馈。
 *
 * 供电须匹配实际驱动型号和推杆额定/堵转电流；资料中的 L9110SL
 * 并非 12V 驱动，不能将底盘 12V 电源直接接到该芯片模块。
 * 上电至初始化前 GPIO 仍是输入；必须依靠模块/外接下拉保持 IA/IB 为低。
 *
 * 接入方法：
 * 1. 将 pushrod.c 加入 Keil 的 HARDWARE 组；头文件目录已在原工程路径内。
 * 2. USER/main.c 顶部添加 #include "pushrod.h"。
 * 3. main() 内在 systemInit() 前调用 Pushrod_Init()，尽早置为停止。
 * 4. 在你的控制任务/命令处理处调用下面的运动函数，不要在上电流程试转。
 * 5. 所有运动命令由同一个任务管理；换向时先停止，待机械静止后再反转。
 *    本驱动不做阻塞延时，不会自行安排等待后的反转。
 */

#define PUSHROD_COUNT 4U

typedef enum
{
    PUSHROD_STOP = 0,
    PUSHROD_FORWARD = 1,
    PUSHROD_REVERSE = 2
} Pushrod_Direction;

typedef enum
{
    PUSHROD_OK = 0,
    PUSHROD_INVALID_ARGUMENT,
    PUSHROD_NOT_INITIALIZED,
    /* 直接换向被拒绝，该路已停止；待机械静止后再发新的方向命令。 */
    PUSHROD_REVERSAL_BLOCKED
} Pushrod_Result;

/* 初始化 8 个 GPIO 为推挽输出，四路全部停止。上电调用一次。 */
void Pushrod_Init(void);

/* id 是 1~4。未初始化或参数无效时不写 GPIO。
 * 返回 PUSHROD_OK 表示命令已应用，不代表实际推杆到位。
 */
Pushrod_Result Pushrod_Control(uint8_t id, Pushrod_Direction direction);
Pushrod_Result Pushrod_Forward(uint8_t id);
Pushrod_Result Pushrod_Reverse(uint8_t id);
Pushrod_Result Pushrod_Stop(uint8_t id);

/* 同时撤去四路驱动；初始化前调用无动作。 */
void Pushrod_StopAll(void);

#endif
