#ifndef TEST_REMOTE_BOARD_SYSTEM_H
#define TEST_REMOTE_BOARD_SYSTEM_H

#include <stdint.h>
#include <stdio.h>

#define configTICK_RATE_HZ 1000
#define CONTROL_FREQUENCY 100

typedef uint8_t u8;
typedef struct { float Encoder, Target, Motor_Pwm; } TestMotor;
extern TestMotor MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern float Voltage, Velocity_KP, Velocity_KI, LineDiffParam, Move_X, Move_Y, Move_Z;
extern int Allow_Recharge, Check, KEY, EN, Flag_Stop;

static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
static inline float float_abs(float x) { return x < 0 ? -x : x; }
static inline void OLED_Clear(void) {}
static inline void OLED_ShowString(int x, int y, const u8 *text) { (void)x; (void)y; (void)text; }
static inline void OLED_Refresh_Gram(void) {}

enum { USART1=1, USART2=2, USART_FLAG_TXE=1, USART_IT_TXE=1, RESET=0, DISABLE=0, ENABLE=1 };
static inline int USART_GetFlagStatus(int port, int flag) { (void)port; (void)flag; return 1; }
int USART_GetITStatus(int port, int flag);
void USART_ITConfig(int port, int flag, int enabled);
void USART_SendData(int port, uint8_t byte);

#endif
