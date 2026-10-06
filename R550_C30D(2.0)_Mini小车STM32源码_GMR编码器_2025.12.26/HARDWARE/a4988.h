#ifndef __A4988_H
#define __A4988_H

#include "stm32f4xx.h"

#define A4988_MOTOR_Left   1
#define A4988_MOTOR_Right  2

/* 左路：PB14=DIR、PB15=STEP；右路：PC7=DIR、PC8=STEP。
   两路 EN 均须由外部电路提供确定电平。 */
void A4988_Init(void);
void A4988_SetDir(u8 motor, u8 high);
void A4988_Step(u8 motor);

#endif
