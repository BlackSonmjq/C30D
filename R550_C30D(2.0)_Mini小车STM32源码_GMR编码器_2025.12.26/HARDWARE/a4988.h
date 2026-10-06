#ifndef __A4988_H
#define __A4988_H

#include "stm32f4xx.h"

#define A4988_MOTOR_Left  Left
#define A4988_MOTOR_Right  Right

/* Motor 1: PB14=DIR, PB15=STEP, PC6=EN (active low).
   Motor 2: PC7=DIR, PC8=STEP, PC9=EN (active low). */
void A4988_Init(void);
void A4988_SetDir(u8 motor, u8 high);
void A4988_Enable(u8 motor, u8 enable);
void A4988_Step(u8 motor);

#endif
