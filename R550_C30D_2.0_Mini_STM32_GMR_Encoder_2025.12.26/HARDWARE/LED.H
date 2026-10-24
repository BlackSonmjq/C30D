#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"

#define LED_TASK_PRIO		3     //Task priority //任务优先级
#define LED_STK_SIZE 		128   //Task stack size //任务堆栈大小


/*--------Buzzer control pin--------*/
#define Buzzer_PORT GPIOA
#define Buzzer_PIN GPIO_Pin_8
/* PA8 is reserved for VL53L0X SCL. Do not enable with this wiring. */
#define BUZZER_ENABLED 0
#if BUZZER_ENABLED
#define Buzzer PAout(8)
#define Buzzer_Set(value) do { Buzzer = (value); } while (0)
#else
#define Buzzer 0
#define Buzzer_Set(value) ((void)(value))
#endif
/*----------------------------------*/

/*--------LED control pin--------*/
#define LED_PORT GPIOA
#define LED_PIN GPIO_Pin_12
//旧板C30DLED控制口
#define V1_0_LED PAout(12) 
/*----------------------------------*/
//新板C30DLED控制口
#define V1_1_LED PEout(8) 


void V1_1_LED_Init(void);
void V1_0_LED_Init(void);  
void Buzzer_Init(void); 
void Led_Flash(u16 time);
void led_task(void *pvParameters);
extern int Led_Count;
#endif
