/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V5.0
�޸�ʱ�䣺2022-05-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V5.0
Update��2022-05-05

All rights reserved
***********************************************/
#include "system.h"
#include "pushrod.h"
#include "servo_task.h"
#include "stepper_task.h"
#include "distance_task.h"
//Task priority    //�������ȼ�
#define START_TASK_PRIO	0

//Task stack size //�����ջ��С	
#define START_STK_SIZE 	256  

//Task handle     //������
TaskHandle_t StartTask_Handler;

//Task function   //������
void start_task(void *pvParameters);

//Main function //������
int main(void)
{ 
  systemInit(); //Hardware initialization //Ӳ����ʼ��
	//Create the start task //������ʼ����
	
	
	xTaskCreate((TaskFunction_t )start_task,            //Task function   //������
							(const char*    )"start_task",          //Task name       //��������
							(uint16_t       )START_STK_SIZE,        //Task stack size //�����ջ��С
							(void*          )NULL,                  //Arguments passed to the task function //���ݸ��������Ĳ���
							(UBaseType_t    )START_TASK_PRIO,       //Task priority   //�������ȼ�
							(TaskHandle_t*  )&StartTask_Handler);   //Task handle     //������    					
    any_printf(USART1, "boot: starting scheduler\r\n");
	vTaskStartScheduler();  //Enables task scheduling //�����������	
}
 
//Start task task function //��ʼ����������
void start_task(void *pvParameters)
{
    BaseType_t servo_created, stepper_created, distance_created;
    any_printf(USART1, "boot: start task running\r\n");
    taskENTER_CRITICAL(); //Enter the critical area //�����ٽ���
    //Create the task //��������
	if(Check == 1)
		xTaskCreate(Check_task,  "Check_task",  CHECK_STK_SIZE,  NULL, CHECK_TASK_PRIO,  NULL);
	else if(Check == 0)
		xTaskCreate(Balance_task,  "Balance_task",  BALANCE_STK_SIZE,  NULL, BALANCE_TASK_PRIO,  NULL);	//Vehicle motion control task //С���˶���������
	if(SysVal.HardWare_Ver==V1_0) 	//IMU data read task //IMU���ݶ�ȡ����,���ݲ�ͬ��Ӳ���汾������ͬ������.
		xTaskCreate(MPU6050_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
	else if( SysVal.HardWare_Ver==V1_1 )
		xTaskCreate(ICM20948_task,  "IMU_task",  IMU_STK_SIZE,  NULL, IMU_TASK_PRIO,  NULL);
	xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     NULL); //The OLED display displays tasks //OLED��ʾ����ʾ����
	xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      NULL);	//LED light flashing task //LED����˸����
	xTaskCreate(pstwo_task,    "PSTWO_task",    PS2_STK_SIZE,      NULL, PS2_TASK_PRIO,      NULL);	//Read the PS2 controller task //��ȡPS2�ֱ�����
	xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, DATA_TASK_PRIO,     NULL);	//Usartx3, Usartx1 and CAN send data task //����3������1��CAN������������
    servo_created = xTaskCreate(servo_task,      "servo_task",      SERVO_STK_SIZE,      NULL, SERVO_TASK_PRIO,      NULL);	//Servo actuator task //??????
    stepper_created = xTaskCreate(stepper_Task,      "stepper_task",      STEPPER_STK_SIZE,      NULL, STEPPER_TASK_PRIO,      NULL);	//Stepper motor control task //????????
    distance_created = xTaskCreate(distance_task,     "distance_task",     DISTANCE_STK_SIZE,     NULL, DISTANCE_TASK_PRIO,     NULL);
    any_printf(USART1, "boot: tasks servo=%ld stepper=%ld distance=%ld heap_free=%lu\r\n",
               (long)servo_created, (long)stepper_created, (long)distance_created,
               (unsigned long)xPortGetFreeHeapSize());
	vTaskDelete(StartTask_Handler); //Delete the start task //ɾ����ʼ����

    taskEXIT_CRITICAL();            //Exit the critical section//�˳��ٽ���
}



