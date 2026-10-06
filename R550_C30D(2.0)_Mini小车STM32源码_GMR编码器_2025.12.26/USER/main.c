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

// 设为 1：上电后循环测试 A4988；设为 0：运行正常底盘程序。
#ifndef A4988_TEST_MODE
#define A4988_TEST_MODE 1
#endif

#if A4988_TEST_MODE
#include "stepper_test.h"
#endif

//Task priority    //�������ȼ�
#define START_TASK_PRIO	1

//Task stack size //�����ջ��С	
#define START_STK_SIZE 	256  

//Task handle     //������
TaskHandle_t StartTask_Handler;

//Task function   //������
void start_task(void *pvParameters);

//Main function //������
int main(void)
{ 
#if A4988_TEST_MODE
    // 仅测试 A4988 引脚和 STEP 定时器，不启动底盘任务。
    Stepper_TestLoop();
#else
  systemInit(); //Hardware initialization //Ӳ����ʼ��
	
	//Create the start task //������ʼ����
	xTaskCreate((TaskFunction_t )start_task,            //Task function   //������
							(const char*    )"start_task",          //Task name       //��������
							(uint16_t       )START_STK_SIZE,        //Task stack size //�����ջ��С
							(void*          )NULL,                  //Arguments passed to the task function //���ݸ��������Ĳ���
							(UBaseType_t    )START_TASK_PRIO,       //Task priority   //�������ȼ�
							(TaskHandle_t*  )&StartTask_Handler);   //Task handle     //������    					
	vTaskStartScheduler();  //Enables task scheduling //�����������	
#endif
}
 
//Start task task function //��ʼ����������
void start_task(void *pvParameters)
{
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
	
    vTaskDelete(StartTask_Handler); //Delete the start task //ɾ����ʼ����

    taskEXIT_CRITICAL();            //Exit the critical section//�˳��ٽ���
}






