#ifndef __CHECK_H
#define __CHECK_H

#include "sys.h"
#include "system.h"

typedef struct
{
	unsigned char Process;				//��־�Լ�ģʽ�Ľ���
	int Motor_forward;						//�����ת����ֵ
	int Motor_retreat;						//�����ת����ֵ
	unsigned char Usart2_check_dir;			//��������ң�ط����־λ
	unsigned char Usart3_check_flag;		//����3�Լ��־λ
	unsigned char Usart3_rec_buffer[50];//����3���ջ�����
	unsigned char Usart3_rec_count;
}Check_parameter;

#define CHECK_TASK_PRIO		4     //Task priority //�������ȼ�
#define CHECK_STK_SIZE 		512   //Task stack size //�����ջ��С

extern Check_parameter check_parameter;

void Check_task(void *pvParameters);
void Check_Key(void);

#endif
