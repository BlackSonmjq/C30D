#include "usartx.h"
#include "can_test.h"
#include "distance_protocol.h"
#include "distance_task.h"
#include "servo.h"
#include "camera_protocol.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* USART1 handles upper-computer commands/logs. USART3 receives camera frames.
 * Keep the legacy bridge disabled while USART3 is connected to the camera. */
#define USART3_RAW_FORWARD_ENABLED 0
#define USART3_FORWARD_SIZE 1024U
#if USART3_RAW_FORWARD_ENABLED
static volatile uint8_t usart3_forward[USART3_FORWARD_SIZE];
static volatile uint16_t usart3_forward_read, usart3_forward_write;
static volatile uint32_t usart3_forward_dropped;
#endif
#define USART1_BINARY_FEEDBACK_ENABLED 0
static DistanceProtocol_Receiver usart3_distance_receiver;
static volatile uint32_t usart3_rx_bytes, usart3_uart_errors, usart3_error_flags;
static volatile uint8_t usart3_raw[USART3_RAW_CAPTURE_SIZE], usart3_raw_write, usart3_raw_length;
static volatile u8 servo_probe_command;
static volatile u8 servo_debug_pending, servo_debug_busy;
static u8 servo_debug_prefix;
/* Task-owned timed sequence: establish zero, +180 degrees, return to zero.
 * Position pulses assume the existing 270-degree mode; no setup commands sent. */
/* Only a full position reply for the selected ID counts, never #OK! or echo. */
static u8 ServoDebug_Position(const char *frame, uint16_t length, u8 id, uint16_t *position)
{
    unsigned i, reply_id=0, pulse=0;
    if (length != 10U || frame[0]!='#' || frame[4]!='P' || frame[9]!='!') return 0;
    for (i=1; i<=3; ++i) {
        if (frame[i]<'0' || frame[i]>'9') return 0;
        reply_id=reply_id*10U+(unsigned)(frame[i]-'0');
    }
    for (i=5; i<=8; ++i) {
        if (frame[i]<'0' || frame[i]>'9') return 0;
        pulse=pulse*10U+(unsigned)(frame[i]-'0');
    }
    if (reply_id != id || pulse<500U || pulse>2500U) return 0;
    *position=(uint16_t)pulse; return 1;
}

/* Firmware variants reply #001MOD2! or #001PMOD2! to PMOD queries. */
static u8 ServoDebug_Mode(const char *frame, uint16_t length, u8 id, u8 *mode)
{
    unsigned i, reply_id=0, offset;
    if ((length!=9U && length!=10U) || frame[0]!='#' || frame[length-1U]!='!') return 0;
    for (i=1; i<=3; ++i) {
        if (frame[i]<'0' || frame[i]>'9') return 0;
        reply_id=reply_id*10U+(unsigned)(frame[i]-'0');
    }
    if (reply_id!=id) return 0;
    offset=length==9U ? 7U : 8U;
    if (length==9U ? memcmp(frame+4,"MOD",3)!=0 : memcmp(frame+4,"PMOD",4)!=0) return 0;
    if (frame[offset]<'1' || frame[offset]>'8') return 0;
    *mode=(u8)(frame[offset]-'0'); return 1;
}

static void ServoDebug_Process(void)
{
    static u8 id, phase, all_home, stage, attempts, stable, failed, mode_reads, requested_mode;
    static TickType_t move_started, query_started, last_motion_sent;
    static u8 motion_retries, stalled_samples, position_seen;
    static uint16_t last_position, half_turn_target;
    u8 request, mode, arrived=0;
    TickType_t now=xTaskGetTickCount();
    uint16_t target, length, position;
    char frame[SERVO_FRAME_CAPACITY];
    Servo_Result result;
    uint32_t mask=__get_PRIMASK();
    __disable_irq(); request=servo_debug_pending; servo_debug_pending=0; __set_PRIMASK(mask);
    if (request == 5U || (!phase && request)) {
        Servo_ClearRx(); all_home=(request==5U); id=all_home?1U:request;
        phase=all_home?3U:1U; stage=all_home?0U:10U; attempts=stable=failed=0; last_position=0;
        half_turn_target=SERVO_HALF_TURN_PULSE;
        mode_reads=0;
        any_printf(USART1,"SERVO debug=dma-feedback-3 request=%u\r\n",(unsigned)request);
    }
    if (request>=6U && request<=9U) {
        id=request-5U; requested_mode=id<=2U?2U:1U; phase=1; stage=20;
    }
    if (!phase) return;
    /* M1/M2 set mode 2; M3/M4 set mode 1. Write once, then read back. */
    if (stage==20U) {
        Servo_ClearRx(); result=Servo_SetMode(id,requested_mode); mode_reads=0;
        any_printf(USART1,"SERVO %u set mode%u tx=%u\r\n",(unsigned)id,(unsigned)requested_mode,(unsigned)result);
        query_started=now; stage=result==SERVO_OK?21U:23U; return;
    }
    if (stage==21U) {
        /* Capture ACK before clearing the mailbox for the mode query. */
        if (Servo_GetFrame(frame,sizeof(frame),&length)==SERVO_OK)
            any_printf(USART1,"SERVO %u setup reply: %s\r\n",(unsigned)id,frame);
        if ((TickType_t)(now-query_started)<pdMS_TO_TICKS(1000U)) return;
        Servo_ClearRx(); snprintf(frame,sizeof(frame),"#%03uPMOD!",(unsigned)id);
        result=Servo_SendCommand(frame); query_started=now; ++mode_reads;
        any_printf(USART1,"SERVO %u mode read attempt=%u tx=%u\r\n",(unsigned)id,(unsigned)mode_reads,(unsigned)result);
        stage=result==SERVO_OK?22U:23U; return;
    }
    if (stage==22U) {
        if (Servo_GetFrame(frame,sizeof(frame),&length)==SERVO_OK) {
            any_printf(USART1,"SERVO %u mode raw[%u]: %s\r\n",(unsigned)id,(unsigned)length,frame);
            if (ServoDebug_Mode(frame,length,id,&mode)) {
                any_printf(USART1,"SERVO %u mode%u %s (read=%u)\r\n",(unsigned)id,(unsigned)requested_mode,mode==requested_mode?"CONFIRMED":"FAILED",(unsigned)mode);
                stage=24;
            }
        }
        if (stage!=24U) {
            if ((TickType_t)(now-query_started)<pdMS_TO_TICKS(600U)) return;
            /* Retry reads only: do not repeatedly write persistent configuration. */
            if (mode_reads<3U) { stage=21; return; }
            stage=23;
        }
    }
    if (stage==23U || stage==24U) {
        if (stage==23U) any_printf(USART1,"SERVO %u mode%u NOT CONFIRMED; check connection/readback\r\n",(unsigned)id,(unsigned)requested_mode);
        Servo_ClearRx(); phase=0;
        mask=__get_PRIMASK(); __disable_irq();
        if (!servo_debug_pending) servo_debug_busy=0;
        __set_PRIMASK(mask); return;
    }
    /* Read existing mode only: never send a mode-setting command. */
    if (stage==10U) {
        snprintf(frame,sizeof(frame),"#%03uPMOD!",(unsigned)id);
        Servo_ClearRx(); result=Servo_SendCommand(frame);
        query_started=now; stage=11; ++mode_reads;
        any_printf(USART1,"SERVO %u mode query tx=%u\r\n",(unsigned)id,(unsigned)result);
        return;
    }
    if (stage==11U) {
        result=Servo_GetFrame(frame,sizeof(frame),&length);
        if (result==SERVO_OK)
            any_printf(USART1,"SERVO %u mode reply: %s\r\n",(unsigned)id,frame);
        if (result==SERVO_OK && ServoDebug_Mode(frame,length,id,&mode)) {
            if (mode<=4U) {
                half_turn_target=mode>=3U ? 2500U : SERVO_HALF_TURN_PULSE;
                any_printf(USART1,"SERVO %u mode=%u target180=%u\r\n",(unsigned)id,(unsigned)mode,(unsigned)half_turn_target);
                Servo_ClearRx(); stage=0;
            } else {
                any_printf(USART1,"SERVO %u rejected: not position mode\r\n",(unsigned)id);
                phase=0; Servo_ClearRx();
                mask=__get_PRIMASK(); __disable_irq();
                if (!servo_debug_pending) servo_debug_busy=0;
                __set_PRIMASK(mask); return;
            }
        } else if ((TickType_t)(now-query_started)>=pdMS_TO_TICKS(600U)) {
            if (mode_reads<3U) { stage=10; return; }
            any_printf(USART1,"SERVO %u mode unknown: assuming 270deg target180=%u\r\n",(unsigned)id,(unsigned)half_turn_target);
            Servo_ClearRx(); stage=0;
        }
        return;
    }
    target=phase==2U ? half_turn_target : SERVO_HOME_PULSE;
    if (stage==0U) {
        Servo_ClearRx();
        result=Servo_MovePulse(id,target,SERVO_MOVE_MS);
        any_printf(USART1,"SERVO cmd=#%03uP%04uT%04u! tx=%u\r\n",
                   (unsigned)id,(unsigned)target,(unsigned)SERVO_MOVE_MS,(unsigned)result);
        ++attempts; stable=0; move_started=last_motion_sent=now; stage=1;
        motion_retries=0; stalled_samples=0; position_seen=0;
        any_printf(USART1,"SERVO %u target=%u attempt=%u tx=%u\r\n",(unsigned)id,(unsigned)target,(unsigned)attempts,(unsigned)result);
        return;
    }
    if (stage==1U) {
        if ((TickType_t)(now-last_motion_sent)<pdMS_TO_TICKS(SERVO_MOVE_MS)) return;
        Servo_ClearRx(); result=Servo_ReadPosition(id);
        query_started=now; stage=2;
        return;
    }
    if (Servo_GetFrame(frame,sizeof(frame),&length)==SERVO_OK) {
        if (ServoDebug_Position(frame,length,id,&position)) {
            int error=(int)position-(int)target;
            /* Use the same arrival tolerance for every servo and target. */
            int tolerance=(int)SERVO_POSITION_TOLERANCE;
            /* A valid but unchanged position far from the target can indicate
             * a lost motion command. Retry the same absolute target at most
             * twice, while retaining the original 3 s phase deadline. */
            if (position_seen && (error < -tolerance || error > tolerance) &&
                (int)position-(int)last_position >= -5 &&
                (int)position-(int)last_position <= 5) {
                ++stalled_samples;
            } else {
                stalled_samples=0;
            }
            last_position=position;
            position_seen=1;
            stable=(error>=-tolerance && error<=tolerance) ? stable+1U : 0U;
            any_printf(USART1,"SERVO %u pos=%u target=%u stable=%u\r\n",(unsigned)id,(unsigned)position,(unsigned)target,(unsigned)stable);
            arrived=stable>=2U;
            if (!arrived && stalled_samples>=1U && motion_retries<2U &&
                (TickType_t)(now-move_started)+pdMS_TO_TICKS(SERVO_MOVE_MS+300U)<pdMS_TO_TICKS(3000U)) {
                Servo_ClearRx();
                result=Servo_MovePulse(id,target,SERVO_MOVE_MS);
                ++motion_retries;
                stalled_samples=0;
                stable=0;
                last_motion_sent=now;
                stage=1;
                any_printf(USART1,"SERVO %u motion retry=%u pos=%u target=%u tx=%u\r\n",
                           (unsigned)id,(unsigned)motion_retries,(unsigned)position,
                           (unsigned)target,(unsigned)result);
                return;
            }
        }
        /* UART mailbox stops after one frame. Wait before the next query. */
    }
    if (arrived) {
        Servo_ClearRx();
        if (all_home && id<SERVO_COUNT) { ++id; stage=0; attempts=stable=0; last_position=0; return; }
        if (!all_home && phase<3U) { ++phase; stage=0; attempts=stable=0; last_position=0; return; }
        any_printf(USART1,"SERVO home within tolerance; failures=%u\r\n",(unsigned)failed);
    } else if ((TickType_t)(now-move_started)>=pdMS_TO_TICKS(3000U)) {
        Servo_ClearRx();
        if (phase!=2U && attempts<2U) { stage=0; return; }
        any_printf(USART1,"SERVO %u POSITION TIMEOUT target=%u last=%u (0=no reply)\r\n",(unsigned)id,(unsigned)target,(unsigned)last_position);
        failed=1;
        if (all_home && id<SERVO_COUNT) { ++id; stage=0; attempts=stable=0; last_position=0; return; }
        /* If outward travel fails, still attempt a verified return to zero. */
        if (!all_home && phase==2U) { phase=3; stage=0; attempts=stable=0; last_position=0; return; }
    } else {
        if ((TickType_t)(now-query_started)>=pdMS_TO_TICKS(150U)) {
            /* Count successive valid positions across missing replies. An
             * out-of-range position still resets stable; the 3 s bound remains. */
            stage=1;
        }
        return;
    }
    phase=0; Servo_ClearRx();
    mask=__get_PRIMASK(); __disable_irq();
    if (!servo_debug_pending) servo_debug_busy=0;
    __set_PRIMASK(mask);
}


SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
SEND_AutoCharge_DATA Send_AutoCharge_Data;

// ����IP��ַ�洢��
u8 Received_IP[4] = {0, 0, 0, 0};          // �洢����ȷ�ϵ�IP��ַ
// IP��ȡ������ر���
u8 Last_IP[4] = {0, 0, 0, 0};              // �洢��һ�ν��յ�IP������������У�飩
u8 IP_Frame_Valid = 0;                     // IP֡��Ч��־
u16 IP_Valid_Count = 0;                    // ������ͬIP������
u8 Debug_Flag=0;
/**************************************************************************
Function: Usartx3, Usartx1,Usartx5 and CAN send data task 
Input   : none
Output  : none
�������ܣ�����3������1������5��CAN������������
��ڲ�������
����  ֵ����
**************************************************************************/
void data_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
     u8 probe_requested, probe_active = 0;
     TickType_t probe_started = 0;
     Servo_Result probe_result;
     Servo_Diagnostics servo_diag;
	
   any_printf(USART1, "boot: data task running\r\n");
   while(1)
    {	
			//The task is run at 20hz
			//��������20Hz��Ƶ������
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));
            CAN_Test_Process();
            ServoDebug_Process();
            /* Consume the UART1 diagnostic request in task context. */
            taskENTER_CRITICAL();
            probe_requested = servo_probe_command;
            servo_probe_command = 0;
            taskEXIT_CRITICAL();

            /* USART3 's': receive-only UART4 probe; report on USART1. */
            if (probe_requested) {
                probe_result = (probe_active || servo_debug_busy) ? SERVO_BUSY : Servo_StartRxProbe();
                if (probe_result == SERVO_OK) {
                    probe_started = xTaskGetTickCount();
                    probe_active = 1;
                    Servo_GetDiagnostics(&servo_diag);
                    any_printf(USART1, "servo probe start pc10=%u window_ms=500\r\n",
                               (unsigned int)servo_diag.pc10_high);
                } else {
                    any_printf(USART1, "servo probe rejected result=%u\r\n", (unsigned int)probe_result);
                }
            }
            if (probe_active && (TickType_t)(xTaskGetTickCount() - probe_started) >= pdMS_TO_TICKS(500U)) {
                Servo_GetDiagnostics(&servo_diag);
                Servo_ClearRx();
                probe_active = 0;
                any_printf(USART1, "servo probe irq=%lu bytes=%lu errors=%lu flags=0x%02lX sr=0x%04lX rx=%u pc10=%u\r\n",
                           (unsigned long)servo_diag.irq_count,
                           (unsigned long)servo_diag.byte_count,
                           (unsigned long)servo_diag.error_count,
                           (unsigned long)servo_diag.error_flags,
                           (unsigned long)servo_diag.last_status,
                           (unsigned int)servo_diag.receiving,
                           (unsigned int)servo_diag.pc10_high);
            }

			if(Check==0)
			{
				//Assign the data to be sent
				//��Ҫ���з��͵����ݽ��и�ֵ
				data_transition(); 
#if USART1_BINARY_FEEDBACK_ENABLED
                USART1_SEND();
#endif
				USART5_SEND();		 //Serial port 5 sends data //����5��������
				CAN_SEND();
        //CAN send data //CAN��������	
			}

		}
}
/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
�������ܣ����ڷ��͵����ݽ��и�ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //֡ͷ
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //֡β
	
	//According to different vehicle types, different kinematics algorithms were selected to carry out the forward kinematics solution, 
	//and the three-axis velocity was obtained from each wheel velocity
	//���ݲ�ͬ����ѡ��ͬ�˶�ѧ�㷨�����˶�ѧ���⣬�Ӹ������ٶ���������ٶ�
	switch(Car_Mode)
	{	
		case Mec_Car:case Mec_Car_V550:
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4)*1000;
	    Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder-MOTOR_D.Encoder)/4)*1000; 
	    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4/(Axle_spacing+Wheel_spacing))*1000;         
		  break; 
		
    case Omni_Car:      
			Send_Data.Sensor_Str.X_speed = ((MOTOR_C.Encoder-MOTOR_B.Encoder)/2/X_PARAMETER)*1000; 
	    Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder*2-MOTOR_B.Encoder-MOTOR_C.Encoder)/3)*1000; 
	    Send_Data.Sensor_Str.Z_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder)/3/Omni_turn_radiaus)*1000;      
		  break; 
    
		case Akm_Car:  
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; 
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/Wheel_spacing)*1000;
		  break; 
		
		case Diff_Car: 
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; 
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/Wheel_spacing)*1000;
			break; 
		
		case FourWheel_Car:case FourWheel_Car_V550:
      Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4)*1000; 
	    Send_Data.Sensor_Str.Y_speed = 0;
	    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_B.Encoder-MOTOR_A.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/2/(Axle_spacing+Wheel_spacing))*1000;
		 break; 
		
		case Tank_Car:   
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; 
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/(Wheel_spacing)*1000);
			break; 
	}
	
	//The acceleration of the triaxial acceleration //���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Accelerometer.X_data= imu.accel.y; //The accelerometer Y-axis is converted to the ros coordinate X axis //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Accelerometer.Y_data=-imu.accel.x; //The accelerometer X-axis is converted to the ros coordinate y axis //���ٶȼ�X��ת����ROS����Y��
	Send_Data.Sensor_Str.Accelerometer.Z_data= imu.accel.z; //The accelerometer Z-axis is converted to the ros coordinate Z axis //���ٶȼ�Z��ת����ROS����Z��
	
	//The Angle velocity of the triaxial velocity //���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Gyroscope.X_data= imu.gyro.y; //The Y-axis is converted to the ros coordinate X axis //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Gyroscope.Y_data=-imu.gyro.x; //The X-axis is converted to the ros coordinate y axis //���ٶȼ�X��ת����ROS����Y��
	if(Flag_Stop==0) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //����������λʹ��״̬����ô��������Z����ٶ�
		Send_Data.Sensor_Str.Gyroscope.Z_data=imu.gyro.z;  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //����������Ǿ�ֹ�ģ��������λʧ�ܣ�����ô���͵�Z����ٶ�Ϊ0		
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;        
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//��ص�ѹ(���ｫ�������Ŵ�һǧ�����䣬��Ӧ���ڽ��ն��ڽ��յ����ݺ�Ҳ����Сһǧ��)
	Send_Data.Sensor_Str.Power_Voltage = Voltage*1000; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //֡ͷ
  Send_Data.buffer[1]=Flag_Stop; //Car software loss marker //С������ʧ�ܱ�־λ
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//С�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed >>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed ;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed >>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed ;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU���ٶȼ�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU���ٶȼ�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//��ص�ѹ,���Ϊ����8λ���ݷ���
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //����У��λ���㣬ģʽ1�Ƿ�������У��
	Send_Data.buffer[22]=Check_Sum(22,1); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //֡β

	///////////////////////�Զ��س���ر�����ֵ/////////////////////
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Header = AutoCharge_HEADER;   //֡ͷ��ֵ0x7C
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail = AutoCharge_TAIL;		//֡β��ֵ0x7F
	Send_AutoCharge_Data.AutoCharge_Str.Charging_Current = (short)Charging_Current;//��������ֵ
	
	Send_AutoCharge_Data.AutoCharge_Str.RED = RED_STATE; //�����־λ��ֵ
	Send_AutoCharge_Data.AutoCharge_Str.Charging = Charging; //�Ƿ��ڳ���־λ��ֵ

	Send_AutoCharge_Data.buffer[0] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Header;		//֡ͷ0x7C
	Send_AutoCharge_Data.buffer[1] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current>>8;//��������8λ
	Send_AutoCharge_Data.buffer[2] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current;	//��������8λ
	Send_AutoCharge_Data.buffer[3] = Send_AutoCharge_Data.AutoCharge_Str.RED;				//�Ƿ���յ������־λ
	Send_AutoCharge_Data.buffer[4] = Send_AutoCharge_Data.AutoCharge_Str.Charging;			//�Ƿ��ڳ���־λ
	Send_AutoCharge_Data.buffer[5] = Allow_Recharge;										//�Ƿ����Զ��س�
	Send_AutoCharge_Data.buffer[6] = Check_Sum_AutoCharge(6,1);								//У��λ
	Send_AutoCharge_Data.buffer[7] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail;		//֡β0x7F
	///////////////////////�Զ��س���ر�����ֵ/////////////////////
	
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
�������ܣ�����1��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART1_SEND(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		usart1_send(Send_Data.buffer[i]);
	}	 
	if(Get_Charging_HardWare==1)
	{
		//���ڻس�װ��ʱ�����ϲ㷢���Զ��س���ر���
		for(i=0; i<8; i++)
		{
			usart1_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}	
}

void USART3_Return(void)
{
	for(int i=0; i<check_parameter.Usart3_rec_count; i++)
	{
		usart3_send(check_parameter.Usart3_rec_buffer[i]);
	}
	usart3_send('\r');
	usart3_send('\n');
}
/**************************************************************************
Function: Serial port 5 sends data
Input   : none
Output  : none
�������ܣ�����5��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART5_SEND(void)
{
  unsigned char i = 0;	
	for(i=0; i<24; i++)
	{
		usart5_send(Send_Data.buffer[i]);
	}	 
	if(Get_Charging_HardWare==1)
	{
		//���ڻس�װ��ʱ�����ϲ㷢���Զ��س���ر���
		for(i=0; i<8; i++)
		{
			usart5_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}
}
/**************************************************************************
Function: CAN sends data
Input   : none
Output  : none
�������ܣ�CAN��������
��ڲ�������
�� �� ֵ����
**************************************************************************/
void CAN_SEND(void) 
{
	u8 CAN_SENT[8],i;
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i];
	}
	CAN1_Send_Num(0x101,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+8];
	}
	CAN1_Send_Num(0x102,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+16];
	}
	CAN1_Send_Num(0x103,CAN_SENT);
	
	//////////////////�Զ��س�������ݷ���//////////////////
	if(Get_Charging_HardWare) CAN_Send_AutoRecharge();
}
/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
�������ܣ�����1��ʼ��
��ڲ�������
�� �� ֵ����
**************************************************************************/
void uart1_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock //ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Subpriority //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	NVIC_Init(&NVIC_InitStructure);	
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //��ʼ������1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //ʹ�ܴ���1
}
/**************************************************************************
Function: Serial port 2 initialization
Input   : none
Output  : none
�������ܣ�����2��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart2_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
	//UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
  //Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);	
	
	//USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //��ʼ������2
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);                     //Enable serial port 2 //ʹ�ܴ���2 
}
/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
�������ܣ�����3��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);      //Initialize serial port 3 //��ʼ������3
	
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                     //Enable serial port 3 //ʹ�ܴ���3 
}
/**************************************************************************
Function: Serial port 5 initialization
Input   : none
Output  : none
�������ܣ�����5��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart5_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//PC12 TX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
		//PD2 RX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //��ʼ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
  USART_Init(UART5, &USART_InitStructure);      //Initialize serial port 5 //��ʼ������5
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
  USART_Cmd(UART5, ENABLE);                     //Enable serial port 5 //ʹ�ܴ���5
}
/* Camera RX on USART3 (PD9), via the onboard CH9102 USB bridge.
 * ISR only parses bytes and updates diagnostics. No RTOS calls, OLED writes
 * or motion requests here. UART errors invalidate the partial frame so later
 * traffic can resynchronize. See camera_protocol.h for integration rules.
 */
static Camera_Receiver camera_receiver;
void Camera_GetData(Camera_Data *data)
{
    uint32_t mask=__get_PRIMASK();
    if (!data) return;
    __disable_irq(); *data=camera_receiver.data; __set_PRIMASK(mask);
}
int USART3_IRQHandler(void)
{
    uint32_t sr=USART3->SR;
    if (sr & (USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
        uint8_t byte=(uint8_t)USART3->DR;
        if (sr & (USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
            ++camera_receiver.data.uart_errors;
            ++camera_receiver.data.bytes;
            camera_receiver.length=0;
        } else Camera_Push(&camera_receiver,byte);
    }
    return 0;
}

static uint8_t ATCommandFeedBack_BT04A(uint8_t recv)
{
	#define DEBUG_BT04ACommand 0
	
	uint8_t isFilter = 0;//�Ƿ��������ַ�ͨ��,1����,0������
	static uint8_t lastrecv = 0;
	static uint8_t filterIndex = 0;
	
	const char* BT04AConnect = "+CONNECTING<<XX:XX:XX:XX:XX:XX\r\n+CONNECTED\r\n";
	const char* BT04ADisConnect = "+DISC:SUCCESS\r\n+READY\r\n+PAIRABLE\r\n";
	enum{
		BT04A_NORMAL=  0,
		BT04A_CONNECTSTART,
		BT04A_DISCONNECTSTART,
	};
	
	static uint8_t statemachine = BT04A_NORMAL;
	
	switch( statemachine )
	{
		case BT04A_NORMAL:
			if( recv=='C'&&lastrecv=='+' )
			{
				statemachine = BT04A_CONNECTSTART;//���յ�����ֵ,��ʼƥ��
				isFilter = 1;//�����ַ�
				filterIndex = 2; //2�ſ�ʼ����
			}
			else if( recv=='D'&&lastrecv=='+' )
			{
				statemachine = BT04A_DISCONNECTSTART;//���յ�����ֵ,��ʼƥ��
				isFilter = 1;//�����ַ�
				filterIndex = 2; //2�ſ�ʼ����
			}
			break;
		case BT04A_CONNECTSTART:
			if( BT04AConnect[filterIndex] == recv )  //��ʼ���������ַ�,����ֽ�ƥ��
			{
				isFilter = 1;//ƥ�������ֶ�,�����ƥ�������
				
				#if 1== DEBUG_BT04ACommand 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else if( (filterIndex>=13&&filterIndex<=29) && \
       				  ((recv>='0'&&recv<='9')||(recv>='a'&&recv<='z')) )//���뵽MAC��ַƥ��׶�.�ý׶�ƥ�� 0~9 ��a~z�ֶ�
			{
				isFilter = 1;//MAC��ַ����
				#if 1== DEBUG_BT04ACommand 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				//��������,�����ַ�ͨ��.���˳�����ģʽ
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand 
				printf("No:get:%c,but:%c\r\n",recv,BT04AConnect[filterIndex]);
				#endif
			}
			
			//����ֱ����ɹ����б�
			filterIndex++;
			if( filterIndex == strlen(BT04AConnect) )
			{
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand 
				printf("filter con done!\r\n");
				#endif
			}
			break;
		case BT04A_DISCONNECTSTART:
			if( BT04ADisConnect[filterIndex] == recv )  //��ʼ���������ַ�,����ֽ�ƥ��
			{
				isFilter = 1;//ƥ�������ֶ�,�����ƥ�������
				#if 1== DEBUG_BT04ACommand 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand
				printf("No:get:%c,but:%c\r\n",recv,BT04ADisConnect[filterIndex]);
				#endif
			}				
			
			//����ֱ����ɹ����б�
			filterIndex++;
			if( filterIndex == strlen(BT04ADisConnect) ) 
			{
				statemachine = BT04A_NORMAL;
				#if 1== DEBUG_BT04ACommand
				printf("filter dis done!\r\n");
				#endif
			}
			break;
	}
	lastrecv = recv;
	
	return isFilter;
}


//JDY-33����ATָ�����
static uint8_t ATCommandFeedBack_JDY33(uint8_t recv)
{
	#define DEBUG_JDY33Command 0
	
	uint8_t isFilter = 0;//�Ƿ��������ַ�ͨ��,1����,0����
	static uint8_t lastrecv = 0;
	static uint8_t filterIndex = 0;
	
	const char* JDY33_SPPConnect = "+CONNECTING<<XX:XX:XX:XX:XX:XX\r\nCONNECTED\r\r\n";
	const char* JDY33_BLEConnect = "CONNECTED\r\r\n";
	const char* JDY33_DisConnect = "+DISC:SUCCESS\r\r\n\0"; //ע��\0���ᱻͳ���ֳ�,��Ҫ���а��ֳ�+1
	enum{
		JDY33_NORMAL=  0,
		JDY33_SPPCONNECTSTART,
		JDY33_BLECONNECTSTART,
		JDY33_DISCONNECTSTART,
	};
	
	static uint8_t statemachine = JDY33_NORMAL;
	
	switch( statemachine )
	{
		case JDY33_NORMAL:
			if( recv=='C'&&lastrecv=='+' )
			{
				statemachine = JDY33_SPPCONNECTSTART;//���յ�����ֵ,��ʼƥ��
				isFilter = 1;//�����ַ�
				filterIndex = 2; //2�ſ�ʼ����
			}
			else if( recv=='O'&&lastrecv=='C' )
			{
				statemachine = JDY33_BLECONNECTSTART;//���յ�����ֵ,��ʼƥ��
				isFilter = 1;//�����ַ�
				filterIndex = 2; //2�ſ�ʼ����
			}
			else if( recv=='D'&&lastrecv=='+' )
			{
				statemachine = JDY33_DISCONNECTSTART;//���յ�����ֵ,��ʼƥ��
				isFilter = 1;//�����ַ�
				filterIndex = 2; //2�ſ�ʼ����
			}
			else if( recv=='C'&&lastrecv!='C' )//����״̬���ж�����,�ٵ�������ж�
			{
				//�й�C������,"+C"��"CO"����ͨ��. "?CҲ������ͨ��",���ܱ�֤����ʱ�����ڿ�������,����Ҫ����С��,��Ҫ������C
				isFilter = 1;//��ֹͨ��
			}
			break;
		case JDY33_SPPCONNECTSTART:
			if( JDY33_SPPConnect[filterIndex] == recv )  //��ʼ���������ַ�,����ֽ�ƥ��
			{
				isFilter = 1;//ƥ�������ֶ�,�����ƥ�������
				
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else if( (filterIndex>=13&&filterIndex<=29) && \
       				  ((recv>='0'&&recv<='9')||(recv>='A'&&recv<='Z')) )//���뵽MAC��ַƥ��׶�.�ý׶�ƥ�� 0~9 ��a~z�ֶ�
			{
				isFilter = 1;//MAC��ַ����
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				//��������,�����ַ�ͨ��.���˳�����ģʽ
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP->No:get:%c,but:%c\r\n",recv,JDY33_SPPConnect[filterIndex]);
				#endif
			}
			
			//����ֱ����ɹ����б�
			filterIndex++;
			if( filterIndex == strlen(JDY33_SPPConnect) )
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP filter con done!\r\n");
				#endif
			}
			break;
		case JDY33_BLECONNECTSTART:
			if( JDY33_BLEConnect[filterIndex] == recv )  //��ʼ���������ַ�,����ֽ�ƥ��
			{
				isFilter = 1;//ƥ�������ֶ�,�����ƥ�������
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("BLE->No:get:%c,but:%c\r\n",recv,JDY33_BLEConnect[filterIndex]);
				#endif
			}				
			
			//����ֱ����ɹ����б�
			filterIndex++;
			if( filterIndex == strlen(JDY33_BLEConnect) ) 
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("ble filter dis done!\r\n");
				#endif
			}
			break;
		case JDY33_DISCONNECTSTART:
			if( JDY33_DisConnect[filterIndex] == recv )  //��ʼ���������ַ�,����ֽ�ƥ��
			{
				isFilter = 1;//ƥ�������ֶ�,�����ƥ�������
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("dis->No:get:%c,but:%c\r\n",recv,JDY33_DisConnect[filterIndex]);
				#endif
			}				
			
			//����ֱ����ɹ����б�
			filterIndex++;
			if( filterIndex == strlen(JDY33_DisConnect)+1 ) //+1Ϊ������ַ�'\0'
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("filter dis done!\r\n");
				#endif
			}
			break;
	}
	lastrecv = recv;
	
	return isFilter;
}

//����ģʽ
void RobotControl_SetDebugLevel(char uart_recv)
{
	
	static char res_buf[4];
	static char res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='L'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==4)
	{
		res_count = 0;
		
		if( res_buf[0]=='L'&&res_buf[1]=='O'&&res_buf[2]=='G')
		{
			switch( res_buf[3] )
			{
				case '3': Debug_Flag= 1; break;
				default: Debug_Flag= 0; break;
			}
			//������ʾ��
		}
	}
}
uint8_t FlashWriteFlag = 0;

int USART2_IRQHandler(void)
{	
	int Usart_Receive;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{	      
		static u8 Flag_PID,i,j,Receive[50],Last_Usart_Receive;
		static float Data;
				
		Usart_Receive=USART2->DR; //Read the data //��ȡ����
		
		//����������ATָ���
		uint8_t ATFilter = 0;
		ATFilter += ATCommandFeedBack_JDY33(Usart_Receive);
		ATFilter += ATCommandFeedBack_BT04A(Usart_Receive);
		if( ATFilter > 0 ) return 0; //�������һ�������Ĺ�����Ϣ,�����
		
		_System_Reset_(Usart_Receive);
		RobotControl_SetDebugLevel(Usart_Receive);
		if(SysVal.Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //����10��ǰ����������
		  return 0;	
		
		if(Check==0)
		{
		if(Usart_Receive==0x41&&Last_Usart_Receive==0x41&&APP_ON_Flag==0)
			//10 seconds after startup, press the forward button of APP to enter APP control mode
		  //The APP controls the flag position 1 and the other flag position 0
			//����10��֮�󣬰���APP��ǰ��������APP����ģʽ
		  //APP���Ʊ�־λ��1��������־λ��0
			PS2_ON_Flag=0,Remote_ON_Flag=0,APP_ON_Flag=1,CAN_ON_Flag=0,Usart1_ON_Flag=0,Usart5_ON_Flag=0;
    Last_Usart_Receive=Usart_Receive;			
	  
		if(Usart_Receive==0x4B) 
			//Enter the APP steering control interface
		  //����APPת����ƽ���
			Turn_Flag=1;  
	  else	if(Usart_Receive==0x49||Usart_Receive==0x4A) 
      // Enter the APP direction control interface		
			//����APP������ƽ���	
			Turn_Flag=0;	
		
		if(Turn_Flag==0) 
		{
			//App rocker control interface command
			//APPҡ�˿��ƽ�������
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				Flag_Direction=Usart_Receive-0x40;
			}
			else	if(Usart_Receive<=8)   
			{			
				Flag_Direction=Usart_Receive;
			}	
			else  Flag_Direction=0;
		}
		else if(Turn_Flag==1)
		{
			//APP steering control interface command
			//APPת����ƽ�������
			if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //Right rotation //����ת
			else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //Left rotation  //����ת
			else                         Flag_Left=0,Flag_Right=0;
			if     (Usart_Receive==0x41||Usart_Receive==0x45) Flag_Direction=Usart_Receive-0x40;
			else  Flag_Direction=0;
		}
		if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s //���ٰ�����+100mm/s
		if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons,   -100mm/s //���ٰ�����-100mm/s
	  
	 // The following is the communication with the APP debugging interface
	 //��������APP���Խ���ͨѶ
	 if(Usart_Receive==0x7B) Flag_PID=1;   //The start bit of the APP parameter instruction //APP����ָ����ʼλ
	 if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stops the bit    //APP����ָ��ֹͣλ
	
	if( Usart_Receive=='b' ) Allow_Recharge = !Allow_Recharge;
		
	 if(Flag_PID==1) //Collect data //�ɼ�����
	 {
		Receive[i]=Usart_Receive;
		i++;
	 }
	 if(Flag_PID==2) //Analyze the data //��������
	 {
			if(Receive[3]==0x50) 	 PID_Send=1;
			else if( Receive[3]==0x57 ) FlashWriteFlag = 1;
			else  if(Receive[1]!=0x23) 
			{								
				for(j=i;j>=4;j--)
				{
					Data+=(Receive[j-1]-48)*pow(10,i-j);
				}
				switch(Receive[1])
				 {
					 case 0x30:  RC_Velocity=Data;break;
					 case 0x31:  Velocity_KP=Data;break;
					 case 0x32:  Velocity_KI=Data;break;
					 case 0x33:  LineDiffParam=Data;break;
					 case 0x34:  break;
					 case 0x35:  break;
					 case 0x36:  break;
					 case 0x37:  break;
					 case 0x38:  break; 	
				 }
      }		
      //Relevant flag position is cleared			
      //��ر�־λ����			
			Flag_PID=0;
			i=0;
			j=0;
			Data=0;
			memset(Receive, 0, sizeof(u8)*50); //Clear the array to zero//��������
	 }
   if(RC_Velocity<0)   RC_Velocity=0; 
 }
		else if(Check==1)
		{
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				check_parameter.Usart2_check_dir = Usart_Receive-0x40;
			}
			else 
				check_parameter.Usart2_check_dir = 9;
		}
  }
  return 0;	
}

// ========== ����3�����жϴ��� ==========
void USART3_GetDistanceData(DistanceProtocol_Data *data)
{
    uint32_t mask;
    if (data == NULL) return;
    /* USART1 priority 2 can preempt RTOS critical sections (BASEPRI=5). */
    mask = __get_PRIMASK();
    __disable_irq();
    *data = usart3_distance_receiver.data;
    __set_PRIMASK(mask);
}

void USART3_GetDistanceDiagnostics(USART3_DistanceDiagnostics *data)
{
    uint32_t mask;
    uint8_t i, start;
    if (data == NULL) return;
    mask = __get_PRIMASK();
    __disable_irq();
    data->rx_bytes = usart3_rx_bytes;
    data->uart_errors = usart3_uart_errors;
    data->error_flags = usart3_error_flags;
    data->checksum_errors = usart3_distance_receiver.checksum_errors;
    data->tail_errors = usart3_distance_receiver.tail_errors;
    data->check_mode = (uint8_t)Check;
    /* Legacy diagnostic field now samples the distance input on USART1 RX. */
    data->pd9_high = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10);
    data->raw_length = usart3_raw_length;
    start = (uint8_t)((usart3_raw_write + USART3_RAW_CAPTURE_SIZE - usart3_raw_length) % USART3_RAW_CAPTURE_SIZE);
    for (i = 0; i < data->raw_length; ++i)
        data->raw[i] = usart3_raw[(start + i) % USART3_RAW_CAPTURE_SIZE];
    __set_PRIMASK(mask);
}

int USART1_IRQHandler(void)
{
    uint32_t status = USART1->SR;
    uint8_t byte;
    if (!(status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))) return 0;
    byte = (uint8_t)USART1->DR; /* Read SR then DR to clear receive/error flags. */
    ++usart3_rx_bytes;
    usart3_raw[usart3_raw_write] = byte;
    usart3_raw_write = (uint8_t)((usart3_raw_write + 1U) % USART3_RAW_CAPTURE_SIZE);
    if (usart3_raw_length < USART3_RAW_CAPTURE_SIZE) ++usart3_raw_length;
    if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        ++usart3_uart_errors;
        usart3_error_flags |= status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE);
        usart3_distance_receiver.length = 0;
        return 0;
    }
    /* ASCII S0 homes all; S1..S4 exercises one servo. */
    if (servo_debug_prefix == 2U) {
        servo_debug_prefix=0;
        if ((byte>='1' && byte<='4') && !servo_debug_busy) {
            servo_debug_busy=1;
            servo_debug_pending=(u8)(6U+byte-'1');
        }
        return 0;
    }
    if (byte == 'M') { servo_debug_prefix=2U; return 0; }
    if (servo_debug_prefix) {
        servo_debug_prefix = 0;
        if (byte >= '0' && byte <= '4' && (byte == '0' || !servo_debug_busy)) {
            servo_debug_busy = 1;
            servo_debug_pending = byte == '0' ? 5U : byte - '0';
        }
        return 0;
    }
    if (byte == 'S') { servo_debug_prefix = 1; return 0; }
    /* USART1 ASCII 1..4 select a local upper laser; do not forward commands. */
    if (byte >= '1' && byte <= '4') {
        StepperTask_RequestLaserFromISR((u8)(byte - '0'));
        return 0;
    }
    if (byte == '\r' || byte == '\n') return 0;
#if USART3_RAW_FORWARD_ENABLED
    if (status & USART_SR_RXNE) {
        uint16_t next = (usart3_forward_write + 1U) % USART3_FORWARD_SIZE;
        if (next != usart3_forward_read) {
            usart3_forward[usart3_forward_write] = byte;
            usart3_forward_write = next;
            USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
        } else {
            ++usart3_forward_dropped;
        }
    }
    return 0;
#else
    if (Check == 0) {
        /* Distance frames have their own buffer, never shared with motor commands.
         * No RTOS calls or startup delay: a complete frame is accepted at once. */
        DistanceProtocol_Push(&usart3_distance_receiver, byte);
    } else {
        usart3_distance_receiver.length = 0;
        /* Preserve bounded serial self-test echo. */
        if (check_parameter.Usart3_check_flag) return 0;
        if (check_parameter.Usart3_rec_count >= sizeof(check_parameter.Usart3_rec_buffer) - 1U)
            check_parameter.Usart3_rec_count = 0;
        if (byte == '#') {
            check_parameter.Usart3_rec_buffer[check_parameter.Usart3_rec_count] = '\0';
            check_parameter.Usart3_check_flag = 1;
        } else {
            check_parameter.Usart3_rec_buffer[check_parameter.Usart3_rec_count++] = byte;
        }
    }
    return 0;
#endif
}

/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
�������ܣ�����5�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int UART5_IRQHandler(void)
{	
	static u8 Count=0;
	u8 Usart_Receive;

	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{
		Usart_Receive = USART_ReceiveData(UART5);//Read the data //��ȡ����
		if(SysVal.Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //����10��ǰ����������
		  return 0;	
		
		//Fill the array with serial data
		//����������������
    Receive_Data.buffer[Count]=Usart_Receive;
		
		// Ensure that the first data in the array is FRAME_HEADER
		//ȷ�������һ������ΪFRAME_HEADER
		if(Usart_Receive == FRAME_HEADER||Count>0) 
			Count++; 
		else 
			Count=0;
		
		if (Count == 11) //Verify the length of the packet //��֤���ݰ��ĳ���
		{   
				Count=0; //Prepare for the serial port data to be refill into the array //Ϊ����������������������׼��
				if(Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet //��֤���ݰ���֡β
				{
					//Data exclusionary or bit check calculation, mode 0 is sent data check
					//�������λУ����㣬ģʽ0�Ƿ�������У��
					if(Receive_Data.buffer[9] ==Check_Sum(9,0))	 
				  {	
						float Vz;						
						//All modes flag position 0, USART1 control mode
            //����ģʽ��־λ��0��ΪUsart5����ģʽ						
						PS2_ON_Flag=0;
						Remote_ON_Flag=0;
						APP_ON_Flag=0;
						CAN_ON_Flag=0;
						Usart5_ON_Flag=0;
						command_lost_count=0;
						//Calculate the target speed of three axis from serial data, unit m/s
						//�Ӵ�������������Ŀ���ٶȣ� ��λm/s
						Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
						Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
						Vz    =XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
						if(Car_Mode==Akm_Car)
						{
							Move_Z=Vz_to_Akm_Angle(Move_X, Vz);
						}
						else
						{
							Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
						}				  }
			}
		}
	} 
  return 0;	
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
�������ܣ�����λ��������Ŀ��ǰ���ٶ�Vx��Ŀ����ٶ�Vz��ת��Ϊ������С������ǰ��ת��
��ڲ�����Ŀ��ǰ���ٶ�Vx��Ŀ����ٶ�Vz����λ��m/s��rad/s
����  ֵ��������С������ǰ��ת�ǣ���λ��rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz)
{
	float R, AngleR, Min_Turn_Radius;
	//float AngleL;
	
	//Ackermann car needs to set minimum turning radius
	//If the target speed requires a turn radius less than the minimum turn radius,
	//This will greatly improve the friction force of the car, which will seriously affect the control effect
	//������С����Ҫ������Сת��뾶
	//���Ŀ���ٶ�Ҫ���ת��뾶С����Сת��뾶��
	//�ᵼ��С���˶�Ħ���������ߣ�����Ӱ�����Ч��
	Min_Turn_Radius=MINI_AKM_MIN_TURN_RADIUS;
	
	if(Vz!=0 && Vx!=0)
	{
		//If the target speed requires a turn radius less than the minimum turn radius
		//���Ŀ���ٶ�Ҫ���ת��뾶С����Сת��뾶
		if(float_abs(Vx/Vz)<=Min_Turn_Radius)
		{
			//Reduce the target angular velocity and increase the turning radius to the minimum turning radius in conjunction with the forward speed
			//����Ŀ����ٶȣ����ǰ���ٶȣ����ת��뾶����Сת��뾶
			if(Vz>0)
				Vz= float_abs(Vx)/(Min_Turn_Radius);
			else	
				Vz=-float_abs(Vx)/(Min_Turn_Radius);	
		}		
		R=Vx/Vz;
		//AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
		AngleR=atan(Axle_spacing/(R+0.5f*Wheel_spacing));
	}
	else
	{
		AngleR=0;
	}
	
	return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//����ת�����м����
	short transition; 
	
	//����8λ�͵�8λ���ϳ�һ��16λ��short������
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //��λת��, mm/s->m/s						
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
�������ܣ�����1��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart1_send(u8 data)
{
    (void)USART_WriteByte(USART1, data);
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
�������ܣ�����2��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart2_send(u8 data)
{
    (void)USART_WriteByte(USART2, data);
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
�������ܣ�����3��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart3_send(u8 data)
{
    (void)USART_WriteByte(USART3, data);
}

/**************************************************************************
Function: Serial port 5 sends data
Input   : The data to send
Output  : none
�������ܣ�����5��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart5_send(u8 data)
{
    (void)USART_WriteByte(UART5, data);
}
/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
�������ܣ�����Ҫ����/���յ�����У����
��ڲ�����Count_Number��У���ǰ��λ����Mode��0-�Խ������ݽ���У�飬1-�Է������ݽ���У��
����  ֵ��У����
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//��Ҫ���͵����ݽ���У��
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//�Խ��յ������ݽ���У��
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}

//�Զ��س䷢���ֽ�ר��У�麯��
u8 Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//��Ҫ���͵����ݽ���У��
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_AutoCharge_Data.buffer[k];
	}
	
	return check_sum;	
}


//����ATָ��ץ������ָֹ����ŵ�����������������ͨ��
u8 AT_Command_Capture(u8 uart_recv)
{
	/*
	��������ʱ���͵��ַ���00:11:22:33:44:55Ϊ������MAC��ַ
	+CONNECTING<<00:11:22:33:44:55\r\n
	+CONNECTED\r\n
	��44���ַ�
	
	�����Ͽ�ʱ���͵��ַ�
	+DISC:SUCCESS\r\n
	+READY\r\n
	+PAIRABLE\r\n
	��34���ַ�
	\r -> 0x0D
	\n -> 0x0A
	*/
	
	static u8 pointer = 0; //��������ʱָ���¼��
	static u8 bt_line = 0; //��ʾ�����ڵڼ���
	static u8 disconnect = 0;
	static u8 connect = 0;
	
	//�Ͽ�����
	static char* BlueTooth_Disconnect[3]={"+DISC:SUCCESS\r\n","+READY\r\n","+PAIRABLE\r\n"};
	
	//��ʼ����
	static char* BlueTooth_Connect[2]={"+CONNECTING<<00:00:00:00:00:00\r\n","+CONNECTED\r\n"};


	//�����ʶ������ʼ����(ʹ��ʱҪ-1)
	if(uart_recv=='+') 
	{
		bt_line++,pointer=0; //�յ���+������ʾ�л�������	
		disconnect++,connect++;
		return 1;//ץ������ֹ����
	}

	if(bt_line!=0) 
	{	
		pointer++;

		//��ʼ׷�������Ƿ���϶Ͽ�������������ʱȫ�����Σ�������ʱȡ������
		if(uart_recv == BlueTooth_Disconnect[bt_line-1][pointer])
		{
			disconnect++;
			if(disconnect==34) disconnect=0,connect=0,bt_line=0,pointer=0;
			return 1;//ץ������ֹ����
		}			

		//׷���������� (bt_line==1&&connect>=13)����������MAC��ַ��ÿһ������MAC��ַ������ͬ������ֱ�����ι�ȥ
		else if(uart_recv == BlueTooth_Connect[bt_line-1][pointer] || (bt_line==1&&connect>=13) )
		{		
			connect++;
			if(connect==44) connect=0,disconnect=0,bt_line=0,pointer=0;		
			return 1;//ץ������ֹ����
		}	

		//��ץ���ڼ��յ��������ֹͣץ��
		else
		{
			disconnect = 0;
			connect = 0;
			bt_line = 0;		
			pointer = 0;
			return 0;//�ǽ�ֹ���ݣ����Կ���
		}			
	}
	
	return 0;//�ǽ�ֹ���ݣ����Կ���
}

//����λ��BootLoader����
void _System_Reset_(u8 uart_recv)
{
	static u8 res_buf[5];
	static u8 res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//���ܵ���λ������ĸ�λ�ַ���reset����ִ��������λ
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			NVIC_SystemReset();//����������λ����λ��ִ�� BootLoader ����
		}
	}
}


