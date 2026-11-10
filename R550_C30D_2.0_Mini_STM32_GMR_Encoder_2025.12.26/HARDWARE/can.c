#include "can.h"
#include "system.h"
#include "can_test.h"
/**************************************************************************
Function: CAN1 initialization
Input   : tsjw��Resynchronize the jump time unit, Scope: 1 ~ 3;
 			    tbs2��Time unit of time period 2, range :1~8;
 			    tbs1��Time unit of time period 1, range :1~16;
 			    brp ��Baud rate divider, range :1 to 1024;(We're actually going to add 1, which is 1 to 1024) tq=(brp)*tpclk1
 			    mode��0, normal mode;1. Loop mode;
Output  : 0- Initialization successful;Other - initialization failed
Note: none of the entry parameters (except mode) can be 0
�������ܣ�CAN1��ʼ��
��ڲ�����tsjw������ͬ����Ծʱ�䵥Ԫ����Χ:1~3;
 			    tbs2��ʱ���2��ʱ�䵥Ԫ����Χ:1~8;
 			    tbs1��ʱ���1��ʱ�䵥Ԫ����Χ:1~16;
 			    brp �������ʷ�Ƶ������Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
 			    mode��0,��ͨģʽ;1,�ػ�ģʽ;
����  ֵ��0-��ʼ���ɹ�; ����-��ʼ��ʧ��
ע�⣺��ڲ���(����mode)������Ϊ0
������/Baud rate=Fpclk1/((tbs1+tbs2+tsjw)*brp)��Fpclk1Ϊ42M

**************************************************************************/
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	u16 i=0;
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
	tsjw-=1; //Subtract 1 before setting //�ȼ�ȥ1.����������
	tbs2-=1;
	tbs1-=1;
	brp-=1;

	//ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��PORTBʱ��	                   											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	//��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��PB8 PB9
//	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); //GPIOB8����ΪCAN1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); //GPIOB9����ΪCAN1

	CAN1->MCR=0x0000;	//Exit sleep mode (setting all bits to 0 at the same time) //�˳�˯��ģʽ(ͬʱ��������λΪ0)
	CAN1->MCR|=1<<0;	//Request CAN1 to enter initialization mode //����CAN1�����ʼ��ģʽ
	while((CAN1->MSR&1<<0)==0)
	{
		i++;
		if(i>100)return 2; //Failed to enter initialization mode //�����ʼ��ģʽʧ��
	}
	//Non-time triggered communication mode
	//��ʱ�䴥��ͨ��ģʽ
	CAN1->MCR|=0<<7;
  //Software automatic offline management	
	//�����Զ����߹���
	CAN1->MCR|=0<<6;	
	//Sleep mode is awakened by software (clear CAN1- >;
  //˯��ģʽͨ����������(���CAN1->MCR��SLEEPλ)	
	CAN1->MCR|=0<<5;
	//Disallow automatic message transmission
  //��ֹ�����Զ�����	
	CAN1->MCR|=1<<4;
	//Messages are not locked, the new overwrites the old
  //���Ĳ�����,�µĸ��Ǿɵ�	
	CAN1->MCR|=0<<3;
	//The priority is determined by the message identifier	
  //���ȼ��ɱ��ı�ʶ������  
	CAN1->MCR|=0<<2;
  //Clear the original Settings	
  //���ԭ��������	
	CAN1->BTR=0x00000000; 
	//Mode set to 0, normal mode;1. Loop mode;
	//ģʽ���� 0,��ͨģʽ;1,�ػ�ģʽ;
	CAN1->BTR|=mode<<30;
  //Resynchronization jump width (TSJW) is TSJW +1 time unit	
	//����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ
	CAN1->BTR|=tsjw<<24; 
  //Tbs2= Tbs2 +1 time unit	
	//Tbs2=tbs2+1��ʱ�䵥λ
	CAN1->BTR|=tbs2<<20; 
	//Tbs1= Tbs1 +1 time unit	
  //Tbs1=tbs1+1��ʱ�䵥λ	
	CAN1->BTR|=tbs1<<16;
	//Frequency division coefficient (Fdiv) is brp +1, boulder rate: Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
  //��Ƶϵ��(Fdiv)Ϊbrp+1��������:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
	CAN1->BTR|=brp<<0;  
  //Request CAN1 to exit initialization mode	
  //����CAN1�˳���ʼ��ģʽ			
	CAN1->MCR&=~(1<<0);	  
	while((CAN1->MSR&1<<0)==1)
	{
		i++;
		if(i>0XFFF0)return 3; //Failed to exit initialization mode //�˳���ʼ��ģʽʧ��
	}
	/*** Filter initialization || ��������ʼ�� ***/
	//Filter groups work in initialization mode
	//�������鹤���ڳ�ʼ��ģʽ
	CAN1->FMR|=1<<0;	
	//Filter 0 is not active
  //������0������	
	CAN1->FA1R&=~(1<<0);	
	//The filter bit width is 32 bits
  //������λ��Ϊ32λ	
	CAN1->FS1R|=1<<0; 
	//Filter 0 works in identifier masking bit mode
  //������0�����ڱ�ʶ������λģʽ	
	CAN1->FM1R|=0<<0;	
	//Filter 0 is associated with FIFO0
  //������0������FIFO0	
	CAN1->FFA1R|=0<<0;	
  //32 bit ID	
	//32λID
	CAN1->sFilterRegister[0].FR1=0X00000000;
	//32-bit MASK
	//32λMASK
	CAN1->sFilterRegister[0].FR2=0X00000000;
	//Activation filter 0
	//���������0
	CAN1->FA1R|=1<<0;	
	//Filter group enters normal mode
  //���������������ģʽ	
	CAN1->FMR&=0<<0;			

#if CAN1_RX0_INT_ENABLE
  //Enable to interrupt reception //ʹ���жϽ���
	
	//FIFO0��Ϣ�Һ��ж�����
	CAN1->IER|=1<<1;			
	
	//Configure CAN to receive interrupts
	//����CAN�����ж�
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  //Preemption priority	
	//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Son priority
	//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  //IRQ channel enablement
	//IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initializes the VIC register according to the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	NVIC_Init(&NVIC_InitStructure);	

  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
#endif
	return 0;
}   
/**************************************************************************
Function: CAN sends data
Input   : id:Standard ID(11 bits)/ Extended ID(11 bits +18 bits)
			    ide:0, standard frame;1, extension frames
			    rtr:0, data frame;1, remote frame
			    len:Length of data to be sent (fixed at 8 bytes, valid data is 6 bytes in time-triggered mode) 
			    *dat:Pointer to the data
Output  : 0~3, mailbox number. 0xFF, no valid mailbox
�������ܣ�CAN��������
��ڲ�����id:��׼ID(11λ)/��չID(11λ+18λ)	    
			    ide:0,��׼֡;1,��չ֡
			    rtr:0,����֡;1,Զ��֡
			    len:Ҫ���͵����ݳ���(�̶�Ϊ8���ֽ�,��ʱ�䴥��ģʽ��,��Ч����Ϊ6���ֽ�)
			    *dat:����ָ��.
����  ֵ��0~3,������.0XFF,����Ч����
**************************************************************************/
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
	u8 mbox;	  
	if(CAN1->TSR&(1<<26))mbox=0;		  //Mailbox 0 is empty //����0Ϊ��
	else if(CAN1->TSR&(1<<27))mbox=1;	//Mailbox 1 is empty //����1Ϊ��
	else if(CAN1->TSR&(1<<28))mbox=2;	//Mailbox 2 is empty //����2Ϊ��
	else return 0XFF;					        //No empty mailbox, cannot send //�޿�����,�޷����� 
	
	CAN1->sTxMailBox[mbox].TIR=0; //Clear the previous Settings //���֮ǰ������		
	if(ide==0) //The standard frame //��׼֡
	{
		id&=0x7ff; //Take the low 11 bit STDID //ȡ��11λstdid
		id<<=21;		  
	}else	//Extend the frame //��չ֡
	{
		id&=0X1FFFFFFF; //Take a low 32-bit extid //ȡ��32λextid
		id<<=3;									   
	}
	CAN1->sTxMailBox[mbox].TIR|=id;		 
	CAN1->sTxMailBox[mbox].TIR|=ide<<2;	  
	CAN1->sTxMailBox[mbox].TIR|=rtr<<1;
	len&=0X0F; //Get lower 4 bits //�õ�����λ
	CAN1->sTxMailBox[mbox].TDTR&=~(0X0000000F);
	CAN1->sTxMailBox[mbox].TDTR|=len;	//Set the DLC	//����DLC
	//The data to be sent is stored in the mailbox
	//���������ݴ�������
	CAN1->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
								((u32)dat[6]<<16)|
 								((u32)dat[5]<<8)|
								((u32)dat[4]));
	CAN1->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
								((u32)dat[2]<<16)|
 								((u32)dat[1]<<8)|
								((u32)dat[0]));
	CAN1->sTxMailBox[mbox].TIR|=1<<0; //Request to send mailbox data//��������������
	return mbox;
}
/**************************************************************************
Function: Get the send status
Input   : Mbox: mailbox number
Output  : 0, hang;0X05, send failed;0X07, successful transmission
�������ܣ���÷���״̬
��ڲ�����mbox��������
����  ֵ��0,����;0X05,����ʧ��;0X07,���ͳɹ�
**************************************************************************/
u8 CAN1_Tx_Staus(u8 mbox)
{	
	u8 sta=0;					    
	switch (mbox)
	{
		case 0: 
			sta |= CAN1->TSR&(1<<0);			   //RQCP0
			sta |= CAN1->TSR&(1<<1);			   //TXOK0
			sta |=((CAN1->TSR&(1<<26))>>24); //TME0
			break;
		case 1: 
			sta |= CAN1->TSR&(1<<8)>>8;		   //RQCP1
			sta |= CAN1->TSR&(1<<9)>>8;		   //TXOK1
			sta |=((CAN1->TSR&(1<<27))>>25); //TME1	   
			break;
		case 2: 
			sta |= CAN1->TSR&(1<<16)>>16;	   //RQCP2
			sta |= CAN1->TSR&(1<<17)>>16;	   //TXOK2
			sta |=((CAN1->TSR&(1<<28))>>26); //TME2
			break;
		default:
			sta=0X05; //Wrong email number, failed //����Ų���,ʧ��
		break;
	}
	return sta;
} 
/**************************************************************************
Function: Returns the number of packets received in FIFO0/FIFO1
Input   : Fifox: FIFO number (0, 1)
Output  : Number of packets in FIFO0/FIFO1
�������ܣ��õ���FIFO0/FIFO1�н��յ��ı��ĸ���
��ڲ�����fifox��FIFO��ţ�0��1��
����  ֵ��FIFO0/FIFO1�еı��ĸ���
**************************************************************************/
u8 CAN1_Msg_Pend(u8 fifox)
{
	if(fifox==0)return CAN1->RF0R&0x03; 
	else if(fifox==1)return CAN1->RF1R&0x03; 
	else return 0;
}
/**************************************************************************
Function: Receive data
Input   : fifox��Email
		    	id:Standard ID(11 bits)/ Extended ID(11 bits +18 bits)
			    ide:0, standard frame;1, extension frames 
			    rtr:0, data frame;1, remote frame
			    len:Length of data received (fixed at 8 bytes, valid at 6 bytes in time-triggered mode)
			    dat:Data cache
Output  : none
�������ܣ���������
��ڲ�����fifox�������
		    	id:��׼ID(11λ)/��չID(11λ+18λ)
			    ide:0,��׼֡;1,��չ֡
			    rtr:0,����֡;1,Զ��֡
			    len:���յ������ݳ���(�̶�Ϊ8���ֽ�,��ʱ�䴥��ģʽ��,��Ч����Ϊ6���ֽ�)
			    dat:���ݻ�����
����  ֵ���� 
**************************************************************************/
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{	   
	*ide=CAN1->sFIFOMailBox[fifox].RIR&0x04; //Gets the value of the identifier selection bit //�õ���ʶ��ѡ��λ��ֵ  
 	if(*ide==0) //Standard identifier //��׼��ʶ��
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>21;
	}else	     //Extended identifier //��չ��ʶ��
	{
		*id=CAN1->sFIFOMailBox[fifox].RIR>>3;
	}
	*rtr=CAN1->sFIFOMailBox[fifox].RIR&0x02;	//Gets the remote send request value //�õ�Զ�̷�������ֵ
	*len=CAN1->sFIFOMailBox[fifox].RDTR&0x0F; //Get the DLC //�õ�DLC
 	//*fmi=(CAN1->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF; //Get the FMI //�õ�FMI
	//Receive data //��������
	dat[0]=CAN1->sFIFOMailBox[fifox].RDLR&0XFF;
	dat[1]=(CAN1->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
	dat[2]=(CAN1->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
	dat[3]=(CAN1->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
	dat[4]=CAN1->sFIFOMailBox[fifox].RDHR&0XFF;
	dat[5]=(CAN1->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
	dat[6]=(CAN1->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
	dat[7]=(CAN1->sFIFOMailBox[fifox].RDHR>>24)&0XFF;    
  if(fifox==0)CAN1->RF0R|=0X20;      //Free the FIFO0 mailbox //�ͷ�FIFO0����
	else if(fifox==1)CAN1->RF1R|=0X20; //Free the FIFO1 mailbox //�ͷ�FIFO1����	 
}
/**************************************************************************
Function: CAN receives interrupt service function, conditional compilation
Input   : none
Output  : none
�������ܣ�CAN�����жϷ���������������
��ڲ�������
����  ֵ���� 
**************************************************************************/
#if CAN1_RX0_INT_ENABLE	//Enable RX0 interrupt //ʹ��RX0�ж�
void CAN1_RX0_IRQHandler(void)
{
	u32 id;
	u8 ide,rtr,len;     
	u8 temp_rxbuf[8];
	u8 i;

 	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,temp_rxbuf);
    if (id == 0x301U) {
        CAN_Test_Parse(ide, rtr, len, temp_rxbuf);
        return;
    }
	
	//�س�װ��������
	if(id==0x12345678)
	{
		u8 check=0;
		for(i=0;i<8;i++) 
		{
			check += temp_rxbuf[i];
		}
		if( check==8 ) Get_Charging_HardWare=1;
	}
	else if(id==0x181)
	{
		float Vz;
		CAN_ON_Flag=1, PS2_ON_Flag=0,Remote_ON_Flag=0,APP_ON_Flag=0,Usart1_ON_Flag=0,Usart5_ON_Flag=0;
		command_lost_count=0;
		//Calculate the three-axis target velocity, unit: m/s
		//��������Ŀ���ٶȣ���λ��m/s
		Move_X=((float)((short)((temp_rxbuf[0]<<8)+(temp_rxbuf[1]))))/1000; 
		Move_Y=((float)((short)((temp_rxbuf[2]<<8)+(temp_rxbuf[3]))))/1000;
		Vz    =((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
		if(Car_Mode==Akm_Car)
		{
			Move_Z=Vz_to_Akm_Angle(Move_X, Vz);
		}
		else
		{
			Move_Z=((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
		}
	}
	
	//�Զ��س����
	else if(id==0x182)
	{
		charger_check=0;
		
		//�Զ��س�״̬����pid����ʹ�����Ӧ�ӿ�
		if(Allow_Recharge) Velocity_KP=600,Velocity_KI=600;
		else Velocity_KP=300,Velocity_KI=300;
		
		//Calculate the three-axis target velocity, unit: mm/s
		//��������Ŀ���ٶȣ���λ��mm/s
		Recharge_Red_Move_X=((float)((short)((temp_rxbuf[0]<<8)+(temp_rxbuf[1]))))/1000; 
		Recharge_Red_Move_Y=((float)((short)((temp_rxbuf[2]<<8)+(temp_rxbuf[3]))))/1000;
		Recharge_Red_Move_Z=((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
		
		Charging=temp_rxbuf[6]&1;		//���״̬��־λ
		L_A = (temp_rxbuf[6]>>5)&0x01;
		L_B = (temp_rxbuf[6]>>4)&0x01;
		R_B = (temp_rxbuf[6]>>3)&0x01;
		R_A = (temp_rxbuf[6]>>2)&0x01;
		RED_STATE = L_A + L_B + R_B + R_A;
		
		if(RED_STATE==0) Recharge_Red_Move_X = Recharge_Red_Move_Y = Recharge_Red_Move_Z = 0;
		if( Charging==1) Recharge_Red_Move_X = Recharge_Red_Move_Y = Recharge_Red_Move_Z = 0;
		
		//����������
		if(temp_rxbuf[7]>128)Charging_Current=-(256-temp_rxbuf[7])*30;
		else Charging_Current=(temp_rxbuf[7]*30);
	}
}
#endif

/**************************************************************************
Function: CAN1 sends a set of data (fixed format :ID 0X601, standard frame, data frame)
Input   : msg:Pointer to the data
    			len:Data length (up to 8)
Output  : 0, success, others, failure;
�������ܣ�CAN1����һ������(�̶���ʽ:IDΪ0X601,��׼֡,����֡)
��ڲ�����msg:����ָ��
    			len:���ݳ���(���Ϊ8)
����  ֵ��0,�ɹ�������,ʧ��;
**************************************************************************/
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(0X601,0,0,len,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending //�ȴ����ͽ���
	if(i>=0XFFF)return 1; //Send failure //����ʧ��
	return 0;	//Send a success //���ͳɹ�									
}
/**************************************************************************
Function: The CAN1 port receives data queries
Input   : Buf: The data cache
Output  : 0, number of data received, other, length of data received
�������ܣ�CAN1�ڽ������ݲ�ѯ
��ڲ�����buf:���ݻ�����
����  ֵ��0,�����ݱ��յ�������,���յ����ݳ���
**************************************************************************/
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
	u32 id;
	u8 ide,rtr,len; 
	if(CAN1_Msg_Pend(0)==0)return 0;			   //No data received, exit directly //û�н��յ�����,ֱ���˳� 	 
  	CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,buf); //Read the data //��ȡ����
    if(id!=0x12||ide!=0||rtr!=0)len=0;		 //Receive error //���մ���	   
	return len;	
}
/**************************************************************************
Function: Can1 sends a set of data tests
Input   : msg:Pointer to the data
			    len:Data length (up to 8)
Output  : 0, success, 1, failure
�������ܣ�CAN1����һ�����ݲ���
��ڲ�����msg:����ָ��
			    len:���ݳ���(���Ϊ8)
����  ֵ��0,�ɹ���1,ʧ��
**************************************************************************/
u8 CAN1_Send_MsgTEST(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
    mbox=CAN1_Tx_Msg(0X701,0,0,len,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending //�ȴ����ͽ���
	if(i>=0XFFF)return 1;	//Send failure //����ʧ��
	return 0;	//Send a success //���ͳɹ�
}
/**************************************************************************
Function: Sends an array to the given ID
Input   : id��ID no.
			    msg��The transmitted data pointer
Output  : 0, success, 1, failure
�������ܣ���������id����һ�����������
��ڲ�����id��ID��
			    msg������������ָ��
����  ֵ��0,�ɹ���1,ʧ��
**************************************************************************/
u8 CAN1_Send_Num(u32 id,u8* msg)
{
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(id,0,0,8,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending //�ȴ����ͽ���
	if(i>=0XFFF)return 1;	//Send failure //����ʧ��
	return 0;
}

u8 CAN1_Send_EXTid_Num(u32 id,u8* msg)
{
	u8 mbox;
	u16 i=0;	  	 						       
	mbox=CAN1_Tx_Msg(id,1,0,8,msg);   
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending //�ȴ����ͽ���
	if(i>=0XFFF)return 1;	//Send failure //����ʧ��
	return 0;
}
