#include "main.h"

#include "arm_math.h"
#include "WS281x.h"
#include "MsgThread.h"
#include "Controller.h"
#include "IMU.h"

#include "QCSLite.h"

uint16_t TxNum = 0;
uint16_t TxDir = 0;
uint16_t TxBuf= 0;
uint16_t TxTask = 0;
uint16_t InStackNum = 0;
uint8_t Test0 = 0;
uint8_t Test1 = 0;
uint8_t Test2 = 0;
uint8_t Test3 = 0;


static void LEDCalculateThread(void* parameter);
static void LEDCANThread(void* parameter);
rt_thread_t LEDCal_thread = RT_NULL;
rt_thread_t LEDCAN_thread = RT_NULL;
extern cWS281x *LED;
uint8_t ColotOff = 0;

static void Test1Thread(void* parameter);
static rt_thread_t Test1_thread = RT_NULL;
static void Test2Thread(void* parameter);
static rt_thread_t Test2_thread = RT_NULL;

static void KeyThread(void* parameter);
static rt_thread_t Key_thread = RT_NULL;
static rt_mailbox_t Key_mailbox;
static void KeyActionThread(void* parameter);
static rt_thread_t KeyAction_thread = RT_NULL;

extern void UARTThread(void* parameter);
extern rt_thread_t UART_thread;
extern rt_mutex_t UART0_TxMut;
extern rt_sem_t UART0_TxSem;	
extern rt_sem_t UART0_RxSem;

extern void USBDThread(void* parameter);
extern rt_thread_t USBD_thread;
extern rt_mutex_t USB_TxMut;
extern rt_sem_t USBD_Sem;

extern void CANThread(void* parameter);
extern rt_thread_t CAN_thread;
extern rt_sem_t CAN_Sem;

extern void ConfigThread(void* parameter);
extern rt_thread_t Config_thread;
extern rt_mailbox_t Config_mailbox;

extern void DataOutputThread(void* parameter);
extern rt_thread_t DataOutput_thread;

extern void IMUThread(void* parameter);
extern void IMU2Thread(void* parameter);
extern void IMUAHRSThread(void* parameter);
extern void IMUHeatThread(void* parameter);
extern rt_thread_t IMU_thread;
extern rt_thread_t IMUSlaver_thread;
extern rt_thread_t IMUAHRS_thread;
extern rt_thread_t IMUHeat_thread;
extern rt_sem_t IMU_INT1Sem;	
extern rt_sem_t IMU_INT2Sem;


uint8_t TxStatue[4];

int main(void)
{	
	LEDCal_thread =                          				/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"LEDCal",           /* �߳����� */
										LEDCalculateThread, /* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										368,                /* �߳�ջ��С */
										16,                 /* �̵߳����ȼ� */
										20);                /* �߳�ʱ��Ƭ */
	
	LEDCAN_thread =                          				/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"LEDCAN",           /* �߳����� */
										LEDCANThread,	    /* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										128,                /* �߳�ջ��С */
										15,                 /* �̵߳����ȼ� */
										20);                /* �߳�ʱ��Ƭ */	
	
	UART_thread =                          					/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"UARTRead",         /* �߳����� */
										UARTThread,   		/* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										768,                /* �߳�ջ��С */
										10,                  /* �̵߳����ȼ� */
										5);                 /* �߳�ʱ��Ƭ */
	
	UART0_TxMut =
	rt_mutex_create(					"UART0_TxMut",		/* ������������ */
										RT_IPC_FLAG_FIFO);	/* �������ı�־λ */

	UART0_TxSem	=
	rt_sem_create(						"UART0_TxSem",		/* �ź��������� */
										0,					/* ��ʼ����ֵ */
										RT_IPC_FLAG_FIFO);	/* �ź����ı�־λ */
	
	UART0_RxSem	=
	rt_sem_create(						"UART0_RxSem",		/* �ź��������� */
										1,					/* ��ʼ����ֵ */
										RT_IPC_FLAG_FIFO);	/* �ź����ı�־λ */
														
	Config_thread =                          				/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"Config",           /* �߳����� */
										ConfigThread,   	/* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										512,              	/* �߳�ջ��С */
										10,                  /* �̵߳����ȼ� */
										5);                 /* �߳�ʱ��Ƭ */	

	Config_mailbox =
	rt_mb_create(						"Config_Mb",		/*��������*/
										8,					/*��������*/
										RT_IPC_FLAG_FIFO);	/*���䷽ʽ*/
										

	IMU_thread =                          					/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"IMURead",          /* �߳����� */
										IMUThread,   		/* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										768,                /* �߳�ջ��С */
										1,                  /* �̵߳����ȼ� */
										10);                /* �߳�ʱ��Ƭ */	

	IMUSlaver_thread =                          			/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"IMUSlaver",        /* �߳����� */
										IMU2Thread,   		/* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										256,                /* �߳�ջ��С */
										2,                  /* �̵߳����ȼ� */
										1);                 /* �߳�ʱ��Ƭ */

	IMUAHRS_thread =                          				/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"IMUAHRS",        	/* �߳����� */
										IMUAHRSThread,   	/* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										768,                /* �߳�ջ��С */
										1,                  /* �̵߳����ȼ� */
										5);                 /* �߳�ʱ��Ƭ */
										
	IMUHeat_thread =                          				/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"IMUHeat",        	/* �߳����� */
										IMUHeatThread,   	/* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										364,                /* �߳�ջ��С */
										4,                  /* �̵߳����ȼ� */
										1);                 /* �߳�ʱ��Ƭ */
										
	IMU_INT1Sem	=
	rt_sem_create(						"IMU_INT1Sem",		/* �ź��������� */
										0,					/* ��ʼ����ֵ */
										RT_IPC_FLAG_FIFO);	/* �ź����ı�־λ */
										
	IMU_INT2Sem	=
	rt_sem_create(						"IMU_INT2Sem",		/* �ź��������� */
										0,					/* ��ʼ����ֵ */
										RT_IPC_FLAG_FIFO);	/* �ź����ı�־λ */								
	
	DataOutput_thread =                          			/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"DataOuput",      	/* �߳����� */
										DataOutputThread,   /* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										1024,                /* �߳�ջ��С */
										2,                  /* �̵߳����ȼ� */
										5);                 /* �߳�ʱ��Ƭ */
	
	Key_thread =                          					/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"Key",				/* �߳����� */
										KeyThread,   		/* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										512,                /* �߳�ջ��С */
										5,                  /* �̵߳����ȼ� */
										5);             	/* �߳�ʱ��Ƭ */
	
	Key_mailbox =
	rt_mb_create(						"Key_Mb",			/*��������*/
										4,					/*��������*/
										RT_IPC_FLAG_FIFO);	/*���䷽ʽ*/
	
	KeyAction_thread =                          			/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"KeyAction",		/* �߳����� */
										KeyActionThread,   	/* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										512,                /* �߳�ջ��С */
										4,                  /* �̵߳����ȼ� */
										5);             	/* �߳�ʱ��Ƭ */
										
	/* �����̣߳��������� */
	rt_thread_startup(UART_thread);
	#ifdef qwDbug									
	rt_kprintf("\n\nUART_thread  = %d\n",UART_thread);rt_thread_delay(2);				
	#endif
	
	rt_thread_startup(Config_thread);
	rt_thread_startup(IMU_thread);
	rt_thread_startup(IMUAHRS_thread);
	rt_thread_startup(IMUHeat_thread);
	
	rt_thread_delay(5);

	
	if(qCtr->OTSel==1)//����CAN
	{
		CAN_thread =                          					/* �߳̿��ƿ�ָ�� */
		rt_thread_create( 					"CAN",            	/* �߳����� */
											CANThread,   		/* �߳���ں��� */
											RT_NULL,            /* �߳���ں������� */
											256,              	/* �߳�ջ��С */
											3,                  /* �̵߳����ȼ� */
											5);                 /* �߳�ʱ��Ƭ */

		CAN_Sem	=
		rt_sem_create(						"CAN_Sem",			/* �ź��������� */
											0,					/* ��ʼ����ֵ */
											RT_IPC_FLAG_FIFO);	/* �ź����ı�־λ */
		rt_thread_startup(CAN_thread);
		rt_thread_startup(LEDCAN_thread);
		#ifdef qwDbug									
		rt_kprintf("\nCAN Enable\n");				
		#endif
	}
	else//����USB
	{	
		USBD_thread =                          					/* �߳̿��ƿ�ָ�� */
		rt_thread_create( 					"USBD",             /* �߳����� */
											USBDThread,   		/* �߳���ں��� */
											RT_NULL,            /* �߳���ں������� */
											512,              	/* �߳�ջ��С */
											3,                  /* �̵߳����ȼ� */
											5);                 /* �߳�ʱ��Ƭ */

		USBD_Sem	=
		rt_sem_create(						"USBD_Sem",			/* �ź��������� */
											0,					/* ��ʼ����ֵ */
											RT_IPC_FLAG_FIFO);	/* �ź����ı�־λ */

		USB_TxMut =
		rt_mutex_create(					"USB_TxMut",		/* ������������ */
											RT_IPC_FLAG_FIFO);	/* �������ı�־λ */
		rt_thread_startup(USBD_thread);
		#ifdef qwDbug									
		rt_kprintf("\nUSB Enable\n");				
		#endif
	}
					
	rt_thread_startup(LEDCal_thread);					
	rt_thread_startup(DataOutput_thread);
	rt_thread_startup(Key_thread);
	rt_thread_startup(KeyAction_thread);
	#ifdef qwDbug
	Test1_thread =                          				/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"Test1",            /* �߳����� */
										Test1Thread,   		/* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										512,                /* �߳�ջ��С */
										5,                  /* �̵߳����ȼ� */
										5);             	/* �߳�ʱ��Ƭ */
	
	Test2_thread =                          				/* �߳̿��ƿ�ָ�� */
	rt_thread_create( 					"Test2",            /* �߳����� */
										Test2Thread,   		/* �߳���ں��� */
										RT_NULL,            /* �߳���ں������� */
										768,                /* �߳�ջ��С */
										5,                  /* �̵߳����ȼ� */
										5);            		/* �߳�ʱ��Ƭ */
	
	rt_thread_startup(Test1_thread);
	rt_thread_startup(Test2_thread);
	#endif
	return 0;
}

static void LEDCalculateThread(void* parameter)
{	
	rt_tick_t ticks;	
	LEDColor_t Color={0};

	float VFie  = PI/256;
	float VTheta = PI/384;
	
	float Theta = 0;
	float Fie = PI/3;
	uint16_t CubeHalfHeight = 30;
	
	LED->LED_UpdateDMA(&Color,1);
	for(;;)
	{	
		ticks = rt_tick_get();
	
		Theta += VTheta;
		Fie	  += VFie;
		
		if(Theta>2*PI){Theta-=(2*PI);}
		else if(Theta<0){Theta+=(2*PI);}
		
		if(Fie>2*PI){Fie-=(2*PI);}
		else if(Fie<0){Fie+=(2*PI);}
		
		if(qCtr->OTSel==0)
		{
			Color.GRB[0] = CubeHalfHeight*(arm_sin_f32(Theta)+1);
			Color.GRB[1] = CubeHalfHeight*(arm_cos_f32(Theta)+1);
			Color.GRB[2] = CubeHalfHeight*(arm_cos_f32(Fie)+1);
		}
		else
		{
			Color.GRB[0] = CubeHalfHeight*(arm_sin_f32(Theta)*arm_cos_f32(Fie)+1);
			Color.GRB[1] = CubeHalfHeight*(arm_sin_f32(Theta)*arm_sin_f32(Fie)+1);
			Color.GRB[2] = CubeHalfHeight*(arm_cos_f32(Theta)+1);
		}

		
		if(ColotOff)
		{
			Color.GRB[0] *= 0.3;
			Color.GRB[1] *= 0.3;
			Color.GRB[2] *= 0.3;
		}

		LED->LED_UpdateDMA(&Color,1);
		rt_thread_delay_until(&ticks,8);
	}
} 

static void LEDCANThread(void* parameter)
{
	rt_thread_delay(1000);
	rt_tick_t ticks;
	uint8_t BlinkTim = 1 + ((qCtr->CAN_ID - 0x300)>>4);
	for(;;)
	{
		ticks = rt_tick_get();
		for(uint8_t i=0;i<BlinkTim;i++)
		{
			ColotOff = 1;
			rt_thread_delay(100);
			ColotOff = 0;
			rt_thread_delay(200);
		}
		rt_thread_delay_until(&ticks,5000);
	}
}

static void KeyThread(void* parameter)
{
	rt_tick_t ticker;
	volatile uint32_t TxBuf;
	uint8_t Last_Key = 1;
	for(;;)
	{
		ticker = rt_tick_get();
		if(Last_Key != gpio_input_bit_get(GPIOB,GPIO_PIN_4))
		{	
			rt_thread_delay(13);
			TxBuf = rt_tick_get();
			if(gpio_input_bit_get(GPIOB,GPIO_PIN_4))//������0
			{
				TxBuf &= (uint32_t)0x7FFFFFFFU;
				Last_Key = 1;
			}
			else//���´�1
			{
				TxBuf |= (uint32_t)0x80000000U;
				Last_Key = 0;
			}
			rt_mb_send(Key_mailbox,TxBuf);
		}
		rt_thread_delay_until(&ticker,15);
	}
}
static void KeyActionThread(void* parameter)
{
	uint32_t Key = 0;
	uint32_t Key_Last = 0;
	uint32_t Time = 0;
	uint32_t Time_Last = 0;
	uint8_t ModSelMode = 0; //0:������ģʽ 1:����л�ģʽ 2:ID�л�ģʽ
	LEDColor_t Color={0};
	for(;;)
	{
		if(ModSelMode==0)	
		{
			rt_mb_recv(Key_mailbox,(rt_ubase_t*)&Key,RT_WAITING_FOREVER);
			Time = Key&0x7FFFF;
			Key  = (Key>>31)&0x01;
			if(Key_Last==1 && Key==0)
			{
				if(Time - Time_Last > 1500)
				{
					/*��������ģʽ*/
					rt_mb_send(Config_mailbox,0x0100);
					ModSelMode = 1;
					LED->LED_UpdateDMA(&Color,1);
					rt_thread_delay(100);
				}
				else if((Time - Time_Last <1000)&qCtr->OTSel)//CANģʽ�²����л�ID
				{
					/*��������ģʽ*/
					rt_mb_send(Config_mailbox,0x0100);
					ModSelMode = 2;
					LED->LED_UpdateDMA(&Color,1);
					rt_thread_delay(100);
				}
			}
			Time_Last = Time;
			Key_Last = Key;
		}
		
		if(ModSelMode==1)	
		{	
			if(qCtr->OTSel)
			{
				//��ǰΪCANģʽ
				Color.GRB[0] = 0;Color.GRB[1] = 0;Color.GRB[2] = 120;
				LED->LED_UpdateDMA(&Color,1);
			}
			else
			{
				//��ǰΪUSBģʽ
				Color.GRB[0] = 120;Color.GRB[1] = 0;Color.GRB[2] = 0;
				LED->LED_UpdateDMA(&Color,1);
			}
			uint8_t  ConfigMod = 0;
			Time_Last= rt_tick_get();
			Time = rt_tick_get();
			Key_Last = 1;//ȷ����һ�ε����Ч
			while(Time - Time_Last<5000)
			{
				Time = rt_tick_get();
				if(rt_mb_recv(Key_mailbox,(rt_ubase_t*)&Key,1)==RT_EOK)
				{

					Key  = (Key>>31)&0x01;
		
					if(Key_Last==1 && Key==0)
					{
						ConfigMod = ConfigMod?0:1;
						if(ConfigMod)
						{
							//��ǰΪCANģʽ
							Color.GRB[0] = 0;Color.GRB[1] = 0;Color.GRB[2] = 120;
							LED->LED_UpdateDMA(&Color,1);
						}
						else
						{
							//��ǰΪUSBģʽ
							Color.GRB[0] = 120;Color.GRB[1] = 0;Color.GRB[2] = 0;
							LED->LED_UpdateDMA(&Color,1);
						}
					}
					
					Time_Last = rt_tick_get() - 5;
					Key_Last = Key;
				}
			}
			
			if(ConfigMod){rt_mb_send(Config_mailbox,0x0201);}
			else{rt_mb_send(Config_mailbox,0x0200);}
			rt_mb_send(Config_mailbox,0x0101);//�˳�����ģʽ
		}
		else if(ModSelMode==2)
		{
			uint32_t IDSet=(uint32_t)0x0400U;
			Time_Last= rt_tick_get();
			Time = rt_tick_get();
			
			Key_Last = 1;//ȷ����һ�ε����Ч
			while(Time - Time_Last<5000)
			{
				Time = rt_tick_get();
				if(rt_mb_recv(Key_mailbox,(rt_ubase_t*)&Key,1)==RT_EOK)
				{

					Key  = (Key>>31)&0x01;
		
					if(Key_Last==1 && Key==0)
					{
						/*���������ã��������Ǽ���*/
						IDSet<0x040A?IDSet++:IDSet=0x040A;
						Color.GRB[0] = 50;Color.GRB[1] = 50;Color.GRB[2] = 50;
						LED->LED_UpdateDMA(&Color,1);
						rt_thread_delay(5);
						Color.GRB[0] = 0;Color.GRB[1] = 0;Color.GRB[2] = 0;
						LED->LED_UpdateDMA(&Color,1);
						rt_thread_delay(1);
					}
					
					Time_Last = rt_tick_get()-10;
					Key_Last = Key;
				}
			}
			if(IDSet==0x0400){rt_mb_send(Config_mailbox,IDSet);}
			else{rt_mb_send(Config_mailbox,IDSet-1);}
			rt_thread_delay(5);
			rt_mb_send(Config_mailbox,0x0101);//�˳�����ģʽ
		}
	}
}

static void Test1Thread(void* parameter)
{
	rt_thread_delay(1000);
	for(;;)
	{			
		while(gpio_input_bit_get(GPIOB,GPIO_PIN_4))rt_thread_delay(10);
		#ifdef qwDbug
		list_thread();
		#endif
		rt_thread_delay(1000);
	}
}
static void Test2Thread(void* parameter)
{
	rt_thread_delay(2000);
	rt_tick_t ticker;
	float Angel[3];
	while(IMU->Temperature<41.0f){rt_thread_delay(100);}
	rt_thread_delay(1000);
	for(;;)
	{	
		ticker = rt_tick_get();
		QCS.Euler(IMU->Q,Angel);		
////		Msg->Printf("GYRO=%d %d %d\n",IMU->Gyro[0],IMU->Gyro[1],IMU->Gyro[2]);rt_thread_delay(1);
//		Msg->Printf("Accel\nX=%f\nY=%f\nZ=%f\n",IMU->AccelCorrected[0],IMU->AccelCorrected[1],IMU->AccelCorrected[2]);rt_thread_delay(1);
		Msg->Printf("\nRoll=%f\nPitch=%f\nYaw=%f\n",Angel[0]*57.2957795f,Angel[1]*57.2957795f,Angel[2]*57.2957795f);
//		Msg->Printf("Tem=%f\n",IMU->Temperature);
		rt_thread_delay_until(&ticker,4);
	}
}

void DMA1_Channel1_IRQHandler(void)
{
	LED->cSPI::IRQ_Tx();
}