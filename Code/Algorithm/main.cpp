#include "main.h"

#include "arm_math.h"
#include "WS281x.h"
#include "MsgThread.h"
#include "Controller.h"
#include "IMU.h"

uint16_t TxNum = 0;
uint16_t TxDir = 0;
uint16_t TxBuf= 0;
uint16_t TxTask = 0;
uint16_t InStackNum = 0;
uint8_t Test0 = 0;
uint8_t Test1 = 0;
uint8_t Test2 = 0;
uint8_t Test3 = 0;



extern void ConfigRead(void);

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
	LEDCal_thread =                          				/* 线程控制块指针 */
	rt_thread_create( 					"LEDCal",           /* 线程名字 */
										LEDCalculateThread, /* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										256,                /* 线程栈大小 */
										16,                 /* 线程的优先级 */
										20);                /* 线程时间片 */
	
	LEDCAN_thread =                          				/* 线程控制块指针 */
	rt_thread_create( 					"LEDCAN",           /* 线程名字 */
										LEDCANThread,	    /* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										128,                /* 线程栈大小 */
										15,                 /* 线程的优先级 */
										20);                /* 线程时间片 */	
	
	UART_thread =                          					/* 线程控制块指针 */
	rt_thread_create( 					"UARTRead",         /* 线程名字 */
										UARTThread,   		/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										768,                /* 线程栈大小 */
										3,                  /* 线程的优先级 */
										5);                 /* 线程时间片 */
	
	UART0_TxMut =
	rt_mutex_create(					"UART0_TxMut",		/* 互斥锁的名称 */
										RT_IPC_FLAG_FIFO);	/* 互斥锁的标志位 */

	UART0_TxSem	=
	rt_sem_create(						"UART0_TxSem",		/* 信号量的名称 */
										0,					/* 初始化的值 */
										RT_IPC_FLAG_FIFO);	/* 信号量的标志位 */
	
	UART0_RxSem	=
	rt_sem_create(						"UART0_RxSem",		/* 信号量的名称 */
										1,					/* 初始化的值 */
										RT_IPC_FLAG_FIFO);	/* 信号量的标志位 */
														
	Config_thread =                          				/* 线程控制块指针 */
	rt_thread_create( 					"Config",           /* 线程名字 */
										ConfigThread,   	/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										512,              	/* 线程栈大小 */
										1,                  /* 线程的优先级 */
										5);                 /* 线程时间片 */	

	Config_mailbox =
	rt_mb_create(						"Config_Mb",		/*邮箱名称*/
										8,					/*邮箱容量*/
										RT_IPC_FLAG_FIFO);	/*邮箱方式*/
										

	IMU_thread =                          					/* 线程控制块指针 */
	rt_thread_create( 					"IMURead",          /* 线程名字 */
										IMUThread,   		/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										512,                /* 线程栈大小 */
										1,                  /* 线程的优先级 */
										10);                /* 线程时间片 */	

	IMUSlaver_thread =                          			/* 线程控制块指针 */
	rt_thread_create( 					"IMUSlaver",        /* 线程名字 */
										IMU2Thread,   		/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										256,                /* 线程栈大小 */
										2,                  /* 线程的优先级 */
										1);                 /* 线程时间片 */

	IMUAHRS_thread =                          				/* 线程控制块指针 */
	rt_thread_create( 					"IMUAHRS",        	/* 线程名字 */
										IMUAHRSThread,   	/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										512,                /* 线程栈大小 */
										1,                  /* 线程的优先级 */
										5);                 /* 线程时间片 */
										
	IMUHeat_thread =                          				/* 线程控制块指针 */
	rt_thread_create( 					"IMUHeat",        	/* 线程名字 */
										IMUHeatThread,   	/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										364,                /* 线程栈大小 */
										4,                  /* 线程的优先级 */
										1);                 /* 线程时间片 */
										
	IMU_INT1Sem	=
	rt_sem_create(						"IMU_INT1Sem",		/* 信号量的名称 */
										0,					/* 初始化的值 */
										RT_IPC_FLAG_FIFO);	/* 信号量的标志位 */
										
	IMU_INT2Sem	=
	rt_sem_create(						"IMU_INT2Sem",		/* 信号量的名称 */
										0,					/* 初始化的值 */
										RT_IPC_FLAG_FIFO);	/* 信号量的标志位 */								
	
	DataOutput_thread =                          			/* 线程控制块指针 */
	rt_thread_create( 					"DataOuput",      	/* 线程名字 */
										DataOutputThread,   /* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										1024,                /* 线程栈大小 */
										2,                  /* 线程的优先级 */
										5);                 /* 线程时间片 */
	
	Key_thread =                          					/* 线程控制块指针 */
	rt_thread_create( 					"Key",				/* 线程名字 */
										KeyThread,   		/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										256,                /* 线程栈大小 */
										5,                  /* 线程的优先级 */
										5);             	/* 线程时间片 */
	
	Key_mailbox =
	rt_mb_create(						"Key_Mb",			/*邮箱名称*/
										4,					/*邮箱容量*/
										RT_IPC_FLAG_FIFO);	/*邮箱方式*/
	
	KeyAction_thread =                          			/* 线程控制块指针 */
	rt_thread_create( 					"KeyAction",		/* 线程名字 */
										KeyActionThread,   	/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										256,                /* 线程栈大小 */
										4,                  /* 线程的优先级 */
										5);             	/* 线程时间片 */
										
	/* 启动线程，开启调度 */
	rt_thread_startup(UART_thread);
	#ifdef qwDbug									
	rt_kprintf("\n\nUART_thread  = %d\n",UART_thread);rt_thread_delay(2);				
	#endif
	rt_thread_startup(IMU_thread);
	rt_thread_startup(IMUAHRS_thread);
	rt_thread_startup(IMUHeat_thread);
	rt_thread_startup(Config_thread);
	rt_thread_delay(5);
	ConfigRead();
	
	if(qCtr->OTSel==1)//启动CAN
	{
		CAN_thread =                          					/* 线程控制块指针 */
		rt_thread_create( 					"CAN",            	/* 线程名字 */
											CANThread,   		/* 线程入口函数 */
											RT_NULL,            /* 线程入口函数参数 */
											256,              	/* 线程栈大小 */
											3,                  /* 线程的优先级 */
											5);                 /* 线程时间片 */

		CAN_Sem	=
		rt_sem_create(						"CAN_Sem",			/* 信号量的名称 */
											0,					/* 初始化的值 */
											RT_IPC_FLAG_FIFO);	/* 信号量的标志位 */
		rt_thread_startup(CAN_thread);
		rt_thread_startup(LEDCAN_thread);
		#ifdef qwDbug									
		rt_kprintf("\nCAN Enable\n");				
		#endif
	}
	else//启动USB
	{	
		USBD_thread =                          					/* 线程控制块指针 */
		rt_thread_create( 					"USBD",             /* 线程名字 */
											USBDThread,   		/* 线程入口函数 */
											RT_NULL,            /* 线程入口函数参数 */
											512,              	/* 线程栈大小 */
											3,                  /* 线程的优先级 */
											5);                 /* 线程时间片 */

		USBD_Sem	=
		rt_sem_create(						"USBD_Sem",			/* 信号量的名称 */
											0,					/* 初始化的值 */
											RT_IPC_FLAG_FIFO);	/* 信号量的标志位 */

		USB_TxMut =
		rt_mutex_create(					"USB_TxMut",		/* 互斥锁的名称 */
											RT_IPC_FLAG_FIFO);	/* 互斥锁的标志位 */
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
	Test1_thread =                          				/* 线程控制块指针 */
	rt_thread_create( 					"Test1",            /* 线程名字 */
										Test1Thread,   		/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										512,                /* 线程栈大小 */
										5,                  /* 线程的优先级 */
										5);             	/* 线程时间片 */
	
	Test2_thread =                          				/* 线程控制块指针 */
	rt_thread_create( 					"Test2",            /* 线程名字 */
										Test2Thread,   		/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										768,                /* 线程栈大小 */
										5,                  /* 线程的优先级 */
										5);            		/* 线程时间片 */
	
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
			if(gpio_input_bit_get(GPIOB,GPIO_PIN_4))//不按传0
			{
				TxBuf &= (uint32_t)0x7FFFFFFFU;
				Last_Key = 1;
			}
			else//按下传1
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
	uint8_t ModSelMode = 0; //0:非配置模式 1:输出切换模式 2:ID切换模式
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
					/*进入配置模式*/
					rt_mb_send(Config_mailbox,0x0100);
					ModSelMode = 1;
					LED->LED_UpdateDMA(&Color,1);
					rt_thread_delay(100);
				}
				else if((Time - Time_Last <1000)&qCtr->OTSel)//CAN模式下才能切换ID
				{
					/*进入配置模式*/
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
				//当前为CAN模式
				Color.GRB[0] = 0;Color.GRB[1] = 0;Color.GRB[2] = 120;
				LED->LED_UpdateDMA(&Color,1);
			}
			else
			{
				//当前为USB模式
				Color.GRB[0] = 120;Color.GRB[1] = 0;Color.GRB[2] = 0;
				LED->LED_UpdateDMA(&Color,1);
			}
			uint8_t  ConfigMod = 0;
			Time_Last= rt_tick_get();
			Time = rt_tick_get();
			Key_Last = 1;//确保第一次点击有效
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
							//当前为CAN模式
							Color.GRB[0] = 0;Color.GRB[1] = 0;Color.GRB[2] = 120;
							LED->LED_UpdateDMA(&Color,1);
						}
						else
						{
							//当前为USB模式
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
			rt_mb_send(Config_mailbox,0x0101);//退出配置模式
		}
		else if(ModSelMode==2)
		{
			uint32_t IDSet=(uint32_t)0x0400U;
			Time_Last= rt_tick_get();
			Time = rt_tick_get();
			
			Key_Last = 1;//确保第一次点击有效
			while(Time - Time_Last<5000)
			{
				Time = rt_tick_get();
				if(rt_mb_recv(Key_mailbox,(rt_ubase_t*)&Key,1)==RT_EOK)
				{

					Key  = (Key>>31)&0x01;
		
					if(Key_Last==1 && Key==0)
					{
						/*所见即所得，按几下是几个*/
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
			rt_mb_send(Config_mailbox,0x0101);//退出配置模式
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

	for(;;)
	{	
		Msg->Printf("GYRO=%d %d %d\n",IMU->Gyro[0],IMU->Gyro[1],IMU->Gyro[2]);rt_thread_delay(1);
		Msg->Printf("Accel=%d %d %d\n",IMU->Accel[0],IMU->Accel[1],IMU->Accel[2]);rt_thread_delay(1);
		Msg->Printf("Tem=%f\n\n",IMU->Temperature);//rt_thread_delay(1);
		rt_thread_delay(18);
	}
}

void DMA1_Channel1_IRQHandler(void)
{
	LED->cSPI::IRQ_Tx();
}