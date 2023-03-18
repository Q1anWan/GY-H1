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
static rt_thread_t LEDCal_thread = RT_NULL;
static rt_thread_t LEDCAN_thread = RT_NULL;
cWS281x *LED;
uint8_t ColotOff = 0;
LEDColor_t Color={0};
uint8_t Color_buf[10]={0};


static void Test1Thread(void* parameter);
static rt_thread_t Test1_thread = RT_NULL;
static void Test2Thread(void* parameter);
static rt_thread_t Test2_thread = RT_NULL;
static void Test3Thread(void* parameter);
static rt_thread_t Test3_thread = RT_NULL;
static void Test4Thread(void* parameter);
static rt_thread_t Test4_thread = RT_NULL;

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
extern void IMUHeatThread(void* parameter);
extern rt_thread_t IMU_thread;
extern rt_thread_t IMUSlaver_thread;
extern rt_thread_t IMUHeat_thread;
extern rt_sem_t IMU_INT1Sem;	
extern rt_sem_t IMU_INT2Sem;

uint8_t dataXXX[128]={0x01,0x01,0x01,0x01};
uint8_t dataYYY[128]={0x02,0x02,0x02,0x02};
uint8_t dataZZZ[128]={0x03,0x03,0x03,0x03};
uint8_t dataUUU[128]={0x04,0x04,0x04,0x04};

uint8_t TxStatue[4];

int main(void)
{	
	LEDCal_thread =                          				/* 线程控制块指针 */
	rt_thread_create( 					"LEDCal",           /* 线程名字 */
										LEDCalculateThread, /* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										384,                /* 线程栈大小 */
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
	rt_thread_create( 					"UART",             /* 线程名字 */
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
										256,              	/* 线程栈大小 */
										2,                  /* 线程的优先级 */
										5);                 /* 线程时间片 */	

	Config_mailbox =
	rt_mb_create(						"Config_Mb",		/*邮箱名称*/
										8,					/*邮箱容量*/
										RT_IPC_FLAG_FIFO);	/*邮箱方式*/
										

	IMU_thread =                          					/* 线程控制块指针 */
	rt_thread_create( 					"IMU",              /* 线程名字 */
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
										1,                  /* 线程的优先级 */
										1);                 /* 线程时间片 */
										
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
										768,                /* 线程栈大小 */
										2,                  /* 线程的优先级 */
										1);                 /* 线程时间片 */
	
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
	
	Test3_thread =                          				/* 线程控制块指针 */
	rt_thread_create( 					"Test3",            /* 线程名字 */
										Test3Thread,   		/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										128,                /* 线程栈大小 */
										5,                  /* 线程的优先级 */
										5);               	/* 线程时间片 */
	
	Test4_thread =                          				/* 线程控制块指针 */
	rt_thread_create( 					"Test4",            /* 线程名字 */
										Test4Thread,   		/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										128,                /* 线程栈大小 */
										5,                  /* 线程的优先级 */
										5);                	/* 线程时间片 */
										
	/* 启动线程，开启调度 */
	rt_thread_startup(UART_thread);
	#ifdef qwDbug									
	rt_kprintf("\n\nUART_thread  = %d\n",UART_thread);rt_thread_delay(2);				
	#endif
	rt_thread_startup(IMU_thread);
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
	rt_thread_startup(Test1_thread);
	rt_thread_startup(Test2_thread);
	return 0;
}

static void LEDCalculateThread(void* parameter)
{	
	rt_tick_t ticks;
	float VFie  = PI/256;
	float VTheta = PI/384;
	
	float Theta = 0;
	float Fie = PI/3;
	uint16_t CubeHalfHeight = 127;
	
	LED = new cWS281x;
	LED->cSPI::SPI_Init(SPI2,GPIOB,GPIO_PIN_6,DMA1,DMA_CH1);
	LED->Init(Color_buf);
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
static void Test3Thread(void* parameter)
{
	rt_thread_delay(1000);

	for(;;)
	{	
		Msg->UartTx(0xAA,dataZZZ,12,RT_WAITING_FOREVER);
		rt_thread_delay(10);
	}
}
static void Test4Thread(void* parameter)
{
	rt_thread_delay(1000);

	for(;;)
	{	
		Msg->UartTx(dataUUU,12,RT_WAITING_FOREVER);
		rt_thread_delay(10);
	}
}

void DMA1_Channel1_IRQHandler(void)
{
	LED->cSPI::IRQ_Tx();
}