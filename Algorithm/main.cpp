#include "main.h"

#include "arm_math.h"
#include "WS281x.h"
#include "MsgThread.h"

uint16_t TxNum = 0;
uint16_t InStackNum = 0;

cWS281x LED;



static rt_thread_t LED_thread = RT_NULL;
static void LEDThread(void* parameter);

static rt_thread_t Test1_thread = RT_NULL;
static void Test1Thread(void* parameter);

extern rt_thread_t UART_thread;
extern void UARTThread(void* parameter);
	
extern rt_sem_t UART0_TxSem;	

extern struct rt_messagequeue UART0_TxMSG;
static rt_uint8_t UART0_TxMSG_POOL[(UART_MSG_MAX_LEN+UART_MSG_CFG_LEN)*16];

static uint8_t dataXXX[128]={0x01,0x02,0x03,0x01};


int main(void)
{	
	LED_thread =                          					/* 线程控制块指针 */
	rt_thread_create( 					"LED",             	/* 线程名字 */
										LEDThread,   		/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										256,                /* 线程栈大小 */
										3,                  /* 线程的优先级 */
										20);                /* 线程时间片 */
	
	Test1_thread =                          				/* 线程控制块指针 */
	rt_thread_create( 					"Test1",            /* 线程名字 */
										Test1Thread,   		/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										256,                /* 线程栈大小 */
										5,                  /* 线程的优先级 */
										20);                /* 线程时间片 */

	UART_thread =                          					/* 线程控制块指针 */
	rt_thread_create( 					"UART",             /* 线程名字 */
										UARTThread,   		/* 线程入口函数 */
										RT_NULL,            /* 线程入口函数参数 */
										512,                /* 线程栈大小 */
										1,                  /* 线程的优先级 */
										20);                /* 线程时间片 */
	
	UART0_TxSem	=
	rt_sem_create(						"UART0_TxSem",		/*信号量的名称*/
										1,					/*初始化的值*/
										RT_IPC_FLAG_FIFO);	/*信号量的标志位*/
	
	rt_mq_init(
										&UART0_TxMSG,						/*消息控制指针*/
										"UART0_TxMSG",						/*消息名字*/
										&UART0_TxMSG_POOL[0],				/*内存池*/
										UART_MSG_MAX_LEN+UART_MSG_CFG_LEN,	/*每个消息的长度*/
										sizeof(UART0_TxMSG_POOL),			/*内存池大小*/
										RT_IPC_FLAG_FIFO);					/*先入先出*/
										


										
	/* 启动线程，开启调度 */
	rt_thread_startup(LED_thread);
	rt_thread_startup(UART_thread);
	rt_thread_startup(Test1_thread);
	return 0;
}


static void LEDThread(void* parameter)
{	rt_thread_delay(100);
	LEDColor_t Color={0};
	uint8_t Color_buf[10]={0};
	rt_tick_t ticks;

	float VFie  = PI/127;
	float VTheta = PI/163;
	
	float Theta = PI/7;
	float Fie = 0;
	uint16_t CubeHalfHeight = 40;
	
	LED.cSPI::SPI_Init(SPI2,GPIOB,GPIO_PIN_6,DMA1,DMA_CH1);
	LED.Init(Color_buf);
	LED.LED_UpdateDMA(&Color,1);
	for(;;)
	{	
		ticks = rt_tick_get();
	
		Theta += VTheta;
		Fie	  += VFie;
		
		if(Theta>2*PI){Theta-=(2*PI);}
		else if(Theta<0){Theta+=(2*PI);}
		
		if(Fie>2*PI){Fie-=(2*PI);}
		else if(Fie<0){Fie+=(2*PI);}
		
		Color.GRB[0] = CubeHalfHeight*(arm_sin_f32(Fie)+1);
		Color.GRB[1] = CubeHalfHeight*(arm_sin_f32(Theta)+1);
		Color.GRB[2] = CubeHalfHeight*(arm_cos_f32(Theta)+1);
		
		LED.LED_UpdateDMA(&Color,1); 
		rt_thread_delay_until(&ticks,10);
	}
}

static void Test1Thread(void* parameter)
{
	rt_thread_delay(100);

	for(;;)
	{	
//		rt_kprintf("12345\n");
//		MSG_TX.MSGTx(dataXXX,40);
//		MSG_TX.MSGTx(dataXXX,128);
//		MSG_TX.MSGTx(dataXXX,128);
		rt_thread_delay(10);
	}
}

void DMA1_Channel1_IRQHandler(void)
{
	LED.cSPI::IRQ_Tx();
}