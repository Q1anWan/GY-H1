#include "MsgThread.h"

/*程序串口发送控制*/
cMSG  *MSG_TX;
/*进程控制指针*/
rt_thread_t UART_thread = RT_NULL;
/*发送完成控制信号量*/
rt_sem_t UART0_TxSem = RT_NULL;
/*接收完成控制信号量*/
rt_sem_t UART0_RxSem = RT_NULL;
/*发送控制互斥锁*/
rt_mutex_t UART0_TxMut = RT_NULL;

/*
	串口控制进程
	控制串口消息的接收
	串口硬件资源的初始化在此进行，串口接收响应需要晚于硬件资源初始化执行
*/
void UARTThread(void* parameter)
{
	uint8_t IsHead = 0;//是否需要添加包头
	uint16_t TxLen = 0;//数据部分长度

	MSG_TX = new cMSG;
	MSG_TX->UART_Init(USART0,DMA0,DMA_CH4,DMA0,DMA_CH3);
	for(;;)
	{
		/*等待接收完成*/
		rt_sem_take(UART0_RxSem,RT_WAITING_FOREVER);
	}
}

void DMA0_Channel3_IRQHandler(void)
{
	if(MSG_TX->Transmit_IRQ()){
	rt_sem_release(UART0_TxSem);
	}
}

/*
	根据发送状态决定压入缓冲或是直接发送
	直接发送时:
	函数使用互斥锁保护串口资源
	使用信号量判断发送是否完成
	函数将在直接发送完成后解锁返回
	压入缓冲时:
	使用消息队列作为缓冲区
*/
uint8_t cMSG::MSGTx(uint8_t *pdata, uint16_t Length)
{
	if(pdata==0){return 1;}
	/*直接使用DMA发送*/
	rt_mutex_take(UART0_TxMut,RT_WAITING_NO);
	this->Transmit_DMA(pdata,Length);
	rt_sem_take(UART0_TxSem,RT_WAITING_FOREVER);
	rt_mutex_release(UART0_TxMut);
	return 0;
}
/*
	在上面函数的基础上，添加一个数据包头
*/
uint8_t cMSG::MSGTx(uint8_t Head, uint8_t *pdata, uint16_t Length)
{
	if(pdata==0){return 1;}
	/*直接使用DMA发送*/
	rt_mutex_take(UART0_TxMut,RT_WAITING_NO);
	usart_data_transmit(USART0,Head);
	while (!usart_flag_get(USART0, USART_FLAG_TBE)){}
	this->Transmit_DMA(pdata,Length);
	rt_sem_take(UART0_TxSem,RT_WAITING_FOREVER);
	rt_mutex_release(UART0_TxMut);
	return 0;
}

void rt_hw_console_output(const char *str)
{
	#ifndef qwDbug
	if(UART_thread==RT_NULL){
	#endif
    /* 进入临界段 */
	rt_enter_critical();
	while(*str != '\0')
	{
		if( *str=='\n' )
		{	
			while (!usart_flag_get(USART0, USART_FLAG_TBE)){}
			usart_data_transmit(USART0,'\r');
			while (!usart_flag_get(USART0, USART_FLAG_TBE)){}
		}
		
		while (!usart_flag_get(USART0, USART_FLAG_TBE)){}
		usart_data_transmit(USART0,*str++);
		while (!usart_flag_get(USART0, USART_FLAG_TBE)){}
	}
	/* 退出临界段 */
	rt_exit_critical();
	#ifndef qwDbug
	}
	else
	{
		uint16_t i=0;
		while(*(str+i) != '\0'){i++;}
		MSG_TX->MSGTx((uint8_t *)str,i);
	}
	#endif
}