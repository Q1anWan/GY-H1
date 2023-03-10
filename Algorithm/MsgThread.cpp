#include "MsgThread.h"

/*程序串口发送控制*/
cMSG  *MSG_TX;
/*进程控制指针*/
rt_thread_t UART_thread = RT_NULL;
/*发送空闲控制信号量*/
rt_sem_t UART0_TxSem = RT_NULL;
/*发送消息队列指针*/
struct rt_messagequeue UART0_TxMSG;

/*
	串口发送进程
	使用了消息队列的方式管理硬件资源，支持发送紧急消息
	串口硬件资源的初始化在此进行，串口接收响应需要晚于硬件资源初始化执行
*/
void UARTThread(void* parameter)
{
	/*	CFGBuf
		发送配置信息缓冲区 长度为3 
		Pack[0]最高为表示是否包含包头
		Pack[0] 0~7 与 Pack[1] 0~8构成数据长度
		Pack[1]在有包头时表示包头，无包头时信息无意义
	*/
	static uint8_t RecBuf[UART_MSG_CFG_LEN+UART_MSG_MAX_LEN];
	uint8_t IsHead = 0;//是否需要添加包头
	uint16_t TxLen = 0;//数据部分长度
	rt_tick_t Ticks;
	/*初始化硬件资源*/
	MSG_TX	= new cMSG;
	MSG_TX->UART_Init(USART0,DMA0,DMA_CH4,DMA0,DMA_CH3);
	for(;;)
	{
//		/*UART硬件就绪*/
//		rt_sem_take(UART0_TxSem,300);
		/*获取发送数据包*/
		rt_mq_recv(&UART0_TxMSG,RecBuf,UART_MSG_CFG_LEN+UART_MSG_MAX_LEN,RT_WAITING_FOREVER);
		
		MSG_TX->TxTestBuf = 0;
		
		Ticks = rt_tick_get();
		rt_enter_critical();
		/*锁死直接发送功能*/
		MSG_TX->DirTxFlag &= 0b10;
		rt_exit_critical();
		
		
		/*计算发送部分长度*/
		TxLen = RecBuf[0]<<8 | RecBuf[1];
		/*判断是否需要添加包头*/
		IsHead = RecBuf[0]&0x80;
		
		rt_enter_critical();
		/*串口忙标志*/
		MSG_TX->TxBSY = 1;
		/*减少发送数量*/
		MSG_TX->InBufNum--;
		rt_exit_critical();
		
		if(IsHead)
		{
			/*调用硬件发送*/
			Test0 = MSG_TX->Transmit_DMA(RecBuf+2,TxLen);
			while(!MSG_TX->TxTestBuf)
			{
				rt_hw_us_delay(100);
			}
		
		}
		else
		{
			Test0 = MSG_TX->Transmit_DMA(RecBuf+3,TxLen);
			while(!MSG_TX->TxTestBuf)
			{
				rt_hw_us_delay(100);
			}
		}
		
		TxBuf++;

	} 
}

void DMA0_Channel3_IRQHandler(void)
{
	if(MSG_TX->Transmit_IRQ()){
//	
//	/*重置直接发送标志*/
//	MSG_TX->DirTxFlag = 0;
//	
//	/*选择性释放直接发送*/
//	if(MSG_TX->InBufNum == 0){MSG_TX->DirTxFlag = 0b01;}
//	
	/*释放硬件就绪信号量*/
//	rt_sem_release(UART0_TxSem);
	MSG_TX->TxTestBuf = 1;
	/*串口忙标志*/
	MSG_TX->TxBSY = 0;
	}
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
/*
	常规消息发送进程,无包头
	将按照FIFO逻辑响应
	Len:注意 最大为 Min(32768,UART_MSG_MAX_LEN)
*/
uint8_t cMSG::MSGTx(uint8_t *pdata, uint16_t Length)
{
	uint8_t DirFlag = 0;
	if(pdata==0){return 1;}
	if(Length>UART_MSG_MAX_LEN){return 1;}
	
//	rt_enter_critical();
//	if(MSG_TX->DirTxFlag==0b01)
//	{
//		MSG_TX->DirTxFlag = 0b10;
//		DirFlag = 1;
//	}
//	rt_exit_critical();
//	
//	if(DirFlag==1)
//	{
//		/*清除UART硬件就绪标志*/
//		rt_sem_take(UART0_TxSem,RT_WAITING_NO);
//		/*调用硬件发送*/
//		MSG_TX->Transmit_DMA(pdata,Length);
//		TxDir++;
//	}
//	else	
//	{
		rt_enter_critical();
		MSG_TX->InBufNum ++;
		rt_exit_critical();
		uint8_t *buf = (uint8_t *)rt_malloc(UART_MSG_CFG_LEN+UART_MSG_MAX_LEN);
		
		buf[0] = Length>>8;
		buf[1] = Length&0xFF;
		
		rt_memcpy(buf+3,pdata,Length);

		if(rt_mq_send(&UART0_TxMSG,buf,UART_MSG_CFG_LEN+UART_MSG_MAX_LEN)!=-RT_EOK)
		{
			rt_free(buf);
			return 1;
		}
		
		rt_free(buf);
//	}
	

	TxTask++;
	return 0;
}

/*
	常规消息发送进程,有包头
	将按照FIFO逻辑响应
	Len:注意 最大为 Min(32768,UART_MSG_MAX_LEN)
*/
uint8_t cMSG::MSGTx(uint8_t Head, uint8_t *pdata, uint16_t Length)
{
	if(pdata==0){return 1;}
	if(Length>UART_MSG_MAX_LEN){return 1;}
	
	uint8_t *buf = (uint8_t *)rt_malloc(UART_MSG_CFG_LEN+UART_MSG_MAX_LEN);
	
	uint8_t CFGBuf[3]={0};
	CFGBuf[0] = Length>>8;
	CFGBuf[0] |=  0x80;
	CFGBuf[1] = Length&0xFF;
	CFGBuf[2] = Head;
	
	rt_memcpy(buf+3,pdata,Length);
	if(rt_mq_send(&UART0_TxMSG,buf,UART_MSG_CFG_LEN+UART_MSG_MAX_LEN)==-RT_EFULL)
	{rt_free(buf);return 1;}
	
	rt_free(buf);
	TxTask++;
	return 0;
}

/*
	紧急消息发送进程,无包头
	消息将被立即送到发送队首位置
	Len:注意 最大为 Min(32768,UART_MSG_MAX_LEN)
*/
uint8_t cMSG::MSGUrgentTx(uint8_t *pdata, uint16_t Length)
{
	if(pdata==0){return 1;}
	if(Length>UART_MSG_MAX_LEN){return 1;}
	
	uint8_t *buf = (uint8_t *)rt_malloc(UART_MSG_CFG_LEN+UART_MSG_MAX_LEN);
	
	buf[0] = Length>>8;
	buf[1] = Length&0xFF;
	
	memcpy(buf+3,pdata,Length);
	if(rt_mq_urgent(&UART0_TxMSG,buf,UART_MSG_CFG_LEN+UART_MSG_MAX_LEN)==-RT_EFULL)
	{rt_free(buf);return 1;}
	
	rt_free(buf);
	return 0;
}

/*
	紧急消息发送进程,有包头
	消息将被立即送到发送队首位置
	Len:注意 最大为 Min(32768,UART_MSG_MAX_LEN)
*/
uint8_t cMSG::MSGUrgentTx(uint8_t Head, uint8_t *pdata, uint16_t Length)
{
	if(pdata==0){return 1;}
	if(Length>UART_MSG_MAX_LEN){return 1;}
	
	uint8_t *buf = (uint8_t *)rt_malloc(UART_MSG_CFG_LEN+UART_MSG_MAX_LEN);
	
	uint8_t CFGBuf[3]={0};
	CFGBuf[0] = Length>>8;
	CFGBuf[0] |=  0x80;
	CFGBuf[1] = Length&0xFF;
	CFGBuf[2] = Head;
	
	memcpy(buf+3,pdata,Length);
	if(rt_mq_urgent(&UART0_TxMSG,buf,UART_MSG_CFG_LEN+UART_MSG_MAX_LEN)==-RT_EFULL)
	{rt_free(buf);return 1;}
	
	rt_free(buf);
	return 0;
}