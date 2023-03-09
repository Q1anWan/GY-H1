#include "MsgThread.h"

extern uint16_t TxNum;
extern uint16_t InStackNum;
/*串口控制类*/
cUART *myUART0;
/*程序串口发送控制*/
cMSG MSG_TX;
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

	/*初始化硬件资源*/
	myUART0 = new cUART;
	myUART0->UART_Init(USART0,DMA0,DMA_CH4,DMA0,DMA_CH3);
	for(;;)
	{
		/*UART硬件就绪*/
		rt_sem_take(UART0_TxSem,3);
		/*获取发送数据包*/
		rt_mq_recv(&UART0_TxMSG,RecBuf,UART_MSG_CFG_LEN+UART_MSG_MAX_LEN,RT_WAITING_FOREVER);
		/*计算发送部分长度*/
		TxLen = RecBuf[0]<<8 | RecBuf[1];
		/*判断是否需要添加包头*/
		IsHead = RecBuf[0]&0x80;
		if(IsHead)
		{
			/*调用硬件发送*/
			myUART0->Transmit_DMA(RecBuf+2,TxLen);
		}
		else
		{
			myUART0->Transmit_DMA(RecBuf+3,TxLen);
		}
		InStackNum --;
		TxNum++;
	} 
}

void DMA0_Channel3_IRQHandler(void)
{
	myUART0->Transmit_IRQ();
	rt_sem_release(UART0_TxSem);
}
void rt_hw_console_output(const char *str)
{
	if(UART_thread==RT_NULL){
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
	}
	else
	{
		uint16_t i=0;
		while(*(str+i) != '\0'){i++;}
		MSG_TX.MSGTx((uint8_t *)str,i);
	}
}
/*
	常规消息发送进程,无包头
	将按照FIFO逻辑响应
	Len:注意 最大为 Min(32768,UART_MSG_MAX_LEN)
*/
uint8_t cMSG::MSGTx(uint8_t *pdata, uint16_t Length)
{
	if(pdata==0){return 1;}
	if(Length>UART_MSG_MAX_LEN){return 1;}
	
	
	
	uint8_t *buf = (uint8_t *)rt_malloc(UART_MSG_CFG_LEN+UART_MSG_MAX_LEN);
	
	buf[0] = Length>>8;
	buf[1] = Length&0xFF;
	
	memcpy(buf+3,pdata,Length);
	if(rt_mq_send(&UART0_TxMSG,buf,UART_MSG_CFG_LEN+UART_MSG_MAX_LEN)==-RT_EFULL)
	{rt_free(buf);return 1;}
	
	rt_free(buf);
	InStackNum++;
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
	
	memcpy(buf+3,pdata,Length);
	if(rt_mq_send(&UART0_TxMSG,buf,UART_MSG_CFG_LEN+UART_MSG_MAX_LEN)==-RT_EFULL)
	{rt_free(buf);return 1;}
	
	rt_free(buf);
	InStackNum++;
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
	InStackNum++;
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
	InStackNum++;
	return 0;
}