#include "MsgThread.h"

/*串口控制类*/
cUART *myUART0;
/*程序串口发送控制*/
cMSG MSG_UART0;
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
	uint8_t CFGBuf[UART_MSG_CFG_LEN]={0};
	uint8_t IsHead = 0;//是否需要添加包头
	
	/*	TxBuf
		发送数据信息缓冲区 长度为数据部分长度+1
	*/
	uint8_t TxBuf[UART_MSG_MAX_LEN+1]={0};
	uint16_t TxLen = 0;//数据部分长度

	/*初始化硬件资源*/
	myUART0 = new cUART;
	myUART0->UART_Init(USART0,DMA0,DMA_CH4,DMA0,DMA_CH3);
	for(;;)
	{
		/*UART硬件就绪*/
		rt_sem_take(UART0_TxSem,RT_WAITING_FOREVER);
		/*获取发送配置信息*/
		rt_mq_recv(&UART0_TxMSG,CFGBuf,UART_MSG_CFG_LEN,RT_WAITING_FOREVER);
		/*计算发送部分长度*/
		TxLen = CFGBuf[0]<<8 | CFGBuf[1];
		/*判断是否需要添加包头*/
		IsHead = CFGBuf[0]&0x80;
		if(IsHead){TxBuf[0]=CFGBuf[2];}
		/*获取数据部分,根据有无包头做数据偏移*/
		if(rt_mq_recv(&UART0_TxMSG,TxBuf+IsHead,TxLen,RT_WAITING_FOREVER)==RT_EOK)
		{
			/*调用硬件发送*/
			myUART0->Transmit_DMA(TxBuf,TxLen+IsHead);
		}
		else/*如果超时，说明消息队列已经失去同步，尝试恢复队列*/
		{
			rt_mq_delete(&UART0_TxMSG);//need to write
		}
	} 
}

void DMA0_Channel3_IRQHandler(void)
{
	if(myUART0->Transmit_IRQ())
	{
		rt_sem_release(UART0_TxSem);
	}

}
/*
	常规消息发送进程,无包头
	将按照FIFO逻辑响应
	Len:注意 最大为 Min(32768,UART_MSG_MAX_LEN)
*/
uint8_t cMSG::MSGTx(uint8_t *pdata, uint16_t Length)
{
	if(pdata==NULL){return 1;}
	if(Length>UART_MSG_MAX_LEN){return 1;}
	
	uint8_t CFGBuf[3]={0};
	CFGBuf[0] = Length>>8;
	CFGBuf[1] = Length&0xFF;
	
	rt_enter_critical();
	rt_mq_send(&UART0_TxMSG,CFGBuf,UART_MSG_CFG_LEN);
	rt_mq_send(&UART0_TxMSG,pdata,Length);
	rt_exit_critical();
	
	return 0;
}

/*
	常规消息发送进程,有包头
	将按照FIFO逻辑响应
	Len:注意 最大为 Min(32768,UART_MSG_MAX_LEN)
*/
uint8_t cMSG::MSGTx(uint8_t Head, uint8_t *pdata, uint16_t Length)
{
	if(pdata==NULL){return 1;}
	if(Length>UART_MSG_MAX_LEN){return 1;}
	
	uint8_t CFGBuf[3]={0};
	CFGBuf[0] = Length>>8;
	CFGBuf[0] |=  0x80;
	CFGBuf[1] = Length&0xFF;
	CFGBuf[2] = Head;
	
	rt_enter_critical();
	rt_mq_send(&UART0_TxMSG,CFGBuf,UART_MSG_CFG_LEN);
	rt_mq_send(&UART0_TxMSG,pdata,Length);
	rt_exit_critical();
	
	return 0;
}

/*
	紧急消息发送进程,无包头
	消息将被立即送到发送队首位置
	Len:注意 最大为 Min(32768,UART_MSG_MAX_LEN)
*/
uint8_t cMSG::MSGUrgentTx(uint8_t *pdata, uint16_t Length)
{
	if(pdata==NULL){return 1;}
	if(Length>UART_MSG_MAX_LEN){return 1;}
	
	uint8_t CFGBuf[3]={0};
	CFGBuf[0] = Length>>8;
	CFGBuf[1] = Length&0xFF;
	
	rt_enter_critical();
	rt_mq_urgent(&UART0_TxMSG,pdata,Length);
	rt_mq_urgent(&UART0_TxMSG,CFGBuf,UART_MSG_CFG_LEN);
	rt_exit_critical();
	
	return 0;
}

/*
	紧急消息发送进程,有包头
	消息将被立即送到发送队首位置
	Len:注意 最大为 Min(32768,UART_MSG_MAX_LEN)
*/
uint8_t cMSG::MSGUrgentTx(uint8_t Head, uint8_t *pdata, uint16_t Length)
{
	if(pdata==NULL){return 1;}
	if(Length>UART_MSG_MAX_LEN){return 1;}
	
	uint8_t CFGBuf[3]={0};
	CFGBuf[0] = Length>>8;
	CFGBuf[0] |=  0x80;
	CFGBuf[1] = Length&0xFF;
	CFGBuf[2] = Head;
	
	rt_enter_critical();
	rt_mq_send(&UART0_TxMSG,pdata,Length);
	rt_mq_send(&UART0_TxMSG,CFGBuf,UART_MSG_CFG_LEN);
	rt_exit_critical();
	
	return 0;
}