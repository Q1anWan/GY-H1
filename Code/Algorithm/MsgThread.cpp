#include "MsgThread.h"
#include "Controller.h"
#include "CRC8.h"
#include <stdio.h>

/*程序串口发送控制*/
cMSG  *Msg;

usb_dev *my_usb_dev = RT_NULL;
/*串口进程控制指针*/
rt_thread_t UART_thread = RT_NULL;
/*发送完成控制信号量*/
rt_sem_t UART0_TxSem = RT_NULL;
/*接收完成控制信号量*/
rt_sem_t UART0_RxSem = RT_NULL;
/*发送控制互斥锁*/
rt_mutex_t UART0_TxMut = RT_NULL;

/*USB进程控制指针*/
rt_thread_t USBD_thread = RT_NULL;
/*USB中断控制信号量*/
rt_sem_t USBD_Sem = RT_NULL;
/*发送控制互斥锁*/
rt_mutex_t USB_TxMut = RT_NULL;
/*USBD接收缓冲区*/
uint8_t *USBD_rev_buf;

/*CAN进程控制指针*/
rt_thread_t CAN_thread = RT_NULL;
/*CAN中断控制信号量*/
rt_sem_t CAN_Sem = RT_NULL;

extern  rt_thread_t Config_thread;
extern rt_mailbox_t Config_mailbox;

/*
	串口控制进程
	控制串口消息的接收
	串口硬件资源的初始化在此进行，串口接收响应需要晚于硬件资源初始化执行
*/
void UARTThread(void* parameter)
{
	uint8_t IsHead = 0;//是否需要添加包头
	uint16_t TxLen = 0;//数据部分长度

	Msg = new cMSG; 
	Msg->UART_Init(USART0,DMA0,DMA_CH4,DMA0,DMA_CH3);
 
	for(;;)
	{
		/*接收进程处理*/
		rt_sem_take(UART0_RxSem,RT_WAITING_FOREVER);
		
		if((Msg->cUART::Recieve_Length == SYS_CONFIG_PACK_LEN)&&(Msg->UartRecBuf[0]==CMD_PACG_HEAD))//长度正确 包头正确
		{
			
			if(Msg->UartRecBuf[3]==cal_crc8_table(Msg->UartRecBuf,3))//CRC通过
			{
				if((Msg->UartRecBuf[1]!=0x00)||(Msg->UartRecBuf[2]!=0x04))
				{Msg->UartTx(Msg->UartRecBuf,4,2);}//回复信息		
				uint16_t mb_buf = (Msg->UartRecBuf[1]<<8) | (Msg->UartRecBuf[2]);
				rt_mb_send(Config_mailbox,(rt_ubase_t)mb_buf);
			}
		}
		Msg->Recieve_DMA(Msg->UartRecBuf,16);//再次进入接受模式
	}
}

void DMA0_Channel3_IRQHandler(void)
{
	if(Msg->Transmit_IRQ()){
	rt_sem_release(UART0_TxSem);
	}
}


void USBDThread(void* parameter)
{
	my_usb_dev = new usb_dev;
	usbd_init(my_usb_dev,&cdc_desc,&cdc_class);
	Msg->USB_COM = (usb_cdc_handler *)my_usb_dev->class_data[CDC_COM_INTERFACE];
	usbd_connect(my_usb_dev);
	#ifdef qwDbug
	/* wait for standard USB enumeration is finished */
    if(my_usb_dev->cur_status == USBD_CONNECTED){rt_kprintf("USB Connective");}
	#endif
	for(;;)
	{
		rt_sem_take(USBD_Sem,RT_WAITING_FOREVER);
		Msg->USB_COM = (usb_cdc_handler *)my_usb_dev->class_data[CDC_COM_INTERFACE];
		if(Msg->USB_COM->receive_length == SYS_CONFIG_PACK_LEN)//长度正确
		{
			if((Msg->USB_COM->data[3]==cal_crc8_table(Msg->USB_COM->data,3))&&Msg->USB_COM->data[0]==CMD_PACG_HEAD)//CRC通过 包头正确
			{	
				if((Msg->USB_COM->data[1]!=0x00)||(Msg->USB_COM->data[2]!=0x04))
				{Msg->USBTx(Msg->USB_COM->data,4,2);}//回复信息
				uint16_t mb_buf = (Msg->USB_COM->data[1]<<8) | (Msg->USB_COM->data[2]);
				rt_mb_send(Config_mailbox,(rt_ubase_t)mb_buf);
			}
		}	
	}
}

void USBD_LP_CAN0_RX0_IRQHandler(void)
{
	usbd_isr();
    
}

void USBD_WKUP_IRQHandler(void)
{
    exti_interrupt_flag_clear(EXTI_18);
}


void USART0_IRQHandler(void)
{
	if(Msg->Recieve_IRQ())
	{
		rt_sem_release(UART0_RxSem);
	}
}
void CANThread(void* parameter)
{
	Msg->CAN_RecMsg = new can_receive_message_struct;
	for(;;)
	{
		rt_sem_take(CAN_Sem,RT_WAITING_FOREVER);
	}
}

void CAN0_RX1_IRQHandler(void)
{
	can_message_receive(CAN0,CAN_FIFO1,Msg->CAN_RecMsg);
	rt_sem_release(CAN_Sem);
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
uint8_t cMSG::UartTx(uint8_t *pdata, uint16_t Length, rt_int32_t WaitTime)
{
	if(pdata==0){return 1;}
	/*直接使用DMA发送*/
	if(rt_mutex_take(UART0_TxMut,WaitTime)!=RT_EOK){return 1;}
	this->Transmit_DMA(pdata,Length);
	rt_sem_take(UART0_TxSem,RT_WAITING_FOREVER);
	rt_mutex_release(UART0_TxMut);
	return 0;
}
/*
	在上面函数的基础上，添加一个数据包头
*/
uint8_t cMSG::UartTx(uint8_t Head, uint8_t *pdata, uint16_t Length, rt_int32_t WaitTime)
{
	if(pdata==0){return 1;}
	/*直接使用DMA发送*/
	if(rt_mutex_take(UART0_TxMut,WaitTime)!=RT_EOK){return 1;}
	usart_data_transmit(USART0,Head);
	while (!usart_flag_get(USART0, USART_FLAG_TBE)){}
	this->Transmit_DMA(pdata,Length);
	rt_sem_take(UART0_TxSem,RT_WAITING_FOREVER);
	rt_mutex_release(UART0_TxMut);
	return 0;
}
uint8_t cMSG::USBTx(uint8_t *pdata, uint16_t Length, rt_int32_t WaitTime)
{
	if(pdata==0){return 1;}
	/*直接使用DMA发送*/
	if(rt_mutex_take(USB_TxMut,WaitTime)!=RT_EOK){return 1;}
	usbd_ep_send(my_usb_dev, CDC_IN_EP, pdata, Length);
	rt_mutex_release(USB_TxMut);
	return 0;
}
void cMSG::CANTx(uint16_t CAN_ID, uint8_t* data, uint8_t len)
{
	if(data==0){return;}
	else if(len==0){return;}
	len = (len>8)?8:len;
	
	can_trasnmit_message_struct transmit_message;
	transmit_message.tx_sfid = CAN_ID;
    transmit_message.tx_efid = 0x00;
    transmit_message.tx_ft	 = CAN_FT_DATA;
    transmit_message.tx_ff	 = CAN_FF_STANDARD;
    transmit_message.tx_dlen = len;
	memcpy(transmit_message.tx_data,data,len);
	
	can_message_transmit(CAN0, &transmit_message);
}
void cMSG::Printf(const char * format, ...)
{
	char* buf = (char*)rt_malloc(128);

	va_list args;
	va_start(args, format);
	uint8_t Len = vsnprintf(buf,128,format, args);
	va_end (args);
	this->UartTx((uint8_t*)buf,Len,5);
	rt_free(buf);
}

void rt_hw_console_output(const char *str)
{
	#ifdef qwDbug
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
	#else
	uint16_t i=0;
	while(*(str+i) != '\0'){i++;}
	Msg->UartTx((uint8_t *)str,i,RT_WAITING_FOREVER);
	#endif
}