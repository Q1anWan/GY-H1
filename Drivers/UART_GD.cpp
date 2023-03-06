/*********************************************************************************
  *FileName:	UART_GD.cpp
  *Author:  	qianwan
  *Description: GD32 串口发送基类
  *Other:		使用宏定义开启DMA收发与DMA内存搬运
				//////////////////////////////////////////////////////////////////
			**	UART_USE_TX_DMA == 1时开启发送DMA
				
				DMA发送需要在接收DMA的DMAx_IRQHandler函数中调用Transmit_IRQ
				//////////////////////////////////////////////////////////////////
			**	UART_USE_RX_DMA == 1时开启接收DMA
				
				UART_TX_DMA_M2M == 0时为普通DMA接收模式
				普通接受使用CPU搬运数据，适合小于168字节的接收，以降低操作开销
				普通接受需要在相应串口的USARTx_IRQHandler函数中调用Recieve_IRQ
				
			**	UART_TX_DMA_M2M == 1时使能内存搬运接收
				内存搬运使用DMA搬运数据，适合大于168字节的接收以降低CPU使用率;内存搬运会引发约3us的数据接收延迟
				内存搬运接收需要在相应串口的USARTx_IRQHandler函数以及接收DMA的DMAx_IRQHandler函数中调用Recieve_IRQ
  
  *Version:  	1.2.1
  *Date:  		2023/03/06
  *Description: 取消对Delay.c的依赖
  
  *Version:  	1.2
  *Date:  		2022/11/24
  *Description: 创建项目
**********************************************************************************/
#include "UART_GD.h"

/*! 
 *  @brief      串口阻塞发送函数
 *  @param  	传入发送数组指针、数据数目
  *  @return  	发送正常返回0 | 发送异常返回USART_FLAG_BSY
 */
uint16_t cUART::Transmit(uint8_t *DT,uint16_t num,uint32_t OVT)
{
	uint16_t wait_cnt = 0;
	for (uint16_t i = 0; i < num; i++)
	{
		wait_cnt = 0;
		
		while (!usart_flag_get(this->UART, USART_FLAG_TBE))
		{	this->Delay();
			if (wait_cnt++ > UART_TIME_OVER_TIME)return USART_FLAG_BSY;
		}
		usart_data_transmit(this->UART,DT[i]);
	}
	wait_cnt = 0;
	while (!usart_flag_get(this->UART, USART_FLAG_TBE))
	{
		this->Delay();
		if(wait_cnt++ > OVT)return USART_FLAG_BSY;
	}
	return 0;
}

/*! 
 *  @brief      串口阻塞发送函数
 *  @param  	传入发送数组指针、数据数目、超时us
 *  @return  	发送正常返回0 | 发送异常返回USART_FLAG_RT
 */
uint16_t cUART::Recieve(uint8_t *DT,uint16_t num,uint32_t OVT)
{
	uint16_t wait_cnt = 0;
	uint16_t i = 0;

	/*数据循环接受*/
	for (; i < num; i++)
	{
		/*等待数据并记录超时*/
		while(!usart_flag_get(this->UART, USART_FLAG_RBNE))
		{ 
			this->Delay();
			if(wait_cnt++>OVT)
			{
				this->Recieve_Length = i;
				return USART_FLAG_RT;
			}
		}
		
		/*RBNE时接收数据，此操作将清楚RBNE标志*/
		DT[i]=usart_data_receive(this->UART);
		wait_cnt = 0;
	}
	
	this->Recieve_Length = i;
	return 0;
}
#if (UART_USE_TX_DMA==1)
/*! 
 *  @brief   	UART DMA发送函数
 *  @param  	发送数组指针，发送字节长度
 *  @return  	发送正常返回0 | 发送异常返回USART_FLAG_BSY
 *  @other  	需要在发送DMA的中断函数内调用本函数
 */
uint16_t cUART::Transmit_DMA(uint8_t *pData, uint16_t length)
{
	/*正在执行发送，返回串口忙*/
	if(!usart_flag_get(this->UART,USART_FLAG_TBE))
	{return USART_FLAG_BSY;}

	/*配置本次传输*/
	dma_interrupt_flag_clear(this->DMAt,this->DMA_CHt,DMA_INT_FLAG_FTF);
	dma_memory_address_config(this->DMAt,this->DMA_CHt,(uint32_t)pData);
	dma_transfer_number_config(this->DMAt,this->DMA_CHt,(uint32_t)length);
	/*使能传输*/
	dma_channel_enable(this->DMAt,this->DMA_CHt);
	return 0;
}
#endif

#if (UART_USE_RX_DMA==1)
/*! 
 *  @brief   	UART DMA接收函数
 *  @param  	接收数组指针，最大接收长度
 *  @return  	正常设置接收返回0 | 无法正常设置接收返回USART_FLAG_BSY
 *  @other  	普通接受时需要在接收USART的中断函数内调用本函数
				内存搬运接受时需要在接收USART的中断函数以及接收DMA的中断函数中调用本函数
 */
uint16_t cUART::Recieve_DMA(uint8_t *pData, uint16_t length)
{
	/*正在执行接收，返回串口忙*/
	if(usart_flag_get(this->UART,USART_FLAG_RBNE))
	{return USART_FLAG_BSY;}
	
	/*长度设置*/
	this->Target_Length = (length>RX_BUF_LEN)?RX_BUF_LEN:length;
	this->pRX_Data = pData;
	/*关闭通道*/
	dma_channel_disable(this->DMAr,this->DMA_CHr);
	/*清除错误标志和IDLE标志*/
	usart_flag_get(this->UART,USART_FLAG_IDLE);
	usart_interrupt_flag_clear(this->UART,USART_INT_FLAG_IDLE);
	usart_data_receive(this->UART);
	/*开启空闲中断*/
	usart_interrupt_enable(this->UART,USART_INT_IDLE);
	/*配置本次传输*/
	#if (UART_TX_DMA_M2M==1)
	dma_periph_increase_disable(this->DMAr,this->DMA_CHr);
	dma_periph_address_config(this->DMAr,this->DMA_CHr,(uint32_t)&USART_DATA(this->UART));
	#endif
	dma_memory_address_config(this->DMAr,this->DMA_CHr,(uint32_t)this->pRX_BUF);
	dma_transfer_number_config(this->DMAr,this->DMA_CHr,RX_BUF_LEN);
	/*使能传输*/
	dma_channel_enable(this->DMAr,this->DMA_CHr);
	return 0;

}
#endif