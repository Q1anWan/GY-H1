/*********************************************************************************
  *FileName:	SPI_GD.cpp
  *Author:  	qianwan
  *Details: 	GD32 SPI控制 API
  
				使用DMA时必须使用带DMA参数的初始化函数！！
			**	SPI_USE_TX_DMA == 1时开启发送DMA
				DMA发送需要在发送DMA的DMAx_IRQHandler函数中调用IRQ_Tx
				//////////////////////////////////////////////////////////////////
			**	SPI_USE_RX_DMA == 1时开启接收DMA
				DMA接收需要在接收DMA的DMAx_IRQHandler函数中调用IRQ_Rx
  
			**	GD32版本驱动与STM32的差异在于：GD32版本SPI_Init无需重载，不使用的
				DMA直接传入空指针即可
				
  *Version:  	1.2.1
  *Date:  		2023/03/05
  *Other:		取消对Delay函数的依赖
 
  *Version:  	1.2
  *Date:  		2023/01/29
  *Other:		使用宏定义开启DMA收发
**********************************************************************************/
#include "SPI_GD.h"

uint8_t cSPI::SPI_ExchangeOneByte(uint8_t Data)
{
	uint16_t wait_cnt = 0;
	while(!spi_i2s_flag_get(this->SPI,SPI_FLAG_TBE))
	{
		if (wait_cnt++ > SPI_TIME_OVER_TIME)
		{return 0xff;}
	}
	spi_i2s_data_transmit(this->SPI,Data);
	while(!spi_i2s_flag_get(this->SPI,I2S_FLAG_RBNE))
	{
		if (wait_cnt++ > SPI_TIME_OVER_TIME)
		{return 0xff;}
	}
	return spi_i2s_data_receive(this->SPI);
}

#if (SPI_USE_TX_DMA==1)
uint8_t cSPI::Transmit_DMA(uint8_t *data,uint16_t num)
{
	/*正在执行发送或接收，返回SPI忙*/
	if(spi_i2s_flag_get(this->SPI,SPI_FLAG_TRANS))
	{return 1;}

	/*配置本次传输*/
	dma_interrupt_flag_clear(this->DMAt,this->DMA_CHt,DMA_INT_FLAG_FTF);
	dma_memory_address_config(this->DMAt,this->DMA_CHt,(uint32_t)data);
	dma_transfer_number_config(this->DMAt,this->DMA_CHt,(uint32_t)num);
	/*使能传输*/
	dma_channel_enable(this->DMAt,this->DMA_CHt);
	return 0;
}
uint8_t cSPI::IRQ_Tx(void)
{
	if(dma_interrupt_flag_get(this->DMAt,this->DMA_CHt,DMA_INT_FLAG_FTF))
	{
		dma_channel_disable(this->DMAt,this->DMA_CHt);
		dma_interrupt_flag_clear(this->DMAt,this->DMA_CHt,DMA_INT_FLAG_FTF);
		return 1;
	}
	return 0;
}
#endif
#if (SPI_USE_RX_DMA==1)
uint8_t cSPI::Receive_DMA(uint8_t *data,uint16_t num)
{
	/*正在执行发送或接收，返回SPI忙*/
	if(spi_i2s_flag_get(this->SPI,SPI_FLAG_TRANS))
	{return 1;}
	
	/*配置本次传输*/
	dma_interrupt_flag_clear(this->DMAr,this->DMA_CHr,DMA_INT_FLAG_FTF);
	dma_memory_address_config(this->DMAr,this->DMA_CHr,(uint32_t)data);
	dma_transfer_number_config(this->DMAr,this->DMA_CHr,(uint32_t)num);
	/*使能传输*/
	dma_channel_enable(this->DMAr,this->DMA_CHr);		
	return 0;
}
uint8_t cSPI::IRQ_Rx(void)
{
	if(dma_interrupt_flag_get(this->DMAr,this->DMA_CHr,DMA_INT_FLAG_FTF))
	{
		dma_channel_disable(this->DMAr,this->DMA_CHr);
		dma_interrupt_flag_clear(this->DMAr,this->DMA_CHr,DMA_INT_FLAG_FTF);
		return 1;
	}
	return 0;	
}
#endif

