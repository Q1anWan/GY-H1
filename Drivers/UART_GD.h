/*********************************************************************************
  *FileName:	UART_GD.h
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
#ifndef UART_GD_H
#define UART_GD_H
#include <main.h>
#include <string.h>

/*******开启DMA收发*****s***/
#define UART_USE_RX_DMA 1
#define UART_USE_TX_DMA 1
#define UART_TX_DMA_M2M 0
/********1开**0关**********/

/********其他参数**********/
//接收缓冲区长度 也是最大接受长度
#define RX_BUF_LEN 128
//每个Byte的超时时间
#define UART_TIME_OVER_TIME 500
/**************************/
#ifdef __cplusplus

class cUART
{
	protected:
	uint32_t UART;
	
	public:
	uint16_t Target_Length=0;
	uint16_t Recieve_Length=0;
	
	uint16_t Transmit(uint8_t *DT,uint16_t num,uint32_t OVT);
	uint16_t Recieve(uint8_t *DT,uint16_t num,uint32_t OVT);

	virtual inline void Delay(void)
	{__NOP();__NOP();__NOP();}
	
	/*没有使用DMA*/
	#if !(UART_USE_RX_DMA||UART_USE_TX_DMA)
	public:
	void UART_Init(uint32_t UART){this->UART = UART;usart_enable(this->UART);}
	#endif
	
	/*仅使用DMA发送*/
	#if (UART_USE_TX_DMA==1)&&(UART_USE_RX_DMA==0)
	protected:
	uint32_t DMAt;
	dma_channel_enum DMA_CHt;
	public:
	void UART_Init(uint32_t UART,uint32_t DMAt,dma_channel_enum DMA_CHt)
	{
		this->UART = UART;
		this->DMAt = DMAt;
		this->DMA_CHt = DMA_CHt;
		usart_enable(this->UART);
	}

	/*仅使用DMA接收*/
	#elif (UART_USE_TX_DMA==0)&&(UART_USE_RX_DMA==1)
	protected:
	uint32_t DMAr;
	dma_channel_enum DMA_CHr;
	uint8_t RX_BUF0[RX_BUF_LEN];
	uint8_t RX_BUF1[RX_BUF_LEN];
	uint8_t *pRX_Data;
	uint8_t *pRX_BUF = RX_BUF0;
	public:
	void UART_Init(uint32_t UART,uint32_t DMAr,dma_channel_enum DMA_CHr)
	{
		this->UART = UART;
		this->DMAr = DMAr;
		this->DMA_CHr = DMA_CHr;
		this->
		usart_enable(this->UART);
	}

	/*使用DMA收发*/
	#elif (UART_USE_TX_DMA==1)&&(UART_USE_RX_DMA==1)
	protected:
	uint32_t DMAt;
	dma_channel_enum DMA_CHt;
	uint32_t DMAr;
	dma_channel_enum DMA_CHr;
	uint8_t RX_BUF0[RX_BUF_LEN];
	uint8_t RX_BUF1[RX_BUF_LEN];
	uint8_t *pRX_Data;
	uint8_t *pRX_BUF = RX_BUF0;
	public:
	void UART_Init(uint32_t UART,uint32_t DMAr,dma_channel_enum DMA_CHr,uint32_t DMAt,dma_channel_enum DMA_CHt)
	{
		this->UART = UART;
		this->DMAr = DMAr;
		this->DMA_CHr = DMA_CHr;
		this->DMAt = DMAt;
		this->DMA_CHt = DMA_CHt;
		usart_enable(this->UART);
	}
	#endif
	
	#if (UART_USE_TX_DMA==1)
	uint16_t Transmit_DMA(uint8_t *pData, uint16_t length);
	inline uint8_t Transmit_IRQ(void)
	{
		if(dma_interrupt_flag_get(this->DMAt,this->DMA_CHt,DMA_INT_FLAG_FTF))
		{
			dma_channel_disable(this->DMAt,this->DMA_CHt);
			dma_interrupt_flag_clear(this->DMAt,this->DMA_CHt,DMA_INT_FLAG_G);
			return 1;
		}
		return 0;
	}
	#endif
	#if (UART_USE_RX_DMA==1)
	uint16_t Recieve_DMA(uint8_t *pData, uint16_t length);
	inline uint8_t Recieve_IRQ(void)
	{ 
		//溢出错误清楚
		usart_interrupt_flag_clear(this->UART,USART_INT_FLAG_RBNE_ORERR);
		if(usart_interrupt_flag_get(this->UART,USART_INT_FLAG_IDLE))
		{
			/*停止DMA传输*/
			dma_channel_disable(this->DMAr,this->DMA_CHr);
			/*关闭空闲中断*/
			usart_interrupt_disable(this->UART,USART_INT_IDLE);
			/*统计传输数量*/
			this->Recieve_Length = RX_BUF_LEN - dma_transfer_number_get(this->DMAr,this->DMA_CHr);
			
			#if (UART_TX_DMA_M2M==1)		
			/*设置M2M的DMA*/
			dma_periph_increase_enable(this->DMAr,this->DMA_CHr);
			dma_memory_to_memory_enable(this->DMAr,this->DMA_CHr);
			dma_periph_address_config(this->DMAr,this->DMA_CHr,(uint32_t)this->pRX_BUF);
			dma_memory_address_config(this->DMAr,this->DMA_CHr,(uint32_t)this->pRX_Data);
			dma_transfer_number_config(this->DMAr,this->DMA_CHr,(this->Recieve_Length>this->Target_Length)?this->Target_Length:this->Recieve_Length);//溢出保护
			/*DMA中断使能*/
			dma_interrupt_flag_clear(this->DMAr,this->DMA_CHr,DMA_INT_FLAG_FTF);
			dma_interrupt_enable(this->DMAr,this->DMA_CHr,DMA_INT_FTF);
			/*开始传输*/
			dma_channel_enable(this->DMAr,this->DMA_CHr);
			#else
			memcpy(this->pRX_Data,this->pRX_BUF,(this->Recieve_Length>this->Target_Length)?this->Target_Length:this->Recieve_Length);
			#endif

			/*双缓冲区交换*/
			this->pRX_BUF = (this->pRX_BUF==this->RX_BUF0)?this->RX_BUF1:this->RX_BUF0;
			/*清除IDLE中断标志*/
			usart_interrupt_flag_clear(this->UART,USART_INT_FLAG_IDLE);
			return 1;
		}
		#if (UART_TX_DMA_M2M==1)	
		else if(dma_interrupt_flag_get(this->DMAr,this->DMA_CHr,DMA_INT_FLAG_FTF))
		{
			/*停止DMA传输*/
			dma_channel_disable(this->DMAr,this->DMA_CHr);
			/*关闭M2M模式*/
			dma_memory_to_memory_disable(this->DMAr,this->DMA_CHr);
			dma_interrupt_disable(this->DMAr,this->DMA_CHr,DMA_INT_FTF);
			dma_interrupt_flag_clear(this->DMAr,this->DMA_CHr,DMA_INT_FLAG_FTF);
			return 2;
		}
		#endif
		return 0;
	}
	#endif
};
#endif

#endif