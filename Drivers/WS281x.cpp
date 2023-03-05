/*********************************************************************************
  *FileName:	WS281x.cpp
  *Author:  	qianwan
  *Details: 	WS281x SPI 驱动 API
				依赖SPI软件或硬件驱动,时钟频率=2.4MHz
  *CharaterCode: UTF-8

  *Version:  	1.0
  *Date:  		2023/03/05
  *Describe:  创建代码
**********************************************************************************/
#include "WS281x.h"

/*发送缓冲区长度应当为 9x灯+1 ,最后一位恒为0*/
void cWS281x::Init(uint8_t *TransmitBuf)
{
	this->TransmitBuf = TransmitBuf;
}

void cWS281x::Color_Transform(LEDColor_t *ColorTable, uint8_t *buf, uint16_t num)
{
	for(uint16_t i=0;i<num;i++)
	{	
		for(uint16_t j=0;j<3;j++)
		{
			uint32_t tem_led=0;
			uint8_t scan = 0x80;
			for(uint8_t k=0;k<8;k++)
			{
				tem_led |= ( (ColorTable[i].GRB[j] & scan) ? 0b110:0b100 );
				tem_led <<= 3;
				scan>>=1;
			}
			buf[9*i+3*j] 	= (uint8_t)((tem_led >> 19)&0xFF);
			buf[9*i+3*j+1]	= (uint8_t)((tem_led >> 11)&0xFF);
			buf[9*i+3*j+2]	= (uint8_t)((tem_led >> 3)&0xFF);
		}
	}
	buf[9*num]=0;
}
	
uint8_t cWS281x::LED_Update(LEDColor_t *ColorTable,uint16_t num)
{
	if(ColorTable==0){return 1;}
	
	this->Color_Transform(ColorTable,this->TransmitBuf,num);
	
	for(uint16_t i=0;i<9*num;i++)
	{this->SPI_ExchangeOneByte(this->TransmitBuf[i]);}
	
	this->SPI_ExchangeOneByte(0);
	return 0;
}

uint8_t cWS281x::LED_UpdateDMA(LEDColor_t *ColorTable,uint16_t num)
{
	if(ColorTable==0){return 1;}
	
	this->Color_Transform(ColorTable,this->TransmitBuf,num);
	
	return this->Transmit_DMA(this->TransmitBuf,9*num+1);
}