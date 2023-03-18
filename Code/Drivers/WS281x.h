/*********************************************************************************
  *FileName:	WS281x.h
  *Author:  	qianwan
  *Details: 	WS281x SPI 驱动 API
					依赖SPI软件或硬件驱动,时钟频率=2.4MHz
  *CharaterCode: UTF-8

  *Version:  	1.0
  *Date:  		2023/03/05
  *Describe:  创建代码
**********************************************************************************/
#ifndef WS281X_H
#define WS281X_H
#include <main.h>
#include "SPI_GD.h"
#ifdef __cplusplus
struct LEDColor_t
{
  uint8_t GRB[3];
};

class cWS281x:public cSPI
{
  protected:
  uint8_t *TransmitBuf;
  void Color_Transform(LEDColor_t *ColorTable, uint8_t *buf, uint16_t num);
  
  public:
  void Init(uint8_t *TransmitBuf);
  uint8_t LED_Update(LEDColor_t *ColorTable,uint16_t num);
  uint8_t LED_UpdateDMA(LEDColor_t *ColorTable,uint16_t num);
};

#endif
#endif