#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "main.h"
#define SYS_CONFIG_PACK_LEN 4
#define SYS_CAN_ID_BASE (uint16_t)0x300


#define SYS_CONFIG_MAX_ID 	9
#define SYS_CONFIG_MAX_ODRK 4

#ifdef __cplusplus

class cCTR
{
	public:
	uint8_t		OTSel			= 0x00;			//输出接口
	uint16_t	CAN_ID			= 0x300;		//CAN_ID
	uint8_t 	ODR				= 0x03;			//输出速率
	uint8_t		OutPutMode		= 0x00;			//对外输出模式
	uint8_t		OutPutModeLast	= OutPutMode;	//原对外输出模式
	
	uint8_t		TemperatureOK	= 0;		//温度补偿OK
	uint8_t		EnableOutput	= 1;		//允许输出
	uint8_t		ConfigFlag		= 0;		//设置模式标志位

};
extern cCTR *qCtr;
extern "C" {

}
#endif	
#endif