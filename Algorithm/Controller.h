#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "main.h"
#define SYS_CONFIG_PACK_LEN 4
#define SYS_CAN_ID_BASE (uint16_t)0x300
#ifdef __cplusplus

class cCTR
{
	public:
	uint8_t OTSel = 0x00;
	uint16_t CAN_ID = 0x300;
	uint8_t ODR = 0x03;
};
extern cCTR *qCtr;
extern "C" {

}
#endif	
#endif