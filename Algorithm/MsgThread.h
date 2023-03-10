#ifndef MSG_THREAD
#define MSG_THREAD
#include <main.h>
#include "UART_GD.h"
#ifdef __cplusplus

#define UART_MSG_MAX_LEN 128
#define UART_MSG_CFG_LEN 3
class cMSG : public cUART
{
	public:
	/*
		DirTxFlag shows whether or not data can directly transmit without buf
		DirTxFlag'0 = 0 Cannot
		DirTxFlag'0 = 1 Ready to do
		DirTxFlag'1 = 0 Not Transmitting
		DirTxFlag'1 = 1 Transmitting
	*/
	uint8_t DirTxFlag = 0b01;
	uint8_t InBufNum = 0;
	uint8_t TxBSY = 0;
	
	uint8_t TxTestBuf = 1;
	
	
	uint8_t MSGTx(uint8_t *pdata, uint16_t Length);
	uint8_t MSGTx(uint8_t Head, uint8_t *pdata, uint16_t Length);
	uint8_t MSGUrgentTx(uint8_t *pdata, uint16_t Length);
	uint8_t MSGUrgentTx(uint8_t Head, uint8_t *pdata, uint16_t Length);
};
extern cMSG *MSG_TX;

extern "C" {

void DMA0_Channel3_IRQHandler(void);
void rt_hw_console_output(const char *str);
}

#endif
#endif