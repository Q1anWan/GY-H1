#ifndef MSG_THREAD
#define MSG_THREAD
#include <main.h>
#include "UART_GD.h"
#ifdef __cplusplus

#define UART_MSG_MAX_LEN 128
#define UART_MSG_CFG_LEN 3
#define UART_MSG_BUF_CAP 15

class cMSG : public cUART
{
	public:
	uint8_t TxBufLen = 0;
	uint8_t TxDirOK = 1;
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