#ifndef MSG_THREAD
#define MSG_THREAD
#include <main.h>
#include "UART_GD.h"
#ifdef __cplusplus

class cMSG : public cUART
{
	public:
	uint8_t MSGTx(uint8_t *pdata, uint16_t Length);
	uint8_t MSGTx(uint8_t Head, uint8_t *pdata, uint16_t Length);
};
extern cMSG *MSG_TX;

extern "C" {
void DMA0_Channel3_IRQHandler(void);
void rt_hw_console_output(const char *str);
}

#endif
#endif