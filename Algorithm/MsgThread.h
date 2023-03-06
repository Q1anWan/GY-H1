#ifndef MSG_THREAD
#define MSG_THREAD
#include <main.h>
#include "UART_GD.h"
#ifdef __cplusplus
#define UART_MSG_POOL_LEN 2048
#define UART_MSG_MAX_LEN 128
#define UART_MSG_CFG_LEN 3
class cMSG
{
	public:
	uint8_t MSGTx(uint8_t *pdata, uint16_t Length);
	uint8_t MSGTx(uint8_t Head, uint8_t *pdata, uint16_t Length);
	uint8_t MSGUrgentTx(uint8_t *pdata, uint16_t Length);
	uint8_t MSGUrgentTx(uint8_t Head, uint8_t *pdata, uint16_t Length);
};
extern cMSG MSG_UART0;

extern "C" {

void DMA0_Channel3_IRQHandler(void);

}

#endif
#endif