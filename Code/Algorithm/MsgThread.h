#ifndef MSG_THREAD
#define MSG_THREAD
#include <main.h>
#include "UART_GD.h"
#include "usbd_core.h"
#include "cdc_acm_core.h"
#include "usbd_lld_int.h"
#include "usbd_lld_core.h"
#ifdef __cplusplus

class cMSG : public cUART
{
	
	public:
	uint8_t UartRecBuf[10];
	usb_cdc_handler *USB_COM = RT_NULL;
	can_receive_message_struct *CAN_RecMsg = RT_NULL;
	
	uint8_t UartTx(uint8_t *pdata, uint16_t Length, rt_int32_t WaitTime);
	uint8_t UartTx(uint8_t Head, uint8_t *pdata, uint16_t Length, rt_int32_t WaitTime);
	void Printf(const char * format, ...);
	
	uint8_t USBTx(uint8_t *pdata, uint16_t Length, rt_int32_t WaitTime);
	void CANTx(uint16_t CAN_ID, uint8_t* data, uint8_t len);
};
extern cMSG *Msg;

extern "C" {
void DMA0_Channel3_IRQHandler(void);
void USART0_IRQHandler(void);
void rt_hw_console_output(const char *str);
void USBD_LP_CAN0_RX0_IRQHandler(void);
void USBD_WKUP_IRQHandler(void);
}

#endif
#endif