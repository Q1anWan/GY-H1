#ifndef MAIN_H
#define MAIN_H
#include "gd32f30x_it.h"
#include "rthw.h"
#include "rtthread.h"
#include "usbd_core.h"
#define GD32_LIB 
#define FLASH_USERDATA_ADDRESS		 ((uint32_t)0x803F800U)//Page 127
#define FLASH_USERDATA_DATAWORDS	 ((uint16_t)0x05));
/*
	Word[0]: Config
	Word[1]: GyroCali-X GyroCali-Y
	Word[2]: GyroCali-Z ACCLCali-X
	Word[3]: ACCLCali-Y ACCLCali-Z
	Word[4]: CRC
*/


#ifdef __cplusplus

extern "C" {
int main(void);
void DMA1_Channel1_IRQHandler(void);
long list_thread(void);
long list_sem(void);
long list_msgqueue(void);
	
//extern uint16_t TxNum;
//extern uint16_t TxTask;
//extern uint16_t TxDir;
//extern uint16_t TxBuf;
//extern uint8_t Test0;
//extern uint8_t Test1;
//extern uint8_t Test2;
//extern uint8_t Test3;
}
#endif
#endif