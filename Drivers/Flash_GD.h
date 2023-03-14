#ifndef FLASH_GD
#define FLASH_GD
#include "main.h"



#ifdef __cplusplus
extern "C" {
#endif
/*!
    \brief      read Data at Addr
    \param[in]  Addr: Address  Data:Read data buf  Words:Number of words
    \param[out] none
	\retval     Word at Addr
*/
inline void fmc_read_u32(uint32_t Addr, uint32_t *Data, uint16_t Words)
{
	for(uint16_t i=0;i<Words;i++)
	{
		Data[i] = *(volatile uint32_t *)Addr;
		Addr+=4;
	}
}
uint8_t fmc_erase_pages(uint32_t StartAddr,uint16_t Pages);
uint8_t fmc_program(uint32_t Addr, uint32_t *Words, uint16_t num);
uint8_t fmc_erase_pages_check(uint32_t Addr,uint32_t WordNum);
uint8_t fmc_program_check(uint32_t Addr, uint32_t ByteNum,uint32_t* WordData);
	
#ifdef __cplusplus
}
#endif
#endif