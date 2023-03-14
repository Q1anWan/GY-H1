#ifndef FLASH_GD
#define FLASH_GD
#include "main.h"



#ifdef __cplusplus
extern "C" {
#endif
/*!
    \brief      read Data at Addr
    \param[in]  Addr: Address
    \param[out] none
	\retval     Word at Addr
*/
inline uint32_t fmc_read_u32(uint32_t Addr)
{
	return *(volatile uint32_t *)Addr;
}
uint8_t fmc_erase_pages(uint32_t StartAddr,uint16_t Pages);
uint8_t fmc_program(uint32_t Addr, uint32_t *Words, uint16_t num);
uint8_t fmc_erase_pages_check(uint32_t Addr,uint32_t WordNum);
uint8_t fmc_program_check(uint32_t Addr, uint32_t ByteNum,uint32_t* WordData);
	
#ifdef __cplusplus
}
#endif
#endif