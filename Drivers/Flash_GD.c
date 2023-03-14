#include "Flash_GD.h"
#define FMC_PAGE_SIZE           ((uint16_t)0x800U)

/*编程保护范围:第127页(256K)*/
/*0等待Flash最后一页*/
#define FMC_PROGRAM_START       ((uint32_t)0x803F800U)
#define FMC_PROGRAM_END        	((uint32_t)0x803FFFFU)

/*!
    \brief      erase a few number of fmc pages from StartAddr
    \param[in]  StartAddr:First page's address  Pages:numbers of pages
    \param[out] none
	\retval     0: Operate OK 	1: Addr is invaild
*/
uint8_t fmc_erase_pages(uint32_t StartAddr,uint16_t Pages)
{
    uint32_t EraseCounter;
	/*Make sure flash address is vaild*/
	if(StartAddr%FMC_PAGE_SIZE != 0){return 1;}
	else if(StartAddr<FMC_PROGRAM_START){return 1;}
	else if(StartAddr + Pages*FMC_PAGE_SIZE > FMC_PROGRAM_END){return 1;}
    
	/* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    for(EraseCounter = 0; EraseCounter < Pages; EraseCounter++){
        fmc_page_erase(StartAddr + (FMC_PAGE_SIZE * EraseCounter));
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
	return 0;
}

/*!
    \brief      program fmc word by word from Addr,write number
	\param[in]  Addr:start addr	 Words:Data need to write  num:Number of words
    \param[out] none
	\retval     0: Operate OK 	1: Addr is invaild
*/
uint8_t fmc_program(uint32_t Addr, uint32_t *Words, uint16_t num)
{
	/*Make sure flash address is vaild*/
	if(Addr<FMC_PROGRAM_START){return 1;}
	else if(Addr + 4*num > FMC_PROGRAM_END){return 1;}
	
    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* program flash */
    for(uint16_t i=0;i<num;i++){
        fmc_word_program(Addr, Words[i]);
        Addr += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the program operation */
    fmc_lock();
	return 0;
}

/*!
    \brief      check fmc erase result
	\param[in]  Addr: StartAddress  WordNum: Number of Addr that need to check
    \param[out] none
	\retval     0:All right 1:Not all right
*/
uint8_t fmc_erase_pages_check(uint32_t Addr,uint32_t WordNum)
{
    uint32_t i;

	uint32_t *ptrd = (uint32_t *)Addr;
    /* check flash whether has been erased */
    for(i = 0; i < WordNum; i++){
        if(0xFFFFFFFF != (*ptrd)){
            return 1;
        }else{
            ptrd++;
        }
    }
	return 0;
}

/*!
    \brief      check fmc program result
	\param[in]  Addr: StartAddress ByteNum: Number of addr that need to check WordData: Compare data
    \param[out] none
    \retval     0:All right 1:Not all right
*/
uint8_t fmc_program_check(uint32_t Addr, uint32_t ByteNum,uint32_t* WordData)
{
    uint32_t i;

    uint32_t *ptrd = (uint32_t *)Addr;

    /* check flash whether has been programmed */
    for(i = 0; i < ByteNum; i++){
        if((*ptrd) != WordData[i]){
            return 1;
        }else{
            ptrd++;
        }
    }
	return 0;
}