#ifndef RINGBUF_H
#define RINGBUF_H
#ifdef __cplusplus
#include "main.h"
#include "stdio.h"
#include "string.h"
class cRingBuf
{
    protected:
    uint16_t DataSize=0;
    uint16_t Capacity=0;
    uint16_t BufSize=0;
    
    uint8_t *pHead=0;         	//Start address of buffer in memory
    uint8_t *pTail=0;         	//End address of buffer in memory
    uint8_t *pRead=0;  			//Start address of data
    uint8_t *pWrite=0; 			//End   address of data
    public:
    uint8_t Config(uint8_t *pBuf, uint16_t Size);
    uint8_t FreeBuf(void);
    uint8_t Write(uint8_t *pData, uint16_t DataSize);
    uint8_t Read(uint8_t *pData, uint16_t DataSize);
    
	inline uint16_t GetDataSize(void){return this->DataSize;}
	inline uint16_t GetCapacity(void){return this->Capacity;}
	inline uint8_t* GetWriteData(void){return this->pWrite;}
};
#endif
#endif