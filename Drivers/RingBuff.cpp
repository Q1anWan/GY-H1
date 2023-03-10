#include "RingBuff.h"
inline uint16_t min(uint16_t a,uint16_t b)
{return a<b?a:b;}

/*Ring buffer's points will be init*/
uint8_t cRingBuf::Config(uint8_t *pBuf,uint16_t Size)
{
	if(pBuf==0){return 1;}
	if(Size==0){return 1;}
    this->pHead = pBuf;
    this->pTail = pBuf + Size;
    this->BufSize = Size;

    this->pRead = this->pHead;
    this->pWrite = this->pHead;

    this->Capacity = Size;
    this->DataSize = 0;
	return 0;
}

uint8_t cRingBuf::FreeBuf(void)
{
    this->pRead = this->pHead;
    this->pWrite = this->pHead;
    this->Capacity = this->BufSize;
    this->DataSize = 0;
	
	return 0;
}

uint8_t cRingBuf::Write(uint8_t *pData, uint16_t Size)
{
    if(Size>this->Capacity){return 1;}

    if (this->pWrite+Size>pTail)//Data is too long that need to be cutout
	{
		uint16_t Data_Part1 = this->pTail - pWrite;
		uint16_t Data_Part2 = Size - Data_Part1;
		memcpy(this->pWrite, pData, Data_Part1 * sizeof(uint16_t));
		memcpy(this->pHead, pData+Data_Part1, Data_Part2 * sizeof(uint16_t));
		this->pWrite = pHead + Data_Part2;
	}
	else
	{
		memcpy(this->pWrite, pData, Size * sizeof(uint16_t));
		pWrite += Size;
	}

    this->DataSize += Size;
    this->Capacity -= Size;

	return 0;
}

uint8_t cRingBuf::Read(uint8_t *pData, uint16_t Size)
{ 
    //Error
	if (Size > this->DataSize){return 1;}
	if (Size == 0){return 0;}

	//Is data continuous
	if (this->pRead + Size > this->pTail)
	{
		uint16_t Data_Part1 = this->pTail - this->pRead; 
		uint16_t Data_Part2 = Size - Data_Part1;
		memcpy(pData, this->pRead, Data_Part1 * sizeof(uint16_t));
		memcpy(pData + Data_Part1, this->pHead, Data_Part2 * sizeof(uint16_t));
		this->pRead = this->pHead + Data_Part2;
	}
	else
	{
		memcpy(pData, this->pRead, Size * sizeof(uint16_t));
		this->pRead += Size;
	}

    this->DataSize -= Size;
    this->Capacity += Size;

	return 1;
}
