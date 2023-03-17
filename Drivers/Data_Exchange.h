/**********************************
		qianwan's Code
		  2022.4.10
///////////////////////////////////	
	数据处理
	将浮点数无损转换
	转换前后数组为由高位到低位(大端)
	数组长度与相对bit数有关
**********************************/
#ifndef DATA_EXCHANGE_H
#define DATA_EXCHANGE_H
#include "main.h"
//结构体封装函数
struct Transform_t
{
	void (*Float_To_U8)(float *Input, uint8_t *Output, uint8_t RawNum);
	void (*U8_To_Float)(uint8_t *Input, float *Output, uint8_t RawNum);
	void (*Float_To_U16)(float *Input, uint16_t *Output, uint8_t RawNum);
	void (*U16_To_Float)(uint16_t *Input, float *Output, uint8_t RawNum);
	void (*Float_To_U32)(float *Input, uint32_t *Output, uint8_t RawNum);
	void (*U32_To_Float)(uint32_t *Input, float *Output, uint8_t RawNum);
};

extern struct Transform_t Transform;
#endif