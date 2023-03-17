#include "Data_Exchange.h"
#include "string.h"
/**********************************
		qianwan's Code
		  2022.4.10
///////////////////////////////////	
	数据处理
	将浮点数无损转换
	转换前后数组为由高位到低位(大端)
	数组长度与相对bit数有关
**********************************/

/*
	Float_To_U8
	浮点转无符号短整型
	长度拓展四倍
	
	*Input	输入指针
	*Output	输出指针
	RawNum	输入数目
*/
void Float_To_U8(float *Input, uint8_t *Output, uint8_t RawNum)
{
	uint8_t Temp[4];
	uint8_t i=0;
	for(i=0;i<RawNum;i++)
	{
		*(float *)Temp = Input[i];
		Output[4*i]   = Temp[0]	;
		Output[4*i+1] = Temp[1]	;
		Output[4*i+2] = Temp[2]	;
		Output[4*i+3] = Temp[3]	;
	}
}

/*
	U8_To_Float
	无符号短整型转浮点
	长度缩短四倍

	*Input	输入指针
	*Output	输出指针
	RawNum	输入数目
*/
void U8_To_Float(uint8_t *Input, float *Output, uint8_t RawNum)
{
	uint8_t Temp[4];
	uint8_t i=0;
	for(i=0;i<( RawNum/4);i++)
	{
		Temp[0] = Input[4*i];
		Temp[1] = Input[4*i+1];
		Temp[2] = Input[4*i+2];
		Temp[3] = Input[4*i+3];
		memcpy(&Output[i], Temp, 4);
	}
}

/*
	Float_To_U16
	浮点转无符号整型
	长度拓展两倍

	*Input	输入指针
	*Output	输出指针
	RawNum	输入数目
*/
void Float_To_U16(float *Input, uint16_t *Output, uint8_t RawNum)
{
	uint32_t Temp[RawNum];
	uint8_t i=0;
	for(i=0;i<RawNum;i++)
	{
		Temp[i] = *(uint32_t *)&Input[i];
		Output[2*i]   = (Temp[i] >> 16) & 0xffff;
		Output[2*i+1] =  Temp[i] 		& 0xffff;
	}
}

/*
	U16_To_Float
	无符号整型转浮点
	长度缩短两倍

	*Input	输入指针
	*Output	输出指针
	RawNum	输入数目
*/
void U16_To_Float(uint16_t *Input, float *Output, uint8_t RawNum)
{
	uint32_t Temp[RawNum/2];
	uint8_t i=0;
	for(i=0;i<( RawNum/2);i++)
	{
		Temp[i] = (uint32_t)((Input[2*i]<<16)|Input[2*i+1]);
		Output[i] = *(float *)&Temp[i];
	}
}

/*
	Float_To_U32
	浮点转无符号长整型
	长度不变

	*Input	输入指针
	*Output	输出指针
	RawNum	输入数目
*/
void Float_To_U32(float *Input, uint32_t *Output, uint8_t RawNum)
{
	uint8_t i=0;
	for(i=0;i<RawNum;i++)
	{
		Output[i] = *(uint32_t *)&Input[i];
	}
}

/*
	U32_To_Float
	无符号长整型转浮点
	长度不变

	*Input	输入指针
	*Output	输出指针
	RawNum	输入数目
*/
void U32_To_Float(uint32_t *Input, float *Output, uint8_t RawNum)
{
	uint8_t i=0;
	for(i=0;i<RawNum;i++)
	{
		Output[i] = *(float *)&Input[i];
	}
}

struct Transform_t Transform = {
    .Float_To_U8=Float_To_U8,
	.U8_To_Float=U8_To_Float,
	.Float_To_U16=Float_To_U16,
	.U16_To_Float=U16_To_Float,
	.Float_To_U32=Float_To_U32,
	.U32_To_Float=U32_To_Float,
};
