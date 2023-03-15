/*********************************************************************************
  *FileName:	    CRC8.h
  *Author:  	    qianwan
  *Details: 	    查表法计算CRC
  *Mode:          CRC-8/MAXIM
  *Polynomial:    X8+X5+X4+1(0x31)
  *Init:          0x00
  *XOROUT:        0x00

  *Version:  	1.0
  *Date:  		2023/03/15
  *Describe:    创建代码
**********************************************************************************/
#ifndef CRC8_H
#define CRC8_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
uint8_t cal_crc8_table(uint8_t *ptr, uint8_t len);
#ifdef __cplusplus
}
#endif
#endif