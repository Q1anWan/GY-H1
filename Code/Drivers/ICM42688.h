/*********************************************************************************
  *FileName:		ICM42688.h
  *Author:  		qianwan
  *Detail: 			ICM42688-P驱动
  
  *Version:  		1.0
  *Date:  			2023/03/10
  *Describe:		新建项目
**********************************************************************************/
#ifndef ICM42688_H
#define ICM42688_H
#include <main.h>
#include "SPI_GD.h"
#ifdef __cplusplus

/*G = 9.8011 in Dalian*/
#define LSB_ACC_16G		0.0047856934f
#define LSB_ACC_8G		0.0023928467f
#define LSB_ACC_4G		0.0011964233f
#define LSB_ACC_2G		0.00059821167f

/*Turn Into Radian*/
#define LSB_GYRO_2000_R	0.0010652644f
#define LSB_GYRO_1000_R	0.00053263222f
#define LSB_GYRO_500_R	0.00026631611f
#define LSB_GYRO_250_R	0.00013315805f
#define LSB_GYRO_125D_R	0.000066579027f

class cICM42688 : public cSPI
{
	
	public:
		
	int16_t Accel[3]={0};//XYZ
	int16_t Gyro[3]={0};//XYZ
	float Temperature=0;
	float LSB_ACC_GYRO[2]={0};
	
	uint8_t ReadReg(  uint8_t Reg);
	void 	ReadReg(  uint8_t Reg, uint8_t* Data, uint8_t num);
	void 	WriteReg( uint8_t Reg, uint8_t  Data);
	void 	WriteReg( uint8_t Reg, uint8_t*	Data, uint8_t num);
	
	void	ReadAccel(void);
	void	ReadGyro(void);
	void	ReadAccelGyro(void);
	void 	ReadTem(void);
	
};


extern "C" {

}

#endif
#endif