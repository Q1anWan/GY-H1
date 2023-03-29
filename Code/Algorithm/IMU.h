#ifndef IMU_H
#define IMU_H
#include <main.h>
#include "ICM42688.h"
#include "PID.h"
#ifdef __cplusplus
class cIMU :public cICM42688,public PID_Inc_C
{
	public:
	float	Q[4]={1,0,0,0};
	
	float GyroCal[3]={0};
	float AccelCal[3]={0};

	float GyroCorrected[3]={0};
	float AccelCorrected[3]={0};
	
	void PID_Init(void)
	{
		this->Ref = 41.0f;
		this->Kp = 200.0f;
		this->Ki = 2.0f;
		this->Kd = 5.0f;
		this->IN_RANGE_EN_D = 3.0f;	//开启微分项范围 值为0时始终开启
		this->IN_RANGE_EN_I = 0.0f;	//开启积分项范围 值为0时始终开启
		this->MaxOutValue=999.0;		//输出限幅
		this->MinOutValue=300.0;
	}
	cIMU()
	{PID_Init();}
};
extern cIMU *IMU;

extern "C" {
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);

}
#endif
#endif