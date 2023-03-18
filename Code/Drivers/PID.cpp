/*********************************************************************************
  *FileName:		PID.cpp
  *Author:  		qianwan
  *Detail: 			PID算法基类,PID_Init为纯虚函数
  
  *Version:  		1.2
  *Date:  			2023/03/13
  *Describe:		修补BUG
  
  *Version:  		1.1
  *Date:  			2023/02/26
  *Describe:		调整积分微分范围判定条件表达式

  *Version:  		1.0
  *Date:  			2023/02/10
  *Describe:		基于测试数据,取消对CMSIS-DSP的依赖
**********************************************************************************/
#include "PID.h"
#include "math.h"
/*增量式PID*/
float PID_Inc_C::PID_Cal(float fdb)
{
	/*中间量*/
	float Kp =0.0f;
	float Ki =0.0f;
	float Kd =0.0f;
	float OUT =0.0f;
	
	/*前期准备*/
	this->FeedBack = fdb;
	this->Error 	= this->Ref 	 - this->FeedBack;	
	this->DError 	= this->Error  - this->PreError;
	this->DDError = this->DError - this->PreDError;			
	this->PreError = this->Error;
	this->PreDError = this->DError;
			
	/*比例积分微分运算*/
	//pid->Out = pid->Out + (pid->Kp * pid->DError + pid->Ki * pid->Error + pid->Kd * pid->DDError);
	Kp = this->Kp * this->DError;		
	//I 积分分离
	if(!IN_RANGE_EN_I?1:(fabs(this->Error)<this->IN_RANGE_EN_I))
	{Ki = this->Ki * this->Error;}		
	//D 微分分离
	if(!IN_RANGE_EN_D?1:(fabs(this->Error)<this->IN_RANGE_EN_D))
	{Kd = this->Kd * this->DDError;}	
	//求和
	OUT = this->Out + Kp + Ki + Kd;
	
	/*后期处理*/
	//输出限幅
	OUT = (OUT > this->MaxOutValue)?this->MaxOutValue:OUT;
	OUT = (OUT < this->MinOutValue)?this->MinOutValue:OUT;
	//赋值
	this->Out = OUT;
	return OUT;
}
	
/*位置式PID*/
float PID_Pla_C::PID_Cal(float fdb)
{
	/*中间量*/
	float Kp =0.0f;
	float Ki =0.0f;
	float Kd =0.0f;
	float OUT =0.0f;
	
	/*前期准备*/
	this->FeedBack = fdb;
	this->Error = this->Ref - this->FeedBack;
	this->integral	+=	this->Error;

	//积分限幅
	this->integral = (this->integral > this->Maxintegral)?this->Maxintegral:this->integral;
	this->integral = (this->integral < this->Minintegral)?this->Minintegral:this->integral;
	
	/*比例积分微分运算*/
	//pid->Out = pid->Kp * pid->Error + pid->Ki * pid->integral  + pid->Kd * (pid->Error - pid->DError);		
	//P	
	Kp = this->Kp * this->Error;		
	//I 积分分离
	if(!IN_RANGE_EN_I?1:(fabs(this->Error)<this->IN_RANGE_EN_I))	
	{Ki = this->Ki * this->integral;}		
	//D 微分分离
	if(!IN_RANGE_EN_D?1:(fabs(this->Error)<this->IN_RANGE_EN_D))
	{Kd = this->Kd * (this->Error - this->DError);}		
	//求和
	OUT = Kp + Ki + Kd;
			
	/*后期处理*/
	//输出限幅
	OUT = (OUT > this->MaxOutValue)?this->MaxOutValue:OUT;
	OUT = (OUT < this->MinOutValue)?this->MinOutValue:OUT;
	//赋值
	this->DError = this->Error;
	this->Out = OUT;
	return OUT;
}
