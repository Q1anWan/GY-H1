/*********************************************************************************
  *FileName:		QCSLite.cpp
  *Author:  		qianwan
  *Detail: 			四元数驱动 右前上-XYZ-RollPitchYaw 弧度制

  *Version:  		1.4.1
  *Date:  			2023/05/20
  *Describe:		修复四元数旋转函数, 使用快速反正切函数
  
  *Version:  		1.4
  *Date:  			2023/03/27
  *Describe:		重写函数名
  
  *Version:  		1.3
  *Date:  			2022/09/04
  *Describe:		修改变量名，增加Cpp支持

  *Version:  		1.0
  *Date:  			2022/04/12
  *Describe:		项目创建
**********************************************************************************/
#include "QCSLite.h"
#include "arm_math.h"
cQCS QCS;

/*---------------------FUNCTIONS---------------------*/
/***********************************************************************
** 函 数 名: Rotate
** 函数说明: 四元数绕指定轴旋转Theta角
**---------------------------------------------------------------------
** 输入参数: 输入四元数、旋转轴、旋转角
** 返回参数: 旋转后的四元数
***********************************************************************/
void cQCS::Rotate(float *inputQ, float *outputQ, float *axis, float radian)
{
	float sinTheat = arm_sin_f32(radian*0.5f);
	float p[4] = {arm_cos_f32(radian*0.5f),sinTheat*axis[0],sinTheat*axis[1],sinTheat*axis[2]};	
	float tem[4];
	
	arm_quaternion_product_single_f32(p,inputQ,outputQ);
}


/***********************************************************************
** 函 数 名: Roll
** 函数说明: 计算某四元数Roll欧拉角 弧度
**---------------------------------------------------------------------
** 输入参数: 四元数
** 返回参数: Roll欧拉角
***********************************************************************/
float cQCS::Roll(float *inputQ)
{
	//atan2(2(q0q1_q2q3),1-2(q1q1+q2q2))
	float result;
	arm_atan2_f32(	2.0f*(inputQ[0]*inputQ[1] + inputQ[2]*inputQ[3]),
					1.0f - 2.0f*(inputQ[1]*inputQ[1] + inputQ[2]*inputQ[2]),
					&result);
	return result;
}

/***********************************************************************
** 函 数 名: Pitch
** 函数说明: 计算某四元数Pitch欧拉角 弧度
**---------------------------------------------------------------------
** 输入参数: 四元数
** 返回参数: Pitch欧拉角
***********************************************************************/
float cQCS::Pitch(float *inputQ)
{
	//asin(2(q0q2-q3q1))
	return asinf(2.0f*(inputQ[0]*inputQ[2]-inputQ[3]*inputQ[1]));
}

/***********************************************************************
** 函 数 名: Yaw
** 函数说明: 计算某四元数Yaw欧拉角 弧度
**---------------------------------------------------------------------
** 输入参数: 四元数
** 返回参数: Yaw欧拉角
***********************************************************************/
float cQCS::Yaw(float *inputQ)
{
	//atan2(2(q0q3+q1q2),1-2(q2q2+q3q3))
	float result;
	arm_atan2_f32(	2.0f*(inputQ[0]*inputQ[3] + inputQ[1]*inputQ[2]),
					1.0f - 2.0f*(inputQ[2]*inputQ[2] + inputQ[3]*inputQ[3]),
					&result);
	return result;
}

/***********************************************************************
** 函 数 名: Euler
** 函数说明: 四元数转欧拉角
**---------------------------------------------------------------------
** 输入参数: 四元数
** 返回参数: RollPitchYaw欧拉角
***********************************************************************/
void cQCS::Euler(float *inputQ, float *radian)
{	
	radian[0] = this->Roll(inputQ);
	radian[1] = this->Pitch(inputQ);
	radian[2] = this->Yaw(inputQ);
}

/***********************************************************************
** 函 数 名: Quaternion
** 函数说明: 欧拉角转四元数
**---------------------------------------------------------------------
** 输入参数: RollPitchYaw欧拉角
** 返回参数: 四元数
***********************************************************************/
void cQCS::Quaternion(float *radian, float *outputQ)
{	
	float cosR = arm_cos_f32(radian[0]*0.5f);
	float sinR = arm_sin_f32(radian[0]*0.5f);
	float cosP = arm_cos_f32(radian[1]*0.5f);
	float sinP = arm_sin_f32(radian[1]*0.5f);
	float cosY = arm_cos_f32(radian[2]*0.5f);
	float sinY = arm_sin_f32(radian[2]*0.5f);
	
	outputQ[0] = cosY*cosP*cosR + sinY*sinP*sinR;
	outputQ[1] = sinY*cosP*cosR + cosY*sinP*sinR;
	outputQ[2] = cosY*sinP*cosR + sinY*cosP*sinR;
	outputQ[3] = cosY*cosP*sinR + sinY*sinP*cosR;
}
