/*********************************************************************************
  *FileName:		QCSLite.cpp
  *Author:  		qianwan
  *Detail: 			四元数驱动 右前上 弧度制
  
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
** 函 数 名： Rotate
** 函数说明： 四元数绕指定轴旋转Theta角
**---------------------------------------------------------------------
** 输入参数： 输入四元数、旋转轴、旋转角
** 返回参数： 旋转后的四元数
***********************************************************************/
void cQCS::Rotate(float *inputQ, float *outputQ, float *axis, float radian)
{
	float sinTheat = arm_sin_f32(radian*0.5f);
	float p[4]={arm_cos_f32(radian*0.5f),sinTheat*axis[0],sinTheat*axis[1],sinTheat*axis[2]};
	float pn[4]={p[0],-p[1],-p[2],-p[3]};
	
	arm_quaternion_product_single_f32(p,inputQ,p);
	arm_quaternion_product_single_f32(p,pn,outputQ);
}


/**@brief 欧拉角采用X-Y-Z**/
/***********************************************************************
** 函 数 名： QCS_GetRollAngel
** 函数说明： 计算某四元数Roll欧拉角
**---------------------------------------------------------------------
** 输入参数： 四元数
** 返回参数： Roll欧拉角
***********************************************************************/
void QCS_GetRollAngel(float Q_input[4],float *Roll)
{
	//Roll = arctan((2 * q2 * q3 + 2 * q0 * q1) / (-2 * q1^2 - 2 * q2^2 + 1)) 
	float temp_a,temp_b,temp_c,temp_d;
	arm_mult_f32(&Q_input[2],&Q_input[3],&temp_a,1);
	arm_mult_f32(&Q_input[0],&Q_input[1],&temp_b,1);
	arm_add_f32(&temp_a,&temp_b,&temp_c,1);
	temp_a =2.0f;
	arm_mult_f32(&temp_a,&temp_c,&temp_c,1);

	arm_mult_f32(&Q_input[1],&Q_input[1],&temp_a,1);
	arm_mult_f32(&Q_input[2],&Q_input[2],&temp_b,1);
	arm_add_f32(&temp_a,&temp_b,&temp_d,1);
	temp_a = -2.0f;
	arm_mult_f32(&temp_a,&temp_d,&temp_d,1);
	
	*Roll = atan2f(temp_c,(temp_d+1));
}

/***********************************************************************
** 函 数 名： QCS_GetYawAngel
** 函数说明： 计算某四元数Yaw欧拉角
**---------------------------------------------------------------------
** 输入参数： 四元数
** 返回参数： Yaw欧拉角
***********************************************************************/
void QCS_GetYawAngel(float Q_input[4],float *Yaw)
{
	//Yaw = arctan((2*q1*q2 + 2*q0*q3) / (-2*q2^2 - 2*q3^2+1))
	float temp_a,temp_b,temp_c,temp_d;
	arm_mult_f32(&Q_input[1],&Q_input[2],&temp_a,1);
	arm_mult_f32(&Q_input[0],&Q_input[3],&temp_b,1);
	arm_add_f32(&temp_a,&temp_b,&temp_c,1);
	temp_a =2.0f;
	arm_mult_f32(&temp_a,&temp_c,&temp_c,1);

	arm_mult_f32(&Q_input[2],&Q_input[2],&temp_a,1);
	arm_mult_f32(&Q_input[3],&Q_input[3],&temp_b,1);
	arm_add_f32(&temp_a,&temp_b,&temp_d,1);
	temp_a = -2.0f;
	arm_mult_f32(&temp_a,&temp_d,&temp_d,1);
	
	*Yaw = atan2f(temp_c,(temp_d+1));
}

/***********************************************************************
** 函 数 名： QCS_GetPitchAngel
** 函数说明： 计算某四元数Pitch欧拉角
**---------------------------------------------------------------------
** 输入参数： 四元数
** 返回参数： Pitch欧拉角
***********************************************************************/
void QCS_GetPitchAngel(float Q_input[4],float *Pitch)
{
	//Pitch = arcsin(2 * q0* q2 - 2 * q1 * q3)
	float temp_a,temp_b,temp_c;
	arm_mult_f32(&Q_input[0],&Q_input[2],&temp_a,1);
	arm_mult_f32(&Q_input[1],&Q_input[3],&temp_b,1);
	arm_sub_f32(&temp_a,&temp_b,&temp_c,1);
	temp_a =2.0f;
	arm_mult_f32(&temp_a,&temp_c,&temp_c,1);
	
	*Pitch = asinf(temp_c);
}

/***********************************************************************
** 函 数 名： QCS_GetErrorQ
** 函数说明： 计算误差四元数
**---------------------------------------------------------------------
** 输入参数： 当前四元数、目标四元数
** 返回参数： 误差四元数
***********************************************************************/
void QCS_GetErrorQ(float Q_Now[4],float Q_Target[4],float Q_output[4])
{
	float InverseQ[4];
	arm_quaternion_inverse_f32(Q_Now,InverseQ,1);
	arm_quaternion_product_f32(Q_Target,InverseQ,Q_output,1);
}
 
/***********************************************************************
** 函 数 名： QCS_CorrectAHRSC
** 函数说明： AHRS四元数修正
**---------------------------------------------------------------------
** 输入参数： AHRS原始四元数，坐标系漂移弧度（单MCU时为0）
** 返回参数： AHRS修正后四元数
***********************************************************************/
void QCS_CorrectAHRSq(float q[4],float AHRSQ[4],float YC)
{
	//C板R标指向X正方向，Z方向垂直R标向外，右手系
	float V[4]={0,0,-1.0f};
	//旋转坐标轴到常规安装方向
	QCS_Rotate(q,AHRSQ,V,PI_Half-YC);
	
}

/***********************************************************************
** 函 数 名： QCS_init_data()
** 函数说明： 初始AHRS算法四元数
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void QCS_init_data(void)
{
	QCS_IMU_Q[0]=1.0f;QCS_IMU_Q[1]=0.0f;QCS_IMU_Q[2]=0.0f;QCS_IMU_Q[3]=0.0f;
}

/***********************************************************************
** 函 数 名： QCS_IMU_update()
** 函数说明： 更新AHRS四元数
**---------------------------------------------------------------------
** 输入参数： 三轴陀螺仪数据,三轴加速度数据,坐标系旋转弧度
** 返回参数： 无
***********************************************************************/
void QCS_AHRS_update(float *AHRS_Q,float *gyro, float *accel,float YC)
{	
	//不使用地磁时调用MahonyAHRSupdateINS
	#ifdef QCS_CORRECT
	MahonyAHRSupdateINS(QCS_IMU_Q, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
	QCS_CorrectAHRSq(QCS_IMU_Q,AHRS_Q,YC);
	#endif
	#ifndef QCS_CORRECT
	MahonyAHRSupdateINS(QCS_IMU_Q, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
	AHRS_Q[0] = QCS_IMU_Q[0];
	AHRS_Q[1] = QCS_IMU_Q[1];
	AHRS_Q[2] = QCS_IMU_Q[2];
	AHRS_Q[3] = QCS_IMU_Q[3];
	#endif
	
}

/***********************************************************************
** 函 数 名： QCS_Show_Degree()
** 函数说明： 计算全部欧拉角
**---------------------------------------------------------------------
** 输入参数： AHRS四元数
** 返回参数： 姿态角
***********************************************************************/
void QCS_Show_Degree(float *AHRSQ, float *IMU_Degree)
{	
	float temp[3];
	QCS_GetRollAngel(AHRSQ,&temp[0]);
	QCS_GetYawAngel(AHRSQ,&temp[1]);
	QCS_GetPitchAngel(AHRSQ,&temp[2]);
	IMU_Degree[0]=temp[0]*57.2957795130f;//Roll
	IMU_Degree[1]=temp[1]*57.2957795130f;//Yaw
	IMU_Degree[2]=temp[2]*57.2957795130f;//Pitch
}
