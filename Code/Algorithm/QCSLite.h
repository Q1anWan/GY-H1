/*********************************************************************************
  *FileName:		QCSLite.h
  *Author:  		qianwan
  *Detail: 			��Ԫ������ ��ǰ��-XYZ-RollPitchYaw ������

  *Version:  		1.4.1
  *Date:  			2023/05/20
  *Describe:		ʹ�ÿ��ٷ����к���
  
  *Version:  		1.4
  *Date:  			2023/03/27
  *Describe:		��д������
  
  *Version:  		1.3
  *Date:  			2022/09/04
  *Describe:		�޸ı�����������Cpp֧��

  *Version:  		1.0
  *Date:  			2022/04/12
  *Describe:		��Ŀ����
**********************************************************************************/
#ifndef QCSLITE_H
#define QCSLITE_H

#include "main.h"
#ifdef __cplusplus

#ifndef PI
  #define PI               		3.14159265358979f
#endif

#ifndef PI_Half
  #define PI_Half               1.57079632679489f
#endif

#ifndef PI_Quart
  #define PI_Quart              0.78539816339744f
#endif

#ifndef PI_Eighth
  #define PI_Eighth             0.39269908169872f
#endif

#ifndef PI_Double
  #define PI_Double             6.28318530717958f
#endif


class cQCS
{
	public:
	void Euler(float *inputQ,float *radian);
	void Quaternion(float *radian,float *outputQ);
	void Rotate(float *inputQ, float *outputQ, float *axis, float radian);
	float Roll(float *inputQ);
	float Pitch(float *inputQ);
	float Yaw(float *inputQ);
};
extern cQCS QCS;

#endif
#endif
