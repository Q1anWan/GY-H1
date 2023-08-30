/**
 ******************************************************************************
 * @file    kalman filter.c
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   C implementation of kalman filter
 
 * @author  qianwan
 * @version V2.0.0
 * @date    2023/07/31
 * @brief   Remove accel lowpass;Remove quaternion from struct;
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "arm_math.h"
#include "kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

typedef struct
{
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float GyroBias[3]; // 陀螺仪零偏估计值

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float YawTotalAngle;

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt; // 姿态更新周期
    mat ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda;                 // 渐消因子

} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float dt);
void IMU_QuaternionEKF_Update(float *q, float gx, float gy, float gz, float ax, float ay, float az);
void IMU_QuaternionEKF_Reset(void);
#ifdef __cplusplus
}
#endif
#endif

