#ifndef IMU_H
#define IMU_H
#include <main.h>
#include "ICM42688.h"
#ifdef __cplusplus

extern cICM42688 *IMU;
extern "C" {
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);

}
#endif
#endif