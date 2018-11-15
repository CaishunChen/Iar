#ifndef __STM32F10x_AHRS_H
#define __STM32F10x_AHRS_H
#ifdef __cplusplus
extern "C" {
#endif
  
#include "stm32f4xx.h"
#include "stdio.h"
#include "structural.h"  
#include "stm32f4xx_it.h"
#include "math.h"
#include "stdlib.h"
#include "filter.h" 
  
void AHRS_EKF(void);
void AHRS_EKF_Init(void);
void MatrixOpp(float A[][3],float B[][3]);

float wraparound(float dta);
#ifdef __cplusplus
}
#endif

#endif