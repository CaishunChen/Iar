#ifndef __IMU_UPDATE_H
#define __IMU_UPDATE_H

#ifdef __cplusplus
extern "C" {
#endif
  
#include "stm32l4xx.h"
#include "stdio.h"
#include "math.h"


extern float ROLL,PITCH;  
  
/******º¯Êý¶¨Òå-------------------------------------------------------------------*/
float constrain_float(float value, const float min_val, const float max_val);
float invSqrt(float x);
//void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float mx,float my,float mz);
  
#ifdef __cplusplus
}
#endif
#endif
