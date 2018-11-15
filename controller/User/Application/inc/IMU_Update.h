#ifndef __IMU_UPDATE_H
#define __IMU_UPDATE_H

#ifdef __cplusplus
extern "C" {
#endif
  
#include "stm32f4xx.h"
#include "stdio.h"
#include "structural.h"
#include "filter.h" 


/******º¯Êý¶¨Òå-------------------------------------------------------------------*/
float constrain_float(float value, const float min_val, const float max_val);
float invSqrt(float x);
//void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float mx,float my,float mz);
void Yaw_Combine(void);
void Get_OneTenth_Height();

void Angle_fuse_roll(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f);
void Angle_fuse_pitch(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f);

void Wave_Cycle_cal();
void Get_Routine_Data();

void Tradion_Sample_Flinish();
void Tradion_Sample();
#ifdef __cplusplus
}
#endif
#endif
