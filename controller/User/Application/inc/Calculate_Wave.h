#ifndef __CALCULATE_WAVE_H
#define __CALCULATE_WAVE_H

#ifdef __cplusplus
extern "C" {
#endif 
  
  
#include "stm32f4xx.h"
#include <stdio.h>
  
#include "structural.h"    
#include "BNO055.h"
  
  
#define FFT_LENGTH		1024 
  
  void Get_Acc_Cycle();
  void Wave_Height_Calculate();
  void Get_Cycle_Height();
  
  void FFT_init();
  void FFT_Cal();
  
//  uint8_t Is_Reaching_RoutinueTime();
  uint32_t Is_Reaching_IntervalTime(u16 year,u8 mon,u8 day,u8 hour,u8 min,u8 sec);//采样间隔
  
  uint32_t HMS_To_Second(u8 day,u8 hour,u8 min,u8 sec);//参考点时间
uint8_t Leap_Year(int year);
uint16_t YMD_To_Day(u16 year,u8 mon,u8 day);//最多一年

  
#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT  *****END OF FILE***********/