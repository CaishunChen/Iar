/**
  ******************************************************************************
  * @file    time_init.h
  * @author  UAV Application Team
  * @version V2.1.0
  * @date    2-September-2014
  * @brief   此文件主要功能是初始化定时器
  ****************************************************************************** 
  * @device  Master---STM32F405RGT6
  ******************************************************************************  
*/ 

#ifndef __STM32F4_TIME_INIT_H
#define __STM32F4_TIME_INIT_H
#ifdef __cplusplus
 extern "C" {
#endif
   
#include "stm32f4xx.h"
void Time_init(void); 
void Tim2_init(void);
   
#ifdef __cplusplus
}
#endif

#endif    

/************************ (C) COPYRIGHT AOYIWEIYENG *****END OF FILE***********/
