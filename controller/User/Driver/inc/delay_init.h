/**
  ******************************************************************************
  * @file    delay_init.h
  * @author  UAV Application Team
  * @version V2.1.0
  * @date    2-September-2014
  * @brief   此文件主要声明延时函数
  ******************************************************************************  
  * @device  Master---STM32F405RGT6
  ******************************************************************************  
*/

#ifndef __STM32F4_DELAY_INIT_H
#define __STM32F4_DELAY_INIT_H
#ifdef __cplusplus
 extern "C" {
#endif
   
#include "stm32f4xx.h"
   
void Delay(__IO uint32_t nCount);  
     
#ifdef __cplusplus
}
#endif

#endif    

/************************ (C) COPYRIGHT AOYIWEIYENG *****END OF FILE***********/
