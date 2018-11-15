/**
  ******************************************************************************
  * @file    SysTick.h
  * @author  UAV Application Team
  * @version V2.1.0
  * @date    2-September-2014
  * @brief   此文件主要包含中断延时函数。
  ****************************************************************************** 
  * @device  Master---STM32F405RGT6
  ******************************************************************************  
*/ 
#ifndef __SYSTICK_H_
#define __SYSTICK_H_

void SysTick_Configuration(void);
void delay_us(uint16_t time);
void delay_ms(uint16_t nTime);
#endif

/************************ (C) COPYRIGHT AOYIWEIYENG *****END OF FILE***********/
