/**
  ******************************************************************************
  * @file    SysTick.c
  * @author  UAV Application Team
  * @version V2.1.0
  * @date    2-September-2014
  * @brief   此文件主要包含中断延时函数。
  ****************************************************************************** 
  * @device  Master---STM32F405RGT6
  ******************************************************************************  
*/ 
#include "stm32f4xx.h"
#include "SysTick.h"
void SysTick_Configuration(void)
{
	if (SysTick_Config(SystemCoreClock/1000000))
	{ 
		while (1);
	}
}


void delay_us(uint16_t nTime)
 {
   u32 temp;
   SysTick_Config(nTime*168);//
   do
    {
      temp=SysTick->CTRL;
    }
   while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
   SysTick->CTRL=0x00;       //关闭计数器
   SysTick->VAL =0X00;       //清空计数器	
 }


void delay_ms(uint16_t nTime)
 {
    for(uint16_t j=0;j<nTime;j++)
    {

         delay_us(1000);
    }	
 }

/************************ (C) COPYRIGHT AOYIWEIYENG *****END OF FILE***********/
