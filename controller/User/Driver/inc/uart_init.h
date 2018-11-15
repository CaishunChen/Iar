/**
  ******************************************************************************
  * @file    uart_init.h
  * @author  UAV Application Team
  * @version V2.1.0
  * @date    2-September-2014
  * @brief   此头文件包含所有UART初始化的头文件
  ******************************************************************************  
  * @device  Master---STM32F405RGT6
  ******************************************************************************  
*/  

#ifndef __STM32F4_UART_INIT_H
#define __STM32F4_UART_INIT_H
#ifdef __cplusplus
 extern "C" {
#endif
   
#include "stm32f4xx.h"

   void UART4_init(u32 bound);
   void BNO055_UARTDMAWriteData(uint8_t *ptr, uint16_t daata_length);
   void UART_Communication_init(void);
   void Communication_Send(uint8_t *ptr, uint16_t daata_length);
   void UART_GPS_init(void);
   void UART_Communication_OriData_init();
   void Communication_OriData_Send(uint8_t *ptr, uint16_t daata_length);
   void GPS_Send(uint8_t *ptr, uint16_t daata_length);


#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT AOYIWEIYENG *****END OF FILE***********/
