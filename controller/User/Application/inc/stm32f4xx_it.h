/**
  ******************************************************************************
  * @file    TIM/TIM_TimeBase/main.h
  * @author  UAV Application Team
  * @version V2.1.0
  * @date    23-September-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F40x_IT_H
#define __STM32F40x_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "BNO055.h"
   
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void RTC_Alarm_IRQHandler(void);
void TIM2_IRQHandler(void);
void double2hex(uint8_t *hexdata, double ddata);
extern void USB_OTG_BSP_TimerIRQ(void);
extern uint8_t Time3_Flag_400hz,Time3_Flag_50hz,Time3_Flag_5hz,Time3_Count_8hz,Time3_Flag_20hz,Time3_Flag_Led1,Time3_Flag_100hz,Time3_Flag_2hz;
extern u8 Time3_Flag_SRF02;
void TIM4_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif /* __STM32F40x_IT_H */

/************************ (C) COPYRIGHT AOYIWEIYENG *****END OF FILE***********/
