/**
  ******************************************************************************
  * @file    SLAVER/STM32F4xx_StdPeriph_V1.3.0/main.h
  * @author  UAV Application Team: xujinqi/guzetao/panjiadi
  * @version V2.1.0
  * @date    23-September-2014
  * @brief   Main program body
  ******************************************************************************
  * @device  Master---STM32F405RGT6
  ******************************************************************************   
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"  
#include "stdio.h"
#include "filter.h" 
#include "Safe_Check.h"
#include "time_init.h" 
#include "SysTick.h"
#include "filter.h" 
#include "eeprom.h"
#include "Senser_calibration.h"
#include "filter.h" 
#include "arm_math.h"  
#include "ff.h"
#include "Communication.h"
#include "GPS.h"
    
#include "BNO055.h"
#include "adconvert.h"
#include "sdio_sdcard.h"
#include "AHRS_EKF.h"
#include "malloc.h"
#include "IMU_Update.h"
#include "sdfatfs.h"
#include "ICM2060x.h"
#include "AHRS_EKF.h"
#include "RX8010.h"
#include "Communication_OriData.h"
#include "Calculate_Wave.h" 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT AOYIWEIYENG *****END OF FILE***********/
