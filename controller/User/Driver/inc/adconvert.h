#ifndef _STM32F4xx_ADCONVERT_H
#define _STM32F4xx_ADCONVERT_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#include "stm32f4xx.h"
#include "structural.h" 
   
 void ADC_DMA_init(void); 
 void AD_To_RawAhrs();
extern __IO uint16_t ADCConvertedValue1[1];
   #ifdef __cplusplus
}
#endif
#endif
