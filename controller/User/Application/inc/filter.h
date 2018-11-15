#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
 extern "C" {
#endif 


#include "stm32f4xx.h"
#include <stdio.h>
#include "ringbuffer.h"
#include "uart_init.h"
#include "structural.h"    
#include "BNO055.h"

   extern d_Senser_Data  *pSenser_Data;

#define pi 3.1415926535897932384626433832795028841971
   
   void ICMdata_filter_Acc(void);
   void ICMdata_filter_Gyro(void);
   void IMUdata_filter_ADIAcc(void);
   void butterworth_filter(void);
   void Senserdata_filter_H();
   extern d_Senser_Data  *pSenser_Data_filtered;
   void Acc_get_average();
  void BNOdata_filter_Acc();
  void BNOdata_filter_Gyro();  
   
   void Cal_Height();
   void Threshold_filter();
   void data_filling();

   void FFT_AXToVel();
      void FFT_AYToVel();
#ifdef __cplusplus
}
#endif

#endif 

/************************ (C) COPYRIGHT  *****END OF FILE***********/