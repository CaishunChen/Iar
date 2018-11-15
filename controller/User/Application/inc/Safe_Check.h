#ifndef __Safe_Check_H
#define __Safe_Check_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx.h"
#include "stdio.h"
#include "structural.h"
#include "math.h"
  
void Senser_Data_SelfCheck(void);
uint8_t  IsSenser_Ready();
void Mag_Health_Check(void);
void BNO_Acc_Health_Check(void);
void ICM_Acc_Health_Check(void);
void BNO_Gyro_Health_Check(void);
void Gyro_Health_Check(void);
void Get_local_gravity();
void System_Check();
void IWDG_Init(u8 prer,u16 rlr);
void IWDG_Feed(void);
  
#ifdef __cplusplus
}
#endif
#endif