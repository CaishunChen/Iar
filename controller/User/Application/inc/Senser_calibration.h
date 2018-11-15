#ifndef __STM32F10x_MAG_CALIBRATION_H
#define __STM32F10x_MAG_CALIBRATION_H
#ifdef __cplusplus
extern "C" {
#endif
  
#include "stm32f4xx.h"
#include "eeprom.h"




  
  void Mag_Calibration_Operation(void);
  void Mag_Calibration_Step_Update(void);
  void Mag_calibration_init(void);
  void Mag_calibration_init_XY(void);
  void Mag_calibration_init_Z(void);
  void Traverse_magvalue_XY(void);
  void Cali_parameter_caculation_XY(void);
  uint8_t isCali_XYFactor_Check_Succeed(void);
  void Traverse_magvalue_Z(void);
  void Cali_parameter_caculation_Z(void);
  uint8_t isCali_XZFactor_Check_Succeed(void);
  void Save_Cali_parameter(void);
  void Mag_calibration_offset(void);
  
  void Acc_Calibration_Operation();
void Acc_Calibration_Step_Update();
void Cali_parameter_1();
void Cali_parameter_2();
void Cali_parameter_3();
  
#ifdef __cplusplus
}
#endif

#endif