#ifndef _BNO055_H
#define _BNO055_H


#include <stdio.h>
#include <stdarg.h>
#include "ringbuffer.h"
#include "uart_init.h" 
#include "structural.h" 


       
typedef struct
{
    s16 ax;
    s16 ay;
    s16 az;
    s16 gx;
    s16 gy;
    s16 gz;
    s16 mx;
    s16 my;
    s16 mz;
}IMU_DataAD_TypeDef;


    /*---------------------------- 1 ------------------------*/
#define ACC_X_OFFSET  (-0.12)
#define ACC_Y_OFFSET  (-1.01)
#define ACC_Z_OFFSET  (-1.05)

#define K_ACC_X     (1.016)
#define K_ACC_Y     (1.011)
#define K_ACC_Z     (1.018)

    /*---------------------------- 2 ------------------------*/
//#define ACC_X_OFFSET  (-0.12)
//#define ACC_Y_OFFSET  (0.035)
//#define ACC_Z_OFFSET  (0.315)
//
//#define K_ACC_X     (1.016)
//#define K_ACC_Y     (1.016)
//#define K_ACC_Z     (0.99)



/*Register Map*/
#define Page_ID 0x07 //the address of register Page_ID is 0x07 in page 0 or page 1

//Page 0========================
#define CHIP_ID 0x00//default value:0xA0
#define ACC_ID  0x01//default value:0xFB
#define MAG_ID  0x02//default value:0x32
#define GYR_ID  0x03//default value:0x0F
#define ACC_OFFSET_X_LSB 0x55
#define ACC_DATA_X_LSB  0x08
#define ACC_DATA_X_MSB  0x09
#define ACC_DATA_Y_LSB  0x0A
#define ACC_DATA_Y_MSB  0x0B
#define ACC_DATA_Z_LSB  0x0C
#define ACC_DATA_Z_MSB  0x0D
#define MAG_DATA_X_LSB  0x0E
#define MAG_DATA_X_MSB  0x0F
#define MAG_DATA_Y_LSB  0x10
#define MAG_DATA_Y_MSB  0x11
#define MAG_DATA_Z_LSB  0x12
#define MAG_DATA_Z_MSB  0x13
#define GYR_DATA_X_LSB  0x14
#define GYR_DATA_X_MSB  0x15
#define GYR_DATA_Y_LSB  0x16
#define GYR_DATA_Y_MSB  0x17
#define GYR_DATA_Z_LSB  0x18
#define GYR_DATA_Z_MSB  0x19
#define EUL_Heading_LSB 0x1A
#define EUL_Heading_MSB 0x1B
#define EUL_Roll_LSB    0x1C
#define EUL_Roll_MSB    0x1D
#define EUL_Pitch_LSB   0x1E
#define EUL_Pitch_MSB   0x1F
#define OPR_MODE        0x3D
#define PWR_MODE        0x3E
#define ACC_OFFSET_X_LSB  0x55
#define UNIT_SEL        0x3B
//Page 1
#define ACC_Config  0x08
#define MAG_Config  0x09
#define GYR_Config_0    0x0A
#define GYR_Config_1    0x0B
#define CALIB_STAT 0x35


extern IMU_DataAD_TypeDef* pIMU_DataAD;
/*function declare*/
void BNO055_Init();


u8 BNO055_IsReadyToRead();
void BNO055_Update();
void UART5_init(u32 bound);

void filter(float modulus);
void BNO_Configure(void);

uint8_t Chose_BNO_WriteData(u8 *txbuf,u8 regaddr,u8 dat);
uint8_t Chose_BNO_ReadData(u8 *txbuf);

void BNO055_Reg_Check(u8 regaddr,u8 dat);
void BNO055_Reg_Check1(void);
void BNO055_WriteStatus_Check(void);
void BNO055_Decode();

void Senser_data_process(void);
#endif







