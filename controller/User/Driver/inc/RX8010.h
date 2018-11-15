#ifndef __RX8010_H
#define __RX8010_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "sys.h"
#include "structural.h"

#define IIC_Dlay   2
#define uchar unsigned char

#define       TIME_SDA_ON               GPIO_SetBits(GPIOE, GPIO_Pin_4);
#define       TIME_SDA_OFF              GPIO_ResetBits(GPIOE, GPIO_Pin_4);

#define       TIME_SCL_ON               GPIO_SetBits(GPIOE, GPIO_Pin_3);
#define       TIME_SCL_OFF              GPIO_ResetBits(GPIOE, GPIO_Pin_3);

#define       TIME_SDA_IN()             {GPIOE->MODER&=~(3<<(9*2));GPIOE->MODER|=0<<9*2;}//GPIOE->MODER|=GPIO_Mode_IN<<22;  //sda输入      //SDA  
#define       TIME_SDA_OUT()            {GPIOE->MODER&=~(3<<(9*2));GPIOE->MODER|=1<<9*2;}//GPIOE->MODER|=GPIO_Mode_OUT<<22;//sda输出      //SDA  
#define       I2C_SDA_READ()      GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)


void RS8010sj_Init();
void RS8010sj_Init_IO();
void RX8010_Init();
char BCDToHex( char dat );
unsigned int HEX2BCD( unsigned char hex_data );
void IIC_Start( void );
void IIC_Stop( void );
u8 IIC_Wait_Ack( void );
void IIC_Ack( void );
void IIC_NAck( void );
void IIC_SendByte( u8 txd );
u8 IIC_ReadByte( void );
u8 Read_Constant_Time( u8 Idx );
void Read_RTC_Time( void );
u8 Write_Constant_Time( u8 Idx, u8 Dat );
void Time_Read_Run( void );
void RTC_Time_Adj();//时间校准


#define   Years  0x16
#define   Month  0x15
#define   Day    0x14
#define   Week  0x13
#define   Hour  0x12
#define   Min  0x11
#define   Sec  0x10

#define EEPROMWRCMD 0xa0
#define EEPROMRDCMD 0xa1


#endif