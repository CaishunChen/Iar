#ifndef __MASTER_I2C_H__
#define __MASTER_I2C_H__

#include "stm32f4xx.h"

#define SCL_H         (GPIO_SetBits(GPIOE,GPIO_Pin_15))
#define SCL_L         (GPIO_ResetBits(GPIOE,GPIO_Pin_15)) 
    
#define SDA_H          (GPIO_SetBits(GPIOE,GPIO_Pin_13))
#define SDA_L          (GPIO_ResetBits(GPIOE,GPIO_Pin_13))

#define SCL_read      (GPIOE->IDR&GPIO_Pin_15) 
#define SDA_read      (GPIOE->IDR&GPIO_Pin_13)

#define SDA_IN()	GPIOE->MODER|=GPIO_Mode_IN<<22;  //sdaÊäÈë
#define SDA_OUT()       GPIOE->MODER|=GPIO_Mode_OUT<<22;//sdaÊä³ö


#define ICM_I2C_ADDR 0xD0
void I2C_Init_IO(void);       

void I2C_Start(void);	

void I2C_Stop(void);	  			

u8 I2C_WaitAck(void); 	

void I2C_Ack(void);

void I2C_NoAck(void);				

void I2C_SendByte(u8 SendByte);	

u8 I2C_ReadByte(void);            

u8 Single_Write(u8 REG_Address,u8 REG_data,u8 SlaveAddress);

u8 Single_Read(u8 REG_Address,u8 SlaveAddress);

void Multiple_write(u8 star_addr,u8 num,u8 SlaveAddress,u8* send_buf);

void Multiple_read(u8 star_addr,u8 num,u8 SlaveAddress,u8* recv_buf);

void i2c_master_write_register(u8 SlaveAddress,u8 RegisterAddr,u8 RegisterLen,u8* send_buf);

void i2c_master_read_register(u8 SlaveAddress,u8 RegisterAddr,u8 RegisterLen,u8* recv_buf);

void io_haldware_reg_write(uint8_t reg, uint8_t *buf, uint16_t buf_size);

void write_1B(uint8_t reg, uint8_t one_data);

void io_haldware_reg_read(uint8_t reg, uint8_t *buf, uint16_t buf_size);

void read_1B(uint8_t reg,uint8_t *temp);

void i2cRead(u8 SlaveAddress,u8 star_addr,u8 num,u8* recv_buf);

void i2cWrite(u8 SlaveAddress,u8 REG_Address,u8 REG_data);

#endif


