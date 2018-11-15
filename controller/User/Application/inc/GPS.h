#ifndef __GPS_H
#define __GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "main.h"
#include "stdio.h"
#include "structural.h"
#include "stm32f4xx_it.h"

#include "uart_init.h"
#include "ringbuffer.h"

void GPS_Configure(void);
uint8_t GPS_IsReadyToRead(void);
void GPS_Update(void);
void GPS_DataDecode_GPGGA(u8 *buf);
void GPS_DataDecode_GPRMC(u8 *buf);
void GPS_DataDecode_HCHDM(u8 *buf);
uint16_t TxData_CheckSum_twoBytes(uint8_t *pData,uint8_t nLength);

uint8_t NMEA_Comma_Pos(u8 *buf,u8 cx);
uint32_t NMEA_Pow(u8 m,u8 n);
int NMEA_Str2num(u8 *buf,u8*dx);

void Time_UTC2BJ();

//extern uint8_t GPS_Connect_Flag;
extern RingBuffer  m_GPS_RX_RingBuff;
#ifdef __cplusplus
}
#endif
#endif