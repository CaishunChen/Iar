#ifndef __COMMUNICATION_RECEIVE_H
#define __COMMUNICATION_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "main.h"
#include "stdio.h"
#include "structural.h"
#include "stm32f4xx_it.h"
#include "Safe_Check.h"
#include "uart_init.h"


void Communication_232_Configure(void);
uint8_t Communication_IsReadyToRead(void);
void Communication_Update(void);
void Communication_DataDecode(void);
uint16_t TxData_CheckSum_twoBytes(uint8_t *pData,uint8_t nLength);
void Data_Transmit_232(void);
uint8_t Choose_Package_RealTime_Data(u8* txbuf);

void _Decode_Data_Speed();
void _Decode_Effective_Height();
void _Decode_Compass_Cail();
void _Decode_Senser_Restar();
void _Decode_Restore_Settings();
void _Decode_OriData_Record();
void _Decode_Scan_Frequence();
void _Decode_Scan_Mode();
void _Decode_Time_Mode();//时间校准
void _Decode_Scan_Space();
void _Decode_Routine_CMD();
void _Decode_Tradion_Sample(u8 *buf);
void _Decode_RTD_CMD();

uint8_t Choose_Package_Data1(u8* txbuf);
uint8_t Choose_Package_Data2(u8* txbuf);
uint8_t Choose_Package_Data3(u8* txbuf);
uint8_t Choose_Package_Data4(u8* txbuf);
uint8_t Choose_Package_Data5(u8* txbuf);
uint8_t Choose_Package_Data7(u8* txbuf);
uint8_t Choose_Package_Data8(u8* txbuf);
uint8_t Choose_Package_Data10(u8* txbuf);//


void NMEA0183_Check_Sum(u8 *pData,u8 nLength);
uint8_t Get_Num_Length(u16 Data);
void float_to_string(float data, char *str,u8 n2);

  uint8_t Float2String_Buf(float data,uint8_t *str,uint8_t potlength,uint8_t num);
  void Float2String_CharBuf(float data,char *str,uint8_t potlength);

extern RingBuffer  m_Communication_RX_RingBuff;
extern d_Wave_Data  *pWave_Data;
extern d_RTC_Time  *pRTC_Time;
extern d_Gps_Data  *pGps_Data;
extern d_Senser_Data  *pSenser_Data;
extern d_Host_Cmd  *pHost_Cmd;
#ifdef __cplusplus
}
#endif
#endif