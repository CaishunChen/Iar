#include "Communication_OriData.h"
#include "filter.h" 


uint8_t Communicate_OriData_TxData[30];


void Communication_OriData_Configure()
{
  UART_Communication_OriData_init();
}


void OriData__Communication_Send()
{
  Communication_OriData_Send(Communicate_OriData_TxData, Choose_Package_OriData(Communicate_OriData_TxData));
//  GPS_Send(Communicate_OriData_TxData,0x01);
}

uint8_t Choose_Package_OriData(u8* txbuf)
{
    uint16_t sTemp = 0;
    uint16_t data_totalnumber = 0;
    uint16_t Checksum_TwoBytes = 0;
     
    txbuf[data_totalnumber++] = (uint8_t)0xB5;
    txbuf[data_totalnumber++] = (uint8_t)0x5B;
    txbuf[data_totalnumber++] = (uint8_t)0xFF;                                   //Data_length
    
    /*--------------------------- Time 1-3 -----------------------------------*/
    sTemp = (s16)((pRTC_Time->year<<8)|pRTC_Time->mon);
    txbuf[data_totalnumber++]  = (uint8_t)((sTemp&0xFF00)>>8);
    txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
    
    sTemp = (s16)((pRTC_Time->day<<8)|pRTC_Time->hour);
    txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8); 
    txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);
    sTemp = (s16)((pRTC_Time->min<<8)|pRTC_Time->sec);
    txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8); 
    txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);
    
    /*--------------------------- 加速度  ------------------------------------*/
    sTemp = (s16)(pSenser_Data->Xacc*1000.0); // acc
    txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8); 
    txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
    
    sTemp = (s16)(pSenser_Data->Yacc*1000.0); // acc
    txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8); 
    txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
    
    sTemp = (s16)(pSenser_Data->Zacc*1000.0); // acc
    txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8); 
    txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
        
    /*--------------------------- 角速度  ------------------------------------*/
    sTemp = (s16)(pSenser_Data->RollRate*1000.0); // gyro
    txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8); 
    txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
    
    sTemp = (s16)(pSenser_Data->PitchRate*1000.0); // gyro
    txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8); 
    txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
    
    sTemp = (s16)(pSenser_Data->YawRate*1000.0); // gyro
    txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8); 
    txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
    
    /*--------------------------- 地磁场  ------------------------------------*/
    sTemp = (s16)(pSenser_Data->magX*1000.0); // mag
    txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8); 
    txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
    
    sTemp = (s16)(pSenser_Data->magY*1000.0); // mag
    txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8); 
    txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
    
    sTemp = (s16)(pSenser_Data->magZ*1000.0); // mag
    txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8); 
    txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
    
    /*------------------------------ Check_Sum -------------------------------*/
    txbuf[2] = (uint8_t)(data_totalnumber+2);
    Checksum_TwoBytes = TxData_CheckSum_twoBytes(txbuf+2, txbuf[2]-4);
    txbuf[data_totalnumber++]  = (u8)((Checksum_TwoBytes >> 8) & 0x00FF);
    txbuf[data_totalnumber++]  = (u8)(Checksum_TwoBytes & 0x00FF);
    return data_totalnumber;    
}