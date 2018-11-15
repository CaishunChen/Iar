/**
  ******************************************************************************
  * @file    ahrs.c
  * @author  UAV Application Team
  * @version V2.1.0
  * @date    2-September-2014
  * @brief   此文件主要定义导航算法函数
  ******************************************************************************  
  * @device  Master---STM32F405RGT6
  ******************************************************************************  
*/ 

#include "ahrs.h"

uint8_t     m_AHRS_RX_Buff[50];
RingBuffer  m_AHRS_RX_RingBuff;
uint8_t     m_AHRS_RX_WorkingBuff[APACKETSIZE];
uint8_t     m_AHRS_RX_WorkingBuff_DataCount = 0;
uint8_t     m_AHRS_RX_PreviousValue = END_BYTE;
uint8_t     ARxbuff[40];

d_IMU_Data  AHRS_Data_Temp;
d_IMU_Data  *pAHRS_Data_Temp=&AHRS_Data_Temp;
d_IMU_Data  AHRS_Data;
d_IMU_Data  *pAHRS_Data = &AHRS_Data;
uint8_t   AHRS_count=0;
void AHRS_Configure(void)
{
   UART_AHRS_init();
   rbInitialize(&m_AHRS_RX_RingBuff, m_AHRS_RX_Buff, sizeof(m_AHRS_RX_Buff));
}

uint8_t AHRS_IsReadyToRead(void)
{
  return !rbIsEmpty(&m_AHRS_RX_RingBuff);
}

void AHRS_Decode(void)
{ 
   uint8_t i;
  for(i=0;i<24;i++)
  {
     ARxbuff[i]=m_AHRS_RX_WorkingBuff[i];
  }
 
    AHRS_DataDecode();
}
void AHRS_Update(void)
{
  if(m_AHRS_RX_RingBuff.flagOverflow==1)
  {
    rbClear(&m_AHRS_RX_RingBuff);
  }
  while( !rbIsEmpty(&m_AHRS_RX_RingBuff) )  
  {
    uint8_t flagClear = ERROR;
    uint8_t cur = rbPop(&m_AHRS_RX_RingBuff);
    
    if( m_AHRS_RX_WorkingBuff_DataCount == 0 )    
    {
      
      m_AHRS_RX_WorkingBuff[0] = m_AHRS_RX_WorkingBuff[1];
      m_AHRS_RX_WorkingBuff[1] = m_AHRS_RX_WorkingBuff[2];
      m_AHRS_RX_WorkingBuff[2] = m_AHRS_RX_WorkingBuff[3];
      m_AHRS_RX_WorkingBuff[3] = cur;
      if((m_AHRS_RX_WorkingBuff[0]==START_BYTE_A) && (m_AHRS_RX_WorkingBuff[1]==START_BYTE_B)&& (m_AHRS_RX_WorkingBuff[2]==START_BYTE_C)&& (m_AHRS_RX_WorkingBuff[3]==START_BYTE_D)) 
        m_AHRS_RX_WorkingBuff_DataCount = 4;
    }
    else { 
      m_AHRS_RX_WorkingBuff[m_AHRS_RX_WorkingBuff_DataCount++] = cur;
    }
    
    if( m_AHRS_RX_WorkingBuff_DataCount == APACKETSIZE ) 
    {  
      if( m_AHRS_RX_WorkingBuff[APACKETSIZE-1] == END_BYTE ) 
      {
        AHRS_Decode();
        flagClear = SUCCESS;
      } 
      else {      
        flagClear = SUCCESS;
      }
    }
    
    m_AHRS_RX_PreviousValue = cur;
    
    // buffer clear & buffer over-run prevention
    if( flagClear || (m_AHRS_RX_WorkingBuff_DataCount >= sizeof(m_AHRS_RX_WorkingBuff)) ) 
    {
      m_AHRS_RX_WorkingBuff_DataCount = 0;
    }
  }
}

void AHRS_DataDecode(void)
{
    if (AHRS_CheckSum(ARxbuff+2,20)==ARxbuff[22])
    {
      if(AHRS_count==0)
      {
        AHRS_count=1;
        pAHRS_Data_Temp->magX = (u16)(ARxbuff[16]<<8|ARxbuff[17]);
        if(pAHRS_Data_Temp->magX>32768)
          pAHRS_Data_Temp->magX = -(pAHRS_Data_Temp->magX-32768);    
        pAHRS_Data->magX = (pAHRS_Data_Temp->magX)*(-0.00075)-(HIC_X);
        
        pAHRS_Data_Temp->magY = (u16)(ARxbuff[18]<<8|ARxbuff[19]);
        if(pAHRS_Data_Temp->magY>32768)
          pAHRS_Data_Temp->magY = -(pAHRS_Data_Temp->magY-32768);
        pAHRS_Data->magY = (pAHRS_Data_Temp->magY)*(-0.00075)-(HIC_Y);
        
        pAHRS_Data_Temp->magZ = (u16)(ARxbuff[20]<<8|ARxbuff[21]);
        if(pAHRS_Data_Temp->magZ>32768)
          pAHRS_Data_Temp->magZ = (-(pAHRS_Data_Temp->magZ-32768));
        pAHRS_Data->magZ = (pAHRS_Data_Temp->magZ)*(-0.00075);
      }
   
      pAHRS_Data_Temp->Xacc = (u16)(ARxbuff[4]<<8|ARxbuff[5]);
      if(pAHRS_Data_Temp->Xacc>32768)
        pAHRS_Data_Temp->Xacc = -(pAHRS_Data_Temp->Xacc-32768);
      pAHRS_Data->Xacc = -(pAHRS_Data_Temp->Xacc+5)*K_Xacc;
      
      pAHRS_Data_Temp->Yacc = (u16)(ARxbuff[6]<<8|ARxbuff[7]);
      if(pAHRS_Data_Temp->Yacc>32768)
        pAHRS_Data_Temp->Yacc = -(pAHRS_Data_Temp->Yacc-32768);
      pAHRS_Data->Yacc = -(pAHRS_Data_Temp->Yacc+190)*K_Yacc;
      
      pAHRS_Data_Temp->Zacc = (u16)(ARxbuff[8]<<8|ARxbuff[9]);
      if(pAHRS_Data_Temp->Zacc>32768)
        pAHRS_Data_Temp->Zacc = -(pAHRS_Data_Temp->Zacc-32768);
      pAHRS_Data->Zacc = -(pAHRS_Data_Temp->Zacc+400)*K_Zacc;
     
      /*
      pAHRS_Data_Temp->Xacc = (u16)(Rxbuff[4]<<8|Rxbuff[5]);
      if(pAHRS_Data_Temp->Xacc>32768)
        pAHRS_Data_Temp->Xacc = -(pAHRS_Data_Temp->Xacc-32768);
      pAHRS_Data->Xacc = pAHRS_Data_Temp->Xacc;
      
      pAHRS_Data_Temp->Yacc = (u16)(Rxbuff[6]<<8|Rxbuff[7]);
      if(pAHRS_Data_Temp->Yacc>32768)
        pAHRS_Data_Temp->Yacc = -(pAHRS_Data_Temp->Yacc-32768);
      pAHRS_Data->Yacc = pAHRS_Data_Temp->Yacc;
      
      pAHRS_Data_Temp->Zacc = (u16)(Rxbuff[8]<<8|Rxbuff[9]);
      if(pAHRS_Data_Temp->Zacc>32768)
        pAHRS_Data_Temp->Zacc = -(pAHRS_Data_Temp->Zacc-32768);
      pAHRS_Data->Zacc = pAHRS_Data_Temp->Zacc;
      */
      pAHRS_Data_Temp->RollRate = (u16)(ARxbuff[10]<<8|ARxbuff[11]);
      if(pAHRS_Data_Temp->RollRate>32768)
        pAHRS_Data_Temp->RollRate = -(pAHRS_Data_Temp->RollRate-32768);
      pAHRS_Data->RollRate = (pAHRS_Data_Temp->RollRate)*K_Rollrate;   //380
      
      pAHRS_Data_Temp->PitchRate= (u16)(ARxbuff[12]<<8|ARxbuff[13]);
      if(pAHRS_Data_Temp->PitchRate>32768)
        pAHRS_Data_Temp->PitchRate = -(pAHRS_Data_Temp->PitchRate-32768);
      pAHRS_Data->PitchRate = (pAHRS_Data_Temp->PitchRate)*K_Pitchrate;
      
      
      pAHRS_Data_Temp->YawRate  = (u16)(ARxbuff[14]<<8|ARxbuff[15]);
      if(pAHRS_Data_Temp->YawRate>32768)
        pAHRS_Data_Temp->YawRate = -(pAHRS_Data_Temp->YawRate-32768);
      pAHRS_Data->YawRate = (pAHRS_Data_Temp->YawRate)*K_Yawrate;   //500
      
      pAHRS_Data_Temp->magX = (u16)(ARxbuff[16]<<8|ARxbuff[17]);
      if(pAHRS_Data_Temp->magX>32768)
        pAHRS_Data_Temp->magX = -(pAHRS_Data_Temp->magX-32768);
      
      //if(pAHRS_Data->magX-((pAHRS_Data_Temp->magX)*(-0.00075)-(HIC_X))<=0.1 && pAHRS_Data->magX-((pAHRS_Data_Temp->magX)*(-0.00075)-(HIC_X))>=-0.1)    
        pAHRS_Data->magX = (pAHRS_Data_Temp->magX)*(-0.00075)-(HIC_X);
      
      pAHRS_Data_Temp->magY = (u16)(ARxbuff[18]<<8|ARxbuff[19]);
      if(pAHRS_Data_Temp->magY>32768)
        pAHRS_Data_Temp->magY = -(pAHRS_Data_Temp->magY-32768);
      
      //if(pAHRS_Data->magY-((pAHRS_Data_Temp->magY)*(-0.00075)-(HIC_Y))<=0.1 && pAHRS_Data->magY-((pAHRS_Data_Temp->magY)*(-0.00075)-(HIC_Y))>=-0.1)    
        pAHRS_Data->magY = (pAHRS_Data_Temp->magY)*(-0.00075)-(HIC_Y);
      
      
      pAHRS_Data_Temp->magZ = (u16)(ARxbuff[20]<<8|ARxbuff[21]);
      if(pAHRS_Data_Temp->magZ>32768)
        pAHRS_Data_Temp->magZ = (-(pAHRS_Data_Temp->magZ-32768));
      
      //if(pAHRS_Data->magZ-((pAHRS_Data_Temp->magZ)*(-0.00075))<=0.1 && pAHRS_Data->magZ-((pAHRS_Data_Temp->magZ)*(-0.00075))>=-0.1)    
        pAHRS_Data->magZ = (pAHRS_Data_Temp->magZ)*(-0.00075) - (HIC_Z);
      }
    }
 
uint8_t AHRS_CheckSum(u8 *pData,u8 nLength)
{
  u8 checksun=0;
  for(u8 i=0;i<nLength;i++) checksun += ((u16)pData[i] & 0xFF);  
  return checksun;
}

/************************ (C) COPYRIGHT AOYIWEIYENG *****END OF FILE***********/
