#include "GPS.h"
#include "string.h"	 

uint8_t     m_GPS_RX_Buff[100];
RingBuffer  m_GPS_RX_RingBuff;
uint8_t     m_GPS_RX_WorkingBuff[100];
uint8_t     m_GPS_RX_WorkingBuff_DataCount = 0;


d_Gps_Data Gps_Data;
d_Gps_Data *pGps_Data = &Gps_Data;

extern d_Senser_Data  *pSenser_Data;


void GPS_Configure(void)
{
  UART_GPS_init();
  rbInitialize(&m_GPS_RX_RingBuff, m_GPS_RX_Buff, sizeof(m_GPS_RX_Buff));
}

uint8_t GPS_IsReadyToRead(void)
{
  return !rbIsEmpty(&m_GPS_RX_RingBuff);
}

/*
GPS_Data_Package:
1: GPRMC
2. GPGGA
3. HCHDM
*/
uint8_t GPS_Data_Package = 0;
uint8_t Gps_cnt = 0;
void GPS_Update(void)
{
  if(m_GPS_RX_RingBuff.flagOverflow == 1)
  {
    rbClear(&m_GPS_RX_RingBuff);
  }
  while( !rbIsEmpty(&m_GPS_RX_RingBuff) )  
  {
    uint8_t cur = rbPop(&m_GPS_RX_RingBuff);
    if( m_GPS_RX_WorkingBuff_DataCount == 0 )    
    {
      m_GPS_RX_WorkingBuff[0] = m_GPS_RX_WorkingBuff[1];
      m_GPS_RX_WorkingBuff[1] = m_GPS_RX_WorkingBuff[2];
      m_GPS_RX_WorkingBuff[2] = m_GPS_RX_WorkingBuff[3];
      m_GPS_RX_WorkingBuff[3] = m_GPS_RX_WorkingBuff[4];
      m_GPS_RX_WorkingBuff[4] = m_GPS_RX_WorkingBuff[5];
      m_GPS_RX_WorkingBuff[5] = m_GPS_RX_WorkingBuff[6];
      m_GPS_RX_WorkingBuff[6] = cur;
      if(m_GPS_RX_WorkingBuff[0] == 0X24 && m_GPS_RX_WorkingBuff[1] == 0X47 && m_GPS_RX_WorkingBuff[2] == 0X50 && m_GPS_RX_WorkingBuff[3] == 0X52 && m_GPS_RX_WorkingBuff[5] == 0X43 && m_GPS_RX_WorkingBuff[6] == 0X2C)
      {
        GPS_Data_Package = 1;
        m_GPS_RX_WorkingBuff_DataCount = 7; 
      }   
      if(m_GPS_RX_WorkingBuff[0] == 0X24 && m_GPS_RX_WorkingBuff[1] == 0X47 && m_GPS_RX_WorkingBuff[2] == 0X50 && m_GPS_RX_WorkingBuff[3] == 0X47 && m_GPS_RX_WorkingBuff[5] == 0X41 && m_GPS_RX_WorkingBuff[6] == 0X2C)
      {
        GPS_Data_Package = 2;
        m_GPS_RX_WorkingBuff_DataCount = 7; 
      }  
//      if(m_GPS_RX_WorkingBuff[0] == 0X24 && m_GPS_RX_WorkingBuff[1] == 0X48 && m_GPS_RX_WorkingBuff[2] == 0X43 && m_GPS_RX_WorkingBuff[3] == 0X48 && m_GPS_RX_WorkingBuff[5] == 0X4D && m_GPS_RX_WorkingBuff[6] == 0X2C)
//      {
//        GPS_Data_Package = 3;
//        m_GPS_RX_WorkingBuff_DataCount = 7; 
//      }   
    }
    else 
    { 
      m_GPS_RX_WorkingBuff[m_GPS_RX_WorkingBuff_DataCount++] = cur;
      if((m_GPS_RX_WorkingBuff[m_GPS_RX_WorkingBuff_DataCount-1] == 0X0A)&&(m_GPS_RX_WorkingBuff[m_GPS_RX_WorkingBuff_DataCount-2] == 0X0D))
      {
               
        if(GPS_Data_Package == 1)
        {
          GPS_DataDecode_GPRMC(m_GPS_RX_WorkingBuff);
          m_GPS_RX_WorkingBuff_DataCount = 0;
          memset(m_GPS_RX_WorkingBuff,0,Gps_cnt);
        } 
        else if(GPS_Data_Package == 2)
        {
          GPS_DataDecode_GPGGA(m_GPS_RX_WorkingBuff);
          m_GPS_RX_WorkingBuff_DataCount = 0;
          memset(m_GPS_RX_WorkingBuff,0,Gps_cnt);
        }
//        else if(GPS_Data_Package == 3)
//        {
//          GPS_DataDecode_HCHDM(m_GPS_RX_WorkingBuff);
//          m_GPS_RX_WorkingBuff_DataCount = 0;
//          memset(m_GPS_RX_WorkingBuff,0,Gps_cnt);
//        }
        

      }
      if(m_GPS_RX_WorkingBuff_DataCount > 99)
      {
        m_GPS_RX_WorkingBuff_DataCount = 0;
      }
      
    }
    
  }
}

void GPS_DataDecode_GPGGA(u8 *buf)
{
  uint8_t dx=0;			 
  u8 posx;    
  u32 temp;
  posx=NMEA_Comma_Pos(buf,1);		//UTC时间					
  if(posx!=0XFF)
  {
    temp=NMEA_Str2num(buf+posx,&dx)/NMEA_Pow(10,dx);	 
    pGps_Data->UTC_Hour=temp/10000;
    pGps_Data->UTC_Minute=(temp/100)%100;
    pGps_Data->UTC_Second=temp%100;	 	 
  }
  posx=NMEA_Comma_Pos(buf,7);	     //星数
  if(posx!=0XFF) pGps_Data->Star_Num=NMEA_Str2num(buf+posx,&dx); 
  posx=NMEA_Comma_Pos(buf,9);			//高度					
  if(posx!=0XFF) pGps_Data->GPS_Alt=NMEA_Str2num(buf+posx,&dx);  
  
    Time_UTC2BJ(); //时间转换
  
}

/*
GPS_Connect_Flag 
0:未接入GPS
1:接入GPS 无卫星
2;接入GPS 有卫星
*/
//uint8_t GPS_Connect_Flag = 0;

void GPS_DataDecode_GPRMC(u8 *buf)
{
  uint8_t dx=0;
  u8 posx;     
  u32 temp;	   
  float rs;  
  
  posx=NMEA_Comma_Pos(buf,2);              //状态  A=定位，V=未定位
  if(posx!=0XFF) 
  {
    pGps_Data->State = m_GPS_RX_WorkingBuff[posx];//NMEA_Str2num(buf+posx,&dx);
    
    if( pGps_Data->State == 0X41)
    {
//      GPS_Connect_Flag = 2;
      
      pSenser_Data->State = pSenser_Data->State | 0x0C;
    }else
    {
//      GPS_Connect_Flag = 1;
      pSenser_Data->State = pSenser_Data->State | 0x08;
    }
  }
  posx=NMEA_Comma_Pos(buf,3);		//纬度						
  if(posx!=0XFF)
  {
    temp=NMEA_Str2num(buf+posx,&dx);		 	 
//    pGps_Data->Lat=temp/NMEA_Pow(10,dx+2);	
//    rs=temp%NMEA_Pow(10,dx+2);						 
//    pGps_Data->Lat=pGps_Data->Lat*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;
    pGps_Data->Lat=temp/NMEA_Pow(10,dx);	
    rs=temp%NMEA_Pow(10,dx);						 
    pGps_Data->Lat=pGps_Data->Lat+rs/100000;
    
  }
  posx=NMEA_Comma_Pos(buf,5);		//经度						
  if(posx!=0XFF)
  {												  
    temp=NMEA_Str2num(buf+posx,&dx);		 	 
//    pGps_Data->Lon=temp/NMEA_Pow(10,dx+2);	
//    rs=temp%NMEA_Pow(10,dx+2);						 
//    pGps_Data->Lon=pGps_Data->Lon*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;
        pGps_Data->Lon=temp/NMEA_Pow(10,dx);	
    rs=temp%NMEA_Pow(10,dx);						 
    pGps_Data->Lon=pGps_Data->Lon+rs/100000;
  }
  
  posx=NMEA_Comma_Pos(buf,9);		//UTC日期						
  if(posx!=0XFF)
  {
    temp=NMEA_Str2num(buf+posx,&dx);		 				
    pGps_Data->UTC_Day=temp/10000;
    pGps_Data->UTC_Month=(temp/100)%100;
    pGps_Data->UTC_Year=2000+temp%100;	 	 
  } 
 
}


void GPS_DataDecode_HCHDM(u8 *buf)
{
  uint8_t dx=0;
  u8 posx;     
  posx=NMEA_Comma_Pos(buf,1);								
  if(posx!=0XFF)
  {
    pGps_Data->Heading=NMEA_Str2num(buf+posx,&dx);
  } 
  

}

//ReadData "$UDWCR,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.,0*79\r\n\r\n"


uint8_t NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
  u8 *p=buf;
  while(cx)
  {		 
    if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//ó?μ?'*'?ò??·?·¨×?·?,?ò2?′??úμúcx???oo?
    if(*buf==',')cx--;
    buf++;
  }
  return buf-p;	 
}

uint32_t NMEA_Pow(u8 m,u8 n)
{
  u32 result=1;	 
  while(n--)result*=m;    
  return result;
}

int NMEA_Str2num(u8 *buf,u8*dx)
{
  u8 *p=buf;
  u32 ires=0,fres=0;
  u8 ilen=0,flen=0,i;
  u8 mask=0;
  int res;
  while(1) //μ?μ???êyoíD?êyμ?3¤?è
  {
    if(*p=='-'){mask|=0X02;p++;}//ê??oêy
    if(*p==','||(*p=='*'))break;//ó?μ??áê?á?
    if(*p=='.'){mask|=0X01;p++;}//ó?μ?D?êyμ?á?
    else if(*p>'9'||(*p<'0'))	//óD·?·¨×?·?
    {	
      ilen=0;
      flen=0;
      break;
    }	
    if(mask&0X01)flen++;
    else ilen++;
    p++;
  }
  if(mask&0X02)buf++;	//è￥μ??oo?
  for(i=0;i<ilen;i++)	//μ?μ???êy2?·?êy?Y
  {  
    ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
  }
  if(flen>5)flen=5;	//×??àè?5??D?êy
  *dx=flen;	 		//D?êyμ???êy
  for(i=0;i<flen;i++)	//μ?μ?D?êy2?·?êy?Y
  {  
    fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
  } 
  res=ires*NMEA_Pow(10,flen)+fres;
  if(mask&0X02)res=-res;		   
  return res;
}


  /*---------------------------   UIC>>北京时间   ----------------------------*/

void Time_UTC2BJ()
{
  pGps_Data->Beijing_Year   = pGps_Data->UTC_Year; 
  pGps_Data->Beijing_Month  = pGps_Data->UTC_Month;
  pGps_Data->Beijing_Day    = pGps_Data->UTC_Day; 
  pGps_Data->Beijing_Hour   = pGps_Data->UTC_Hour + 8; 
  pGps_Data->Beijing_Minute = pGps_Data->UTC_Minute; 
  pGps_Data->Beijing_Second = pGps_Data->UTC_Second; 
  if (pGps_Data->Beijing_Hour >= 24)  
  {  
    pGps_Data->Beijing_Hour-=24;  
    pGps_Data->Beijing_Day++;  
    switch (pGps_Data->Beijing_Month)  
    {  
    case 1:
    case 3:
    case 5:
    case 7:
    case 8:
    case 10:
      if (pGps_Data->Beijing_Day > 31)  
      {  
        pGps_Data->Beijing_Day = 1;  
        pGps_Data->Beijing_Month++;  
      }  
      break;  
    case 2:  
      if (((0==pGps_Data->Beijing_Year%4) && (0!=pGps_Data->Beijing_Year%100)) || (0==pGps_Data->Beijing_Year%400))  
      {  
        if (pGps_Data->Beijing_Day > 29)  
        {  
          pGps_Data->Beijing_Day = 1;  
          pGps_Data->Beijing_Month++;  
        }  
      }  
      else  
      {  
        if (pGps_Data->Beijing_Day > 28)  
        {  
          pGps_Data->Beijing_Day = 1;  
          pGps_Data->Beijing_Month++;    
        }  
      }  
      break;  
    case 4:
    case 6:
    case 9:
    case 11:
      if (pGps_Data->Beijing_Day > 30)  
      {  
        pGps_Data->Beijing_Day = 1;  
        pGps_Data->Beijing_Month++;  
      }  
      break;  
    case 12:  
      if (pGps_Data->Beijing_Day > 31)  
      {  
        pGps_Data->Beijing_Day = 1;  
        pGps_Data->Beijing_Month++;
        pGps_Data->Beijing_Year++;
      }  
      break; 
    default:break;  
    }  
  }  
}
