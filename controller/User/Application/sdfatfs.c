#include "sdfatfs.h"
#include <stdio.h>
#include "eeprom.h"
#include "math.h"
#include "IMU_Update.h"
#include "malloc.h"
#include "eeprom.h"
#include "GPS.h"
#include "Communication.h"
#include <string.h>

FATFS sdfs;
FIL file_rcd;//1024
FIL file_loc;//��������
FIL file_cfg;//config file

unsigned int Asc_To_Num(char *str);
uint8_t SDRam_Lack_Flag = 0;
char* ftype = ".txt"; 
char* ftype1 = ".log";

char slocal[30];  
char Routine[30];  
uint16_t Routine_file_reccnt = 0;
uint16_t local_file_reccnt = 0;
uint8_t log_time = 0;


char Device_Inf[10];
char Lat_File[10];
char Lon_File[10];

uint8_t send_time = 0;

uint16_t cycle_i = 0;

/*---------------------------- ��ȡcfg�ļ� -----------------------------------*/
#define Log_File_Length 512
#define _CFI_KWORD_NUMBER 8


const u8 CFI_keywords[_CFI_KWORD_NUMBER][20] =
{
  //  "#START\0",
  
  "#Lon\0",        /* ����Ŀ�꾭�ȣ�С�����6λ*/
  "#Lat\0",   /* ����Ŀ��γ�ȣ�С�����6λ */
  "#Senser-ID\0",       /* ��������� 00~99*/
  "#calculate-Mode\0",       /* ����ͳ�Ʒ�ʽ 1:100;2:1024*/
  "#Tradion-Sample\0",           /* 1����������۲⣻2���ر� */
  "#Time-Interval\0",      /* ������� ��λ��s */
  "#Original-Data\0",      /* ԭʼ���ݼ�¼ 1���ǣ�2���� */
  
  //  "#END\0",
};

  char  m_Log_Data_Buff[Log_File_Length];
FRESULT Read_CfgFile()
{

  u16 rdCnt = 0;
  
  /* mount */
  while(f_mount(&sdfs, "", 1) != FR_OK);//�����ļ�ϵͳʧ�ܣ��ȴ�ע��ɹ�
  
  f_open(&file_cfg, "TWS.cfg", FA_OPEN_EXISTING | FA_READ);
  
  f_read(&file_cfg,m_Log_Data_Buff,Log_File_Length,(UINT*)&rdCnt);    //��30���ֽڵ�����
  
  
  Get_Config_Data(m_Log_Data_Buff);
  
  f_close(&file_cfg);//�ر��ļ�
  f_mount(&sdfs, "", 0);//�رչ�����
  
  return FR_OK;
  
}


void Get_Config_Data(char *buf)
{
  u8 i;
  int sta_i,sta_j;
  u8 polling =1;
  sta_i=strstr_Length(buf,"#START");
  if(sta_i == 0) polling =0;
  
  
  while(polling)
  {
    sta_i = strstr_Length(buf,"\r\n");
    if(sta_i ==0) 
    {
      polling = 0;}
    buf += sta_i+1;
    
    if(i<_CFI_KWORD_NUMBER) 
    {
      sta_i = strstr_Length(buf, (char *)&CFI_keywords[i][0]);
      if(sta_i){
      sta_j = strchr_Length(buf, '=');
      if( i ==2){
      Parmeters_DecodeID(buf,sta_j,i);
      }else{
        Parmeters_Decode(buf+sta_j+1,i);
      }
      i++;
      }else{
       polling = 0;
      }
    }
  }
}
uint8_t strstr_Length(char *str, char *string)
{
  char *res;
  s8 buf_i;
  res = strstr(str,string);//�ж��Ƿ�������ͷ
  
  if(res == 0x00)
  {
    buf_i =0;
  }else{
    buf_i = res -str+1;
  }
  return buf_i;
}

uint8_t strchr_Length(char *str, char chr)
{
  char *res;
  s8 buf_i;
  res = strchr(str,chr);//�ж��Ƿ�������ͷ
  buf_i = res -str;
  return buf_i;
}

char ID_buffer[2];
uint8_t id_num;
void Parmeters_DecodeID(char *buf,uint8_t parmeters_j,uint8_t parmeters_i)
{
  id_num = strchr_Length(buf, '=');
  ID_buffer[0]=buf[id_num+2];
  ID_buffer[1] =buf[id_num+3];
  Device_Information.User_ID[3]  = ID_buffer[0];
  Device_Information.User_ID[4]  = ID_buffer[1];
}
  
void Parmeters_Decode(char *buf,uint8_t parmeters_i)
{

  switch(parmeters_i)
  {
  case 0:
    pGps_Data->Log_Lon = atof(buf);
    break;
    
  case 1:
    pGps_Data->Log_Lat = atof(buf);
    break;
    
//  case 2:
//    
////    parmeters_id=atof(buf);
////    Float2String_CharBuf(parmeters_id,ID_buffer,0);
////    if(atof(buf) < 10)
////    {
////      ID_buffer[1] = ID_buffer[0];
////      ID_buffer[0] = '0';
////      
////    }
//    id_num = strchr_Length(buf, '=');
//    ID_buffer[0]=buf[id_num+2];
//      ID_buffer[1] =buf[id_num+3];
//    Device_Information.User_ID[3]  = ID_buffer[0];
//    Device_Information.User_ID[4]  = ID_buffer[1];
//   
//    break;
    
  case 3:
    pHost_Cmd->Scan_Mode_Select = atoi(buf);
    if(pHost_Cmd->Scan_Mode_Select == 2)
    {
//      pRTC_Time->Routinue_year = pRTC_Time->year;
//      pRTC_Time->Routinue_month = pRTC_Time->mon;
//      pRTC_Time->Routinue_day = pRTC_Time->day;
//      pRTC_Time->Routinue_hour = pRTC_Time->hour;
//      pRTC_Time->Routinue_min = pRTC_Time->min;
//      pRTC_Time->Routinue_sec = pRTC_Time->sec;
    }
    break;
    
  case 4:
    pWave_Data->Tradion_Sample_Enable= atoi(buf);
    if(pWave_Data->Tradion_Sample_Enable ==1)//�����������ݹ۲�
    {
      pWave_Data->Tradion_Sample_Start =1;
        
      pRTC_Time->Interval_year = pRTC_Time->year;
      pRTC_Time->Interval_month = pRTC_Time->mon;
      pRTC_Time->Interval_day = pRTC_Time->day;
      pRTC_Time->Interval_hour = pRTC_Time->hour;
      pRTC_Time->Interval_min = pRTC_Time->min;
      pRTC_Time->Interval_sec = pRTC_Time->sec;
      
    }
    break;
    
  case 5:
    pHost_Cmd->Routine_Time_Spacing = atoi(buf);
    break;
    
  case 6:
    pHost_Cmd->OriData_Record_Flag = atoi(buf);
//    pSenser_Data->State = pSenser_Data->State
    break;
    
  default:
    break;
  }
}

/*----------------------�½�ԭʼ���ݼ�¼�ļ�----------------------------------*/
char dir_num[30] ={0};
FRESULT Add_LocalFile()
{
  FRESULT result;
  u8 retry = 50;
  DIR dirw;
  
  u8 year    =   (u8)pRTC_Time->year ;
  u8 month   =   (u8)pRTC_Time->mon;
  u8 day     =   (u8)pRTC_Time->day;
  u8 hour    =   (u8)pRTC_Time->hour;
  u8 minute  =   (u8)pRTC_Time->min;
  u8 second  =   (u8)pRTC_Time->sec;
  
  sprintf(dir_num,"%d-20%d",month,year);
  
  
  sprintf(Device_Inf,"%c%c%c%c%c%c-%c%c",Device_Information.Hardware_Version[1],Device_Information.Hardware_Version[3],
          Device_Information.Software_Version[1],Device_Information.Software_Version[3],
          Device_Information.Device_ID[1]       ,Device_Information.Device_ID[3],
          Device_Information.User_ID[3]         ,Device_Information.User_ID[4]);
  
  
  /* mount */
  while(f_mount(&sdfs, "", 1) != FR_OK);//�����ļ�ϵͳʧ�ܣ��ȴ�ע��ɹ�
  
  result = f_opendir(&dirw,dir_num);
  if(result != FR_OK)//�ļ��в�����
  {
    result = f_mkdir(dir_num);//�½����·�Ϊ���Ƶ��ļ�
  }
  
  sprintf(slocal,"%d_%d_%d_%d",day,hour,minute,second);
  strcat(slocal,ftype);
  
  strcat(dir_num,"/");
  strcat(dir_num,slocal);
  retry = 50;
  
  while(f_open(&file_loc, dir_num, FA_CREATE_ALWAYS | FA_READ | FA_WRITE) != FR_OK && --retry);
  if(retry==0)
    return FR_INT_ERR;
  else
  {     
    f_printf(&file_loc,"$$-------TWS_Original_Data-------$$");//original data
    
    f_printf(&file_loc,"\r\n$$-DeviceInformation:");//DeviceInformation
    f_printf(&file_loc,Device_Inf);//DeviceInformation
    
    
    f_printf(&file_loc,"\r\n$$-Data:");//Data
    sprintf(slocal,"20%d/%d/%d",year,month,day);
    f_printf(&file_loc,slocal);//Data
    
    f_printf(&file_loc,"\r\n$$-Location:");//Location
    sprintf(slocal,"%.6f",pGps_Data->Log_Lon);
    f_printf(&file_loc,slocal);
    
    f_printf(&file_loc,"   ");
    
    sprintf(slocal,"%.6f",pGps_Data->Log_Lat);
    f_printf(&file_loc,slocal);
    
    f_printf(&file_loc,"\r\n");
    f_printf(&file_loc,"\r\n");
    
    f_sync(&file_loc); 
    
    return FR_OK;    
    
  }
  
}


void Record_Data(FIL* f)
{
  char buffer[10];   
  
  char ori_time[10];
  char tws_time[10];
  
  sprintf(ori_time,"%2d:%2d:%2d",(u8)pRTC_Time->hour,(u8)pRTC_Time->min,(u8)pRTC_Time->sec);
  
  strcat(tws_time,"/");
  strcat(tws_time,"r");
  strcat(tws_time,"/");
  strcat(tws_time,"n");
  strcat(tws_time,ori_time);
  strcat(ori_time,": ");
  
  
  for(u8 i = 0;i<=11;i++)
  {
    
    switch(i)
    {
      
      /*-------------------------------- �����ʽ .txt --------------------*/
    case 0  :   f_printf(f,"%s","\r\nSenserData: ");     break;    
    
    //        case 0  :   f_printf(f,"\r\n%s",ori_time); break;
    
    /*---------------------------- �������˲�����    --------------------*/     
    case 1  :   //sprintf(buffer,"%.3f ",pSenser_Data->RollRate);
      Float2String_CharBuf(pSenser_Data->RollRate,buffer,3);break;
    case 2  :   //sprintf(buffer,"%.3f ",pSenser_Data->PitchRate);break;
      Float2String_CharBuf(pSenser_Data->PitchRate,buffer,3);break;
    case 3  :   //sprintf(buffer,"%.3f ",pSenser_Data->YawRate);break;
      Float2String_CharBuf(pSenser_Data->YawRate,buffer,3);break;
      
    case 4  :   //sprintf(buffer,"%.3f ",pSenser_Data->magX);break;
      Float2String_CharBuf(pSenser_Data->magX,buffer,3);break;
    case 5  :   //sprintf(buffer,"%.3f ",pSenser_Data->magY);break;
      Float2String_CharBuf(pSenser_Data->magY,buffer,3);break;
    case 6  :   //sprintf(buffer,"%.3f ",pSenser_Data->magZ);break;
      Float2String_CharBuf(pSenser_Data->magZ,buffer,3);break;
      
    case 7  :   //sprintf(buffer,"%.3f ",pSenser_Data->Xacc);break;
      Float2String_CharBuf(pSenser_Data->ICMAZ,buffer,3);//pSenser_Data->Xacc
      break;
    case 8  :   //sprintf(buffer,"%.3f ",pSenser_Data->Yacc);break;
      Float2String_CharBuf(pSenser_Data_filtered->Zaccf,buffer,3);//pSenser_Data->Yacc
      break;
    case 9  :   //sprintf(buffer,"%.3f ",pSenser_Data->Zacc);break;
      Float2String_CharBuf(pSenser_Data->Zacc,buffer,3);break;
      
    case 10  :   //sprintf(buffer,"%.1f ",pSenser_Data->Roll);break;
      Float2String_CharBuf(pSenser_Data->Roll,buffer,1);break;
    case 11  :   //sprintf(buffer,"%.1f ",pSenser_Data->Pitch);break;
      //          Float2String_CharBuf(pSenser_Data->Pitch,buffer,1);break;
      Float2String_CharBuf(pWave_Data->Height,buffer,1);break;
      
      
      
      
      //        /*----------------------------    ϵͳ״̬    ------------------------*/
      //        case 15  :   sprintf(buffer,"%d ",pSenser_Data->State);break;
      //        /*----------------------------    ��γ����Ϣ  ------------------------*/
      //        case 16  :   sprintf(buffer,"%.1f ",pGps_Data->Lon);break;
      //        case 17  :   sprintf(buffer,"%.1f ",pGps_Data->Lat);break;
      
      /*-------------------------------------- �����ʽ .csv --------------------*/
      //                        case 0  :   f_printf(f,"%s","\r\nSenserData:, ");     break;                       
      //                        case 1  :   sprintf(buffer,"%.3f, ",pSenser_Data->Roll);  break;
      //                        case 2  :   sprintf(buffer,"%.3f, ",pSenser_Data->Pitch); break;
      //                        case 3  :   sprintf(buffer,"%.3f, ",pSenser_Data->Yaw);   break;
      //                        /*---------------------------- BNO�������˲����� --------------------*/
      //                        case 4  :   sprintf(buffer,"%.3f, ",pSenser_Data->RollRate);break;
      //                        case 5  :   sprintf(buffer,"%.3f, ",pSenser_Data->PitchRate);break;
      //                        case 6  :   sprintf(buffer,"%.3f, ",pSenser_Data->YawRate);break;
      //                        
      //                        case 7  :   sprintf(buffer,"%.3f, ",pSenser_Data->Xacc);break;
      //                        case 8  :   sprintf(buffer,"%.3f, ",pSenser_Data->Yacc);break;
      //                        case 9  :   sprintf(buffer,"%.3f, ",pSenser_Data->Zacc);break;
      //                        
      //                        case 10  :   sprintf(buffer,"%.3f, ",pSenser_Data->magX);break;
      //                        case 11  :   sprintf(buffer,"%.3f, ",pSenser_Data->magY);break;
      //                        case 12  :   sprintf(buffer,"%.3f, ",pSenser_Data->magZ);break;
      //                         /*---------------------------- ADI�������˲����� --------------------*/
      //                        case 13  :   sprintf(buffer,"%.3f, ",pSenser_Data->Height);break;
      //                        case 14  :   sprintf(buffer,"%.3f, ",pSenser_Data->Cycle);break;
      //                        case 15  :   sprintf(buffer,"%.3f, ",pSenser_Data->State);break;
      
      //                    case 16  :   sprintf(buffer,"%.6f, ",pGps_Data->Lon);break;
      //                    case 17  :   sprintf(buffer,"%.6f, ",pGps_Data->Lat);break;
      
    default:break;  
    
    }
    if(i!=0)
      f_puts(buffer,f);
    memset(buffer,0,10);
    
    
  }
  //        f_sync(f);
  
  send_time++;
  
}

void Add_Ending_Message(FIL* f)
{
  
  u8 year    =   (u8)pRTC_Time->year ;
  u8 month   =   (u8)pRTC_Time->mon;
  u8 day     =   (u8)pRTC_Time->day;
  
  
  sprintf(Lon_File,"%.6f ",pGps_Data->Lon);
  sprintf(Lat_File,"%.6f ",pGps_Data->Lat);
  
  f_printf(&file_loc,"\r\n$$-------END-------$$");//file end
  
  f_printf(&file_loc,"\r\n$$-DeviceInformation:");//DeviceInformation
  f_printf(&file_loc,Device_Inf);//DeviceInformation
  
  
  f_printf(&file_loc,"\r\n$$-Data:");//Data
  sprintf(slocal,"20%d_%d_%d",year,month,day);
  f_printf(&file_loc,slocal);//Data
  
  f_printf(&file_loc,"\r\n$$-Lat:");//Location
  f_printf(&file_loc,Lat_File);
  f_printf(&file_loc,"$$-Lon:");
  f_printf(&file_loc,Lon_File);
}


unsigned int Asc_To_Num(char *str)
{
  unsigned int i = strlen(str);
  unsigned int num=0;
  unsigned int sum=0;
  for(int j=0;j<i;j++)
  {
    num = str[j]-48;//num = str[j]-��0��;           -48��-��0��     //�����ַ�ת����
    sum = num+sum*10;
  }
  return sum;
}



/**
* @brief  Get SDCard free on the SD
* @param  drv: pointer to drv
* @param  total: The toal of SDCard.
* @param  free: The free of SDCard.
* @retval The SD Response: 
*         - SD_RESPONSE_FAILURE: Sequence failed
*         - SD_RESPONSE_NO_ERROR: Sequence succeed
*/
//�õ�����ʣ������
//drv:���̱��("0:"/"1:")
//total:������	 ����λKB��
//free:ʣ������	 ����λKB��
//����ֵ:0,����.����,�������
FRESULT ISO_GetSDCardfree(uint8_t *drv,uint32_t *total,uint32_t *free)
{
  FATFS *fs1;
  FRESULT res;
  uint32_t fre_clust=0, fre_sect=0, tot_sect=0;
  //�õ�������Ϣ�����д�����
  res =f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs1);
  if(FR_OK==res)
  {											   
    tot_sect=(fs1->n_fatent-2)*fs1->csize;	//�õ���������
    fre_sect=fre_clust*fs1->csize;			//�õ�����������	   
#if _MAX_SS!=512				  				//������С����512�ֽ�,��ת��Ϊ512�ֽ�
    tot_sect*=fs1->ssize/512;
    fre_sect*=fs1->ssize/512;
#endif	  
    *total=tot_sect>>1;	//��λΪKB
    *free=fre_sect>>1;	//��λΪKB 
  }
  return res;
}	



FRESULT set_timestamp (
                       char *obj,     /* Pointer to the file name */
                       int year,
                       int month,
                       int mday,
                       int hour,
                       int min,
                       int sec
                         )
{
  FILINFO fno;
  
  fno.fdate = (WORD)(((year - 1980) * 512U) | month * 32U | mday);
  fno.ftime = (WORD)(hour * 2048U | min * 32U | sec / 2U);
  
  return f_utime(obj, &fno);
}


/*----------------------------  ������־�ļ����� -----------------------*/
char Log_time[30] ={0};
uint8_t week_num =0;

//  FRESULT result1;
FRESULT Creat_Log_File()
{
  
  //  f_sync(&file_loc); 
  //  f_close(&file_loc); 
  
   week_num =(u8)(((u8)pRTC_Time->day +7)/7);
  
  sprintf(Log_time,"%d-20%d",(u8)pRTC_Time->mon,(u8)pRTC_Time->year);//��־�ļ�������ʽ  ��-��-�� һ���ļ���¼һ�ܵ�����
  
  sprintf(Routine,"20%d-%d",(u8)pRTC_Time->year,week_num);
  strcat(Routine,ftype1);
  
  strcat(Log_time,"/");
  strcat(Log_time,Routine);
  
  u8 retry =50;
  //  result1 = f_open(&file_rcd,Log_time,FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
  while(f_open(&file_rcd,Log_time,FA_OPEN_ALWAYS | FA_READ | FA_WRITE)!=FR_OK && --retry);//�½��������ݼ�¼
  
  if (retry == 0)//(result1 != FR_OK) 
  {
    //    f_open(&file_loc, dir_num, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
    return FR_INT_ERR;
  }
  else
  {
    log_time = 0;
    f_printf(&file_rcd,"$$-------��������־-------$$");
    f_sync(&file_rcd); 
    //    f_close(&file_rcd);  
    
    //    f_open(&file_loc, dir_num, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
    return FR_OK; 
  }
  
}


char log_data_time[20];
void Record_Log_Data(FIL* f)
{
  //  FRESULT re;
  char rou_buff[5];
  
  
  //  re = f_open(&file_rcd, Log_time, FA_OPEN_EXISTING | FA_READ | FA_WRITE  );
  //  if(re == FR_OK)  
  //  {
  
  //      f_lseek(&file_rcd,57*log_time);//��һ�Σ�����Ҫ����ƫ��
  
  /*------------------------------��ʱ��Ϊ��ʶ----------------------------*/
  sprintf(log_data_time,"20%d/%2d/%2d %2d:%2d:%2d",(u8)pRTC_Time->year,(u8)pRTC_Time->mon,(u8)pRTC_Time->day,(u8)pRTC_Time->hour,(u8)pRTC_Time->min,(u8)pRTC_Time->sec);
  
  strcat(log_data_time,", ");
  switch(pHost_Cmd->Host_Data_Package)
  {
  case 1://mag cal
    for(u8 i = 0;i<=1;i++)
    {
      switch(i)
      {
        
        /*--------------------------------- �����ʽ .log --------------------*/
      case 0  :   f_printf(f,"\r\n%s",log_data_time); break;
      //      case 0  :   f_printf(f,"%s","\r\nRoutineData:, ");     break;     
      
      case 1  :   f_printf(f,"%s","MC");break;
      
      
      default:break;  
      
      }
      if(i!=0)
        f_puts(rou_buff,f);    
    }
    
    break;
    
  case 2://time adj
    for(u8 i = 0;i<=1;i++)
    {
      switch(i)
      {
        
        /*--------------------------------- �����ʽ .log --------------------*/
      case 0  :   f_printf(f,"\r\n%s",log_data_time); break;
      //      case 0  :   f_printf(f,"%s","\r\nRoutineData:, ");     break;     
      
      case 1  :  
        if(pHost_Cmd->Time_Adj_Mode == 1)
        {
          f_printf(f,"%s","TLC");
        }else if(pHost_Cmd->Time_Adj_Mode == 2)
        {
          f_printf(f,"%s","TGC");
        }break;
        
      default:break;  
      
      }
      if(i!=0)
        f_puts(rou_buff,f);    
    }
    
    break;
    
  case 3://�ı�������
    for(u8 i = 0;i<=2;i++)
    {
      switch(i)
      {
        
        /*--------------------------------- �����ʽ .log --------------------*/
      case 0  :   f_printf(f,"\r\n%s",log_data_time); break;
      //      case 0  :   f_printf(f,"%s","\r\nRoutineData:, ");     break;     
      
      case 1  :   f_printf(f,"%s","TSI");break;
      case 2  :   sprintf(rou_buff,"%d,",pHost_Cmd->Routine_Time_Spacing_pre);break; 
      
      default:break;  
      
      }
      if(i!=0)
        f_puts(rou_buff,f);    
    }
    
    break;
    
  case 4://��ȡ�������
    for(u8 i = 0;i<=11;i++)
    {
      switch(i)
      {
        
        /*--------------------------------- �����ʽ .log --------------------*/
      case 0  :   f_printf(f,"\r\n%s",log_data_time); break;
      //      case 0  :   f_printf(f,"%s","\r\nRoutineData:, ");     break;     
      case 1  :   f_printf(f,"%s,","WCR"); break;
      
      case 2  :   sprintf(rou_buff,"%2.1f,",pWave_Data->Max_Height);break;
      case 3  :   sprintf(rou_buff,"%2.1f,",pWave_Data->Max_Cycle); break;
      
      case 4  :   sprintf(rou_buff,"%2.1f,",pWave_Data->One_Tenth_Height);break;
      case 5  :   sprintf(rou_buff,"%2.1f,",pWave_Data->One_Tenth_Cycle); break;
      
      case 6  :   sprintf(rou_buff,"%2.1f,",pWave_Data->Effective_Height);break;
      case 7  :   sprintf(rou_buff,"%2.1f,",pWave_Data->Effective_Cycle); break;
      
      case 8  :   sprintf(rou_buff,"%2.1f,",pWave_Data->Average_Height);break;
      case 9  :   sprintf(rou_buff,"%2.1f,",pWave_Data->Average_Cycle); break;
      
      case 10  :   sprintf(rou_buff,"%3.1f,",pWave_Data->Main_heading);break;
      case 11 :   sprintf(rou_buff,"%3.1f,",pWave_Data->Num_Of_Wave); break;
      
      default:break;  
      
      }
      if(i!=0)
        f_puts(rou_buff,f);    
    }
    break;
    
  case 5://ԭʼ���ݼ�¼
    
    if(pHost_Cmd->OriData_Record_Flag == 1)//start
    {
      for(u8 i = 0;i<=1;i++)
      {
        switch(i)
        {
          
          /*--------------------------------- �����ʽ .log --------------------*/
        case 0  :   f_printf(f,"\r\n%s",log_data_time); break;
        //      case 0  :   f_printf(f,"%s","\r\nRoutineData:, ");     break;     
        
        case 1  :   f_printf(f,"%s","ODS");break;
        
        default:break;  
        
        }
        if(i!=0)
          f_puts(rou_buff,f);    
      }
    }else if(pHost_Cmd->OriData_Record_Flag == 2)//stop
    {
      char log_record_time[20];
      
      
      sprintf(log_record_time,"20%d/%2d/%2d %2d:%2d:%2d",(u8)pRTC_Time->Recording_Year,(u8)pRTC_Time->Recording_Month,(u8)pRTC_Time->Recording_Day,
              (u8)pRTC_Time->Recording_Hour,(u8)pRTC_Time->Recording_Minute,(u8)pRTC_Time->Recording_Second);
      
      strcat(log_record_time,", ");
      
      for(u8 i = 0;i<=1;i++)
      {
        switch(i)
        {
          
          /*--------------------------------- �����ʽ .log --------------------*/
        case 0  :   f_printf(f,"\r\n%s",log_record_time); break;
        //      case 0  :   f_printf(f,"%s","\r\nRoutineData:, ");     break;     
        
        case 1  :   f_printf(f,"%s","ODE");break;
        
        default:break;  
        
        }
        if(i!=0)
          f_puts(rou_buff,f);    
      }
    }
    break;
    
  case 7://ģʽѡ��
    for(u8 i = 0;i<=2;i++)
    {
      switch(i)
      {
        
        /*--------------------------------- �����ʽ .log --------------------*/
      case 0  :   f_printf(f,"\r\n%s",log_data_time); break;
      //      case 0  :   f_printf(f,"%s","\r\nRoutineData:, ");     break;     
      
      case 1  :   f_printf(f,"%s,","SCM");break;
      case 2  :   
        if(pHost_Cmd->Scan_Mode_Select == 1)
        {
          f_printf(f,"%s","2");break;
        }else if(pHost_Cmd->Scan_Mode_Select == 0)
        {
          f_printf(f,"%s","1");break;
        }
      default:break;  
      
      }
      if(i!=0)
        f_puts(rou_buff,f);    
    }
    break;
    
  case 8://�Ƿ�������۲�
    for(u8 i = 0;i<=2;i++)
    {
      switch(i)
      {
        
        /*--------------------------------- �����ʽ .log --------------------*/
      case 0  :   f_printf(f,"\r\n%s",log_data_time); break;
      //      case 0  :   f_printf(f,"%s","\r\nRoutineData:, ");     break;     
      
      case 1  :   f_printf(f,"%s,","TSS");break;
      
      case 2  :   
        if(pWave_Data->Tradion_Sample_Start == 1)
        {
          f_printf(f,"%s","1");break;
        }else if(pWave_Data->Tradion_Sample_Start == 0)
        {
          f_printf(f,"%s","0");break;
        }
        
      default:break;  
      
      }
      if(i!=0)
        f_puts(rou_buff,f);    
    }
    break;
    
  default:break;  
  }
  
  
  f_sync(f);
  //    f_close(&file_rcd);    
  log_time++;
  
  pHost_Cmd->Host_Data_Package = 0;
}


