#include "Communication.h"
#include "GPS.h"
#include "Safe_Check.h"
#include "stm32f4xx_it.h"
#include "sdfatfs.h"
#include "IMU_Update.h"
#include "GPS.h"
#include "eeprom.h"

d_Host_Cmd Host_Cmd;
d_Host_Cmd* pHost_Cmd = &Host_Cmd;

uint8_t     m_Communication_RX_Buff[50];
RingBuffer  m_Communication_RX_RingBuff;
uint8_t     m_Communication_RX_WorkingBuff[50];
uint8_t     m_Communication_RX_WorkingBuff_DataCount = 0;

uint8_t Communicate_TxData[50];





void Communication_232_Configure( void )
{
    UART_Communication_init();
    rbInitialize( &m_Communication_RX_RingBuff, m_Communication_RX_Buff, sizeof( m_Communication_RX_Buff ) );
}

uint8_t Communication_IsReadyToRead( void )
{
    return !rbIsEmpty( &m_Communication_RX_RingBuff );
}


void Communication_Update( void )
{
    if( m_Communication_RX_RingBuff.flagOverflow == 1 )
    {
        rbClear( &m_Communication_RX_RingBuff );
    }
    while( !rbIsEmpty( &m_Communication_RX_RingBuff ) )
    {
        uint8_t flagClear = ERROR;
        uint8_t cur = rbPop( &m_Communication_RX_RingBuff );
        if( m_Communication_RX_WorkingBuff_DataCount == 0 )
        {
            m_Communication_RX_WorkingBuff[0] = m_Communication_RX_WorkingBuff[1];
            m_Communication_RX_WorkingBuff[1] = m_Communication_RX_WorkingBuff[2];
            m_Communication_RX_WorkingBuff[2] = m_Communication_RX_WorkingBuff[3];
            m_Communication_RX_WorkingBuff[3] = m_Communication_RX_WorkingBuff[4];
            m_Communication_RX_WorkingBuff[4] = m_Communication_RX_WorkingBuff[5];
            m_Communication_RX_WorkingBuff[5] = cur;
            if( m_Communication_RX_WorkingBuff[0] == 0X24 && m_Communication_RX_WorkingBuff[1] == 0X53 && m_Communication_RX_WorkingBuff[2] == 0X44 ) //����ͷ $SD
            {
                if( ( m_Communication_RX_WorkingBuff[3] == 0X4d ) && ( m_Communication_RX_WorkingBuff[4] == 0X53 ) && ( m_Communication_RX_WorkingBuff[5] == 0X43 ) ) //MSA
                {
                    pHost_Cmd->Host_Data_Package = 1;//����У׼
                }
                if( ( m_Communication_RX_WorkingBuff[3] == 0X53 ) && ( m_Communication_RX_WorkingBuff[4] == 0X54 ) && ( m_Communication_RX_WorkingBuff[5] == 0X41 ) ) //STA
                {
                    pHost_Cmd->Host_Data_Package = 2;//ʱ��У׼ GPS������ʱ��
                }
                if( ( m_Communication_RX_WorkingBuff[3] == 0X4f ) && ( m_Communication_RX_WorkingBuff[4] == 0X44 ) && ( m_Communication_RX_WorkingBuff[5] == 0X54 ) )
                {
                    pHost_Cmd->Host_Data_Package = 5;//ԭʼ�����Ƿ��¼
                }
                if( ( m_Communication_RX_WorkingBuff[3] == 0X53 ) && ( m_Communication_RX_WorkingBuff[4] == 0X53 ) && ( m_Communication_RX_WorkingBuff[5] == 0X52 ) )
                {
                    pHost_Cmd->Host_Data_Package = 6;//��λ
                }
                if( ( m_Communication_RX_WorkingBuff[3] == 0X53 ) && ( m_Communication_RX_WorkingBuff[4] == 0X4D ) && ( m_Communication_RX_WorkingBuff[5] == 0X53 ) ) //SMS
                {
                    pHost_Cmd->Host_Data_Package = 7;//����۲�ģʽѡ�� 1024s/100��
                }
                if( ( m_Communication_RX_WorkingBuff[3] == 0X52 ) && ( m_Communication_RX_WorkingBuff[4] == 0X54 ) && ( m_Communication_RX_WorkingBuff[5] == 0X44 ) ) //RTD
                {
                    pHost_Cmd->Host_Data_Package = 9;//�Ƿ���ʵʱ���ݴ���
                }
                if( ( m_Communication_RX_WorkingBuff[3] == 0X47 ) && ( m_Communication_RX_WorkingBuff[4] == 0X53 ) && ( m_Communication_RX_WorkingBuff[5] == 0X43 ) ) //GSC
                {
                    pHost_Cmd->Host_Data_Package = 10;//��������ͬ��
                }
                m_Communication_RX_WorkingBuff_DataCount = 6;
            }
            else if( m_Communication_RX_WorkingBuff[0] == 0X24 && m_Communication_RX_WorkingBuff[1] == 0X55 && m_Communication_RX_WorkingBuff[2] == 0X44 ) //����ͷ $UD
            {
                if( ( m_Communication_RX_WorkingBuff[3] == 0X54 ) && ( m_Communication_RX_WorkingBuff[4] == 0X53 ) && ( m_Communication_RX_WorkingBuff[5] == 0X49 ) ) //TSI
                {
                    pHost_Cmd->Host_Data_Package = 3;//�ı䳣�����ݲɼ����
                }
                if( ( m_Communication_RX_WorkingBuff[3] == 0X57 ) && ( m_Communication_RX_WorkingBuff[4] == 0X45 ) && ( m_Communication_RX_WorkingBuff[5] == 0X44 ) ) //WED
                {
                    pHost_Cmd->Host_Data_Package = 4;//��ȡ����۲���
                }
                if( ( m_Communication_RX_WorkingBuff[3] == 0X54 ) && ( m_Communication_RX_WorkingBuff[4] == 0X53 ) && ( m_Communication_RX_WorkingBuff[5] == 0X53 ) ) //TSS,tradion sample start
                {
                    pHost_Cmd->Host_Data_Package = 8;//�Ƿ�������۲�
                }
                m_Communication_RX_WorkingBuff_DataCount = 6;
            }
        }
        else
        {
            m_Communication_RX_WorkingBuff[m_Communication_RX_WorkingBuff_DataCount++] = cur;
            if( ( m_Communication_RX_WorkingBuff[m_Communication_RX_WorkingBuff_DataCount - 2] == 0x0D ) && ( m_Communication_RX_WorkingBuff[m_Communication_RX_WorkingBuff_DataCount - 1] == 0x0A ) )
            {
                Communication_DataDecode();
                flagClear = SUCCESS;
                memset( m_Communication_RX_WorkingBuff, 0, 50 );
            }
            if( flagClear || ( m_Communication_RX_WorkingBuff_DataCount >= 50 ) )
            {
                m_Communication_RX_WorkingBuff_DataCount = 0;
                memset( m_Communication_RX_WorkingBuff, 0, 50 );
            }
        }
    }
}

void Communication_DataDecode()
{
    pHost_Cmd->Receive_CMD_Flag = 1;
    if( pHost_Cmd->Host_Data_Package == 1 )
    {
        _Decode_Compass_Cail( m_Communication_RX_WorkingBuff ); //����У׼
    }
    if( pHost_Cmd->Host_Data_Package == 2 )
    {
        _Decode_Time_Mode();//ʱ��У׼
    }
    if( pHost_Cmd->Host_Data_Package == 3 )
    {
        _Decode_Scan_Space( m_Communication_RX_WorkingBuff ); //�ı䳣�����ݲɼ����
    }
    if( pHost_Cmd->Host_Data_Package == 4 )
    {
        _Decode_Routine_CMD( m_Communication_RX_WorkingBuff ); //����۲�����ȡָ��
    }
    if( pHost_Cmd->Host_Data_Package == 5 )
    {
        _Decode_OriData_Record();//ԭʼ�����Ƿ��¼
    }
    if( pHost_Cmd->Host_Data_Package == 6 )
    {
        _Decode_Senser_Restar( m_Communication_RX_WorkingBuff ); //��λ
    }
    if( pHost_Cmd->Host_Data_Package == 7 )
    {
        _Decode_Scan_Mode( m_Communication_RX_WorkingBuff ); //����۲�ģʽѡ��
    }
    if( pHost_Cmd->Host_Data_Package == 8 )
    {
        _Decode_Tradion_Sample( m_Communication_RX_WorkingBuff ); //�Ƿ�������۲�
    }
    if( pHost_Cmd->Host_Data_Package == 9 )
    {
        _Decode_RTD_CMD( m_Communication_RX_WorkingBuff ); //�Ƿ���ʵʱ���ݴ���
    }
}


void _Decode_Compass_Cail( u8* buf ) //�ش�У׼
{
    u8 posx;
    uint8_t dx = 0;
    posx = NMEA_Comma_Pos( buf, 1 );         //���ʱ��
    if( posx != 0XFF )
    {
        pHost_Cmd->Senser_Cail_CMD = NMEA_Str2num( buf + posx, &dx );;
    }
}

void _Decode_Time_Mode()//ʱ��У׼
{
    if( ( m_Communication_RX_WorkingBuff[7] == 0x47 ) && ( m_Communication_RX_WorkingBuff[8] == 0x50 ) && ( m_Communication_RX_WorkingBuff[9] == 0x53 ) ) //GPS
    {
        pHost_Cmd->Time_Adj_Mode = 2;
    }
    else if( ( m_Communication_RX_WorkingBuff[7] == 0x4c ) && ( m_Communication_RX_WorkingBuff[8] == 0x4f ) && ( m_Communication_RX_WorkingBuff[9] == 0x43 ) ) //LOC
    {
        pRTC_Time->Local_Second = ( m_Communication_RX_WorkingBuff[28] - 0x30 ) * 10 + ( m_Communication_RX_WorkingBuff[29] - 0x30 );
        pRTC_Time->Local_Minute = ( m_Communication_RX_WorkingBuff[25] - 0x30 ) * 10 + ( m_Communication_RX_WorkingBuff[26] - 0x30 );
        pRTC_Time->Local_Hour   = ( m_Communication_RX_WorkingBuff[22] - 0x30 ) * 10 + ( m_Communication_RX_WorkingBuff[23] - 0x30 );
        pRTC_Time->Local_Day    = ( m_Communication_RX_WorkingBuff[19] - 0x30 ) * 10 + ( m_Communication_RX_WorkingBuff[20] - 0x30 );
        pRTC_Time->Local_Month  = ( m_Communication_RX_WorkingBuff[16] - 0x30 ) * 10 + ( m_Communication_RX_WorkingBuff[17] - 0x30 );
        pRTC_Time->Local_Year   = ( m_Communication_RX_WorkingBuff[13] - 0x30 ) * 10  + ( m_Communication_RX_WorkingBuff[14] - 0x30 );
        pHost_Cmd->Time_Adj_Mode = 1;
    }
}

void _Decode_Scan_Space( u8* buf ) //�������
{
    u8 posx, mode;
    uint8_t dx = 0;
    posx = NMEA_Comma_Pos( buf, 1 );
    mode = NMEA_Str2num( buf + posx, &dx );
    if( mode == 1 )
    {
        posx = NMEA_Comma_Pos( buf, 2 );         //���ʱ��
        if( posx != 0XFF )
        {
            pHost_Cmd->Routine_Time_Spacing_pre = pHost_Cmd->Routine_Time_Spacing;
            pHost_Cmd->Routine_Time_Spacing = NMEA_Str2num( buf + posx, &dx );
            pHost_Cmd->Routine_Time_Spacing = pHost_Cmd->Routine_Time_Spacing > 1024 ? pHost_Cmd->Routine_Time_Spacing : 1024;//Ĭ����Сֵ1024
        }
    }
}

void _Decode_Routine_CMD( u8* buf ) //����۲�����ȡָ��
{
    u8 posx;
    uint8_t dx = 0;
    u8 temp;
    posx = NMEA_Comma_Pos( buf, 1 );
    if( posx != 0XFF )
    {
        temp = NMEA_Str2num( buf + posx, &dx );;
    }
}

void _Decode_OriData_Record()//Ĭ�Ͽ���
{
    pRTC_Time->Recording_Year    = pRTC_Time->year;
    pRTC_Time->Recording_Month   = pRTC_Time->year;
    pRTC_Time->Recording_Day     = pRTC_Time->day;
    pRTC_Time->Recording_Hour    = pRTC_Time->hour;
    pRTC_Time->Recording_Minute  = pRTC_Time->min;
    pRTC_Time->Recording_Second  = pRTC_Time->sec;
    if( ( m_Communication_RX_WorkingBuff[9] == 0x6f ) && ( m_Communication_RX_WorkingBuff[10] == 0x70 ) ) //stop
    {
        pHost_Cmd->OriData_Record_Flag = 2;
    }
    else if( ( m_Communication_RX_WorkingBuff[9] == 0x61 ) && ( m_Communication_RX_WorkingBuff[10] == 0x72 ) ) //star
    {
        pHost_Cmd->OriData_Record_Flag = 1;
    }
}

void  _Decode_Senser_Restar( u8* buf )
{
    u8 posx;
    uint8_t dx = 0;
    posx = NMEA_Comma_Pos( buf, 1 );
    if( posx != 0XFF )
    {
        pHost_Cmd->System_Restart_Flag = NMEA_Str2num( buf + posx, &dx );;
    }
}

/*
Scan_Mode  1 100��
           2 1024s
*/
void _Decode_Scan_Mode( u8* buf )
{
    u8 posx;
    uint8_t dx = 0;
    u8 temp;
    posx = NMEA_Comma_Pos( buf, 1 );
    if( posx != 0XFF )
    {
        temp = NMEA_Str2num( buf + posx, &dx );;
        if( temp == 0x01 )
        {
            pHost_Cmd->Scan_Mode_Select = 1;
        }
        else if( temp == 0x02 )
        {
            pHost_Cmd->Scan_Mode_Select = 2;
            //      pWave_Data->Wave_Cycle_Times = 0;
            //      pRTC_Time->Routinue_year = pRTC_Time->year;
            //      pRTC_Time->Routinue_month = pRTC_Time->mon;
            //      pRTC_Time->Routinue_day = pRTC_Time->day;
            //      pRTC_Time->Routinue_hour = pRTC_Time->hour;
            //      pRTC_Time->Routinue_min = pRTC_Time->min;
            //      pRTC_Time->Routinue_sec = pRTC_Time->sec;
        }
    }
}

void _Decode_Tradion_Sample( u8* buf ) //�Ƿ�������۲�
{
    u8 posx;
    uint8_t dx = 0;
    posx = NMEA_Comma_Pos( buf, 1 );
    if( posx != 0XFF )
    {
        pWave_Data->Tradion_Sample_Enable = NMEA_Str2num( buf + posx, &dx ); //�����Ϲ����ݹ۲��־
        if( pWave_Data->Tradion_Sample_Enable == 1 )
        {
            pWave_Data->Tradion_Sample_Start = 1;//��ʼ�������ݹ۲�
            pRTC_Time->Interval_year = pRTC_Time->year;
            pRTC_Time->Interval_month = pRTC_Time->mon;
            pRTC_Time->Interval_day = pRTC_Time->day;
            pRTC_Time->Interval_hour = pRTC_Time->hour;
            pRTC_Time->Interval_min = pRTC_Time->min;
            pRTC_Time->Interval_sec = pRTC_Time->sec;
        }
    }
}
uint8_t flag1;
void _Decode_RTD_CMD( u8* buf ) //ʵʱ���ݴ���
{
    u8 posx;
    uint8_t dx = 0;
    posx = NMEA_Comma_Pos( buf, 1 );
    if( posx != 0XFF )
    {
        pHost_Cmd->Real_Time_Data_Flag = NMEA_Str2num( buf + posx, &dx );;
    }
}

void Data_Transmit_232()
{
    //  pSenser_Data->State  =  (pSenser_Data->State| (GPS_Connect_Flag<<2));//GPS״̬
    //
    //  pSenser_Data->State  =  (pSenser_Data->State|(pHost_Cmd->Routine_Data_flag<<6));//����۲�״̬
    //
    //  pHost_Cmd->Min_WaveHeught = pHost_Cmd->Min_WaveHeught > 5 ? pHost_Cmd->Min_WaveHeught : 5;//Ĭ����Сֵ5
    //
    //
    //  Communication_Send(Communicate_TxData, Choose_Package_Data(Communicate_TxData));
    if( pHost_Cmd->Receive_CMD_Flag == 1 ) //�յ���λ��ָ��
    {
        switch( pHost_Cmd->Host_Data_Package )
        {
            case 1:
                Communication_Send( Communicate_TxData, Choose_Package_Data1( Communicate_TxData ) );
                break;
            case 2:
                Communication_Send( Communicate_TxData, Choose_Package_Data2( Communicate_TxData ) );
                break;
            case 3:
                Communication_Send( Communicate_TxData, Choose_Package_Data3( Communicate_TxData ) );
                break;
            case 4:
                Communication_Send( Communicate_TxData, Choose_Package_Data4( Communicate_TxData ) );
                break;
            case 5:
                Communication_Send( Communicate_TxData, Choose_Package_Data5( Communicate_TxData ) );
                break;
            case 7:
                Communication_Send( Communicate_TxData, Choose_Package_Data7( Communicate_TxData ) );
                break;
            case 8:
                Communication_Send( Communicate_TxData, Choose_Package_Data8( Communicate_TxData ) );
                break;
            case 10:
                Communication_Send( Communicate_TxData, Choose_Package_Data10( Communicate_TxData ) );
                break;
            default:
                break;
        }
        Record_Log_Data( &file_rcd ); //��¼����۲�������־
        pHost_Cmd->Receive_CMD_Flag = 0;//clear the flag;only send once
    }
    else
    {
        if( pHost_Cmd->Real_Time_Data_Flag == 1 )
        {
            Communication_Send( Communicate_TxData, Choose_Package_RealTime_Data( Communicate_TxData ) ); //real time data send
            if( ( ( pSenser_Data->State >> 6 ) & 0x03 ) == 3 )
            {
                pSenser_Data->State = pSenser_Data->State & 0x3F;//����ɼ���ɱ�־λ
            }
        }
    }
}



/*----------------------------------------------------------------------------
------------------------------ Data Choose -----------------------------------
----------------------------------------------------------------------------*/
u8 c[2] = {0};
char str_return[10] = {0};

uint8_t Choose_Package_Data1( u8* txbuf ) //�ش�У׼
{
    uint8_t data_totalnumber = 0;
    txbuf[data_totalnumber++] = ( uint8_t )0x24; //$
    txbuf[data_totalnumber++] = ( uint8_t )0x55; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x44; //D
    txbuf[data_totalnumber++] = ( uint8_t )0x4d; //M
    txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x43; //C
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
    txbuf[data_totalnumber++] = ( uint8_t )0x4f; //O
    txbuf[data_totalnumber++] = ( uint8_t )0x4b; //K
    txbuf[data_totalnumber++] = ( uint8_t )0x2a; //*
    NMEA0183_Check_Sum( txbuf, data_totalnumber );
    txbuf[data_totalnumber++]  = c[0];
    txbuf[data_totalnumber++]  = c[1];
    txbuf[data_totalnumber++] = ( uint8_t )0x0D; //:
    txbuf[data_totalnumber++] = ( uint8_t )0x0A; //:
    return data_totalnumber;
}

uint8_t Choose_Package_Data2( u8* txbuf ) //ʱ��У׼
{
    uint8_t data_totalnumber = 0;
    txbuf[data_totalnumber++] = ( uint8_t )0x24; //$
    txbuf[data_totalnumber++] = ( uint8_t )0x55; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x44; //D
    txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x54; //T
    txbuf[data_totalnumber++] = ( uint8_t )0x41; //C
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
    txbuf[data_totalnumber++] = ( uint8_t )0x4f; //O
    txbuf[data_totalnumber++] = ( uint8_t )0x4b; //K
    txbuf[data_totalnumber++] = ( uint8_t )0x2a; //*
    NMEA0183_Check_Sum( txbuf, data_totalnumber );
    txbuf[data_totalnumber++]  = c[0];
    txbuf[data_totalnumber++]  = c[1];
    txbuf[data_totalnumber++] = ( uint8_t )0x0D; //:
    txbuf[data_totalnumber++] = ( uint8_t )0x0A; //:
    return data_totalnumber;
}

uint8_t Choose_Package_Data3( u8* txbuf ) //�ı�������
{
    uint8_t data_totalnumber = 0;
    txbuf[data_totalnumber++] = ( uint8_t )0x24; //$
    txbuf[data_totalnumber++] = ( uint8_t )0x55; //U
    txbuf[data_totalnumber++] = ( uint8_t )0x44; //D
    txbuf[data_totalnumber++] = ( uint8_t )0x54; //T
    txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x49; //I
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
    txbuf[data_totalnumber++] = ( uint8_t )0x32; //2
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
    //  float_to_string(pHost_Cmd->Routine_Time_Spacing,str_return,Get_Num_Length(pHost_Cmd->Routine_Time_Spacing),0);
    //  for(i = 0; i< Get_Num_Length(pHost_Cmd->Routine_Time_Spacing);i++)
    //  {
    //    txbuf[data_totalnumber++] = str_return[i];
    //  }
    data_totalnumber += Float2String_Buf( pHost_Cmd->Routine_Time_Spacing_pre, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    //  sTemp = (s16)(pRTC_Time->year+2000);//year
    //  float_to_string(sTemp,str_return,Get_Num_Length(sTemp),0);
    //  for(i = 0; i< Get_Num_Length(sTemp);i++)
    //  {
    //    txbuf[data_totalnumber++] = str_return[i];
    //  }
    data_totalnumber += Float2String_Buf( pRTC_Time->year + 2000, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2f; // /
    //  sTemp = (s16)(pRTC_Time->mon);//month
    //  float_to_string(sTemp,str_return,Get_Num_Length(sTemp),0);
    //  for(i = 0; i< Get_Num_Length(sTemp);i++)
    //  {
    //    txbuf[data_totalnumber++] = str_return[i];
    //  }
    data_totalnumber += Float2String_Buf( pRTC_Time->mon, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2f; // /
    //  sTemp = (s16)(pRTC_Time->day);//day
    //  float_to_string(sTemp,str_return,Get_Num_Length(sTemp),0);
    //  for(i = 0; i< Get_Num_Length(sTemp);i++)
    //  {
    //    txbuf[data_totalnumber++] = str_return[i];
    //  }
    data_totalnumber += Float2String_Buf( pRTC_Time->day, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x20; //''
    //  sTemp = (s16)(pRTC_Time->hour);//hour
    //  float_to_string(sTemp,str_return,Get_Num_Length(sTemp),0);
    //  for(i = 0; i< Get_Num_Length(sTemp);i++)
    //  {
    //    txbuf[data_totalnumber++] = str_return[i];
    //  }
    data_totalnumber += Float2String_Buf( pRTC_Time->hour, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x3a; //:
    //  sTemp = (s16)(pRTC_Time->min);//min
    //  float_to_string(sTemp,str_return,Get_Num_Length(sTemp),0);
    //  for(i = 0; i< Get_Num_Length(sTemp);i++)
    //  {
    //    txbuf[data_totalnumber++] = str_return[i];
    //  }
    data_totalnumber += Float2String_Buf( pRTC_Time->min, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x3a; //:
    //  sTemp = (s16)(pRTC_Time->sec);//second
    //  float_to_string(sTemp,str_return,Get_Num_Length(sTemp),0);
    //  for(i = 0; i< Get_Num_Length(sTemp);i++)
    //  {
    //    txbuf[data_totalnumber++] = str_return[i];
    //  }
    data_totalnumber += Float2String_Buf( pRTC_Time->sec, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2a; //*
    NMEA0183_Check_Sum( txbuf, data_totalnumber );
    txbuf[data_totalnumber++]  = c[0];
    txbuf[data_totalnumber++]  = c[1];
    txbuf[data_totalnumber++] = ( uint8_t )0x0D; //:
    txbuf[data_totalnumber++] = ( uint8_t )0x0A; //:
    return data_totalnumber;
}

uint8_t Choose_Package_Data4( u8* txbuf ) //��ȡ�������
{
    uint8_t data_totalnumber = 0;
    txbuf[data_totalnumber++] = ( uint8_t )0x24; //$
    txbuf[data_totalnumber++] = ( uint8_t )0x55; //U
    txbuf[data_totalnumber++] = ( uint8_t )0x44; //D
    txbuf[data_totalnumber++] = ( uint8_t )0x57; //W
    txbuf[data_totalnumber++] = ( uint8_t )0x43; //C
    txbuf[data_totalnumber++] = ( uint8_t )0x52; //R
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
    data_totalnumber += Float2String_Buf( pWave_Data->Max_Height, txbuf, 2, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pWave_Data->Max_Cycle, txbuf, 2, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pWave_Data->One_Tenth_Height, txbuf, 2, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pWave_Data->One_Tenth_Cycle, txbuf, 2, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pWave_Data->Effective_Height, txbuf, 2, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pWave_Data->Effective_Cycle, txbuf, 2, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pWave_Data->Average_Height, txbuf, 2, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pWave_Data->Average_Cycle, txbuf, 2, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pWave_Data->Main_heading, txbuf, 1, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pWave_Data->Num_Of_Wave, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2a; // *
    NMEA0183_Check_Sum( txbuf, data_totalnumber );
    txbuf[data_totalnumber++]  = c[0];
    txbuf[data_totalnumber++]  = c[1];
    txbuf[data_totalnumber++] = ( uint8_t )0x0D; //:
    txbuf[data_totalnumber++] = ( uint8_t )0x0A; //:
    return data_totalnumber;
}

uint8_t Choose_Package_Data5( u8* txbuf ) //ԭʼ���ݼ�¼
{
    uint8_t data_totalnumber = 0;
    txbuf[data_totalnumber++] = ( uint8_t )0x24; //$
    txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x44; //D
    txbuf[data_totalnumber++] = ( uint8_t )0x4f; //O
    txbuf[data_totalnumber++] = ( uint8_t )0x44; //D
    txbuf[data_totalnumber++] = ( uint8_t )0x54; //T
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
    if( pHost_Cmd->OriData_Record_Flag == 1 ) //start
    {
        txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
        txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
        data_totalnumber += Float2String_Buf( pRTC_Time->year + 2000, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x2f; // /
        data_totalnumber += Float2String_Buf( pRTC_Time->mon, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x2f; // /
        data_totalnumber += Float2String_Buf( pRTC_Time->day, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x20; //''
        data_totalnumber += Float2String_Buf( pRTC_Time->hour, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x3a; //:
        data_totalnumber += Float2String_Buf( pRTC_Time->min, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x3a; //:
        data_totalnumber += Float2String_Buf( pRTC_Time->sec, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x2a; //*
        NMEA0183_Check_Sum( txbuf, data_totalnumber );
        txbuf[data_totalnumber++]  = c[0];
        txbuf[data_totalnumber++]  = c[1];
        txbuf[data_totalnumber++] = ( uint8_t )0x0D; //:
        txbuf[data_totalnumber++] = ( uint8_t )0x0A; //:
    }
    else if( pHost_Cmd->OriData_Record_Flag == 2 )
    {
        txbuf[data_totalnumber++] = ( uint8_t )0x45; //END
        txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
        data_totalnumber += Float2String_Buf( pRTC_Time->Recording_Year + 2000, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x2f; // /
        data_totalnumber += Float2String_Buf( pRTC_Time->Recording_Month, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x2f; // /
        data_totalnumber += Float2String_Buf( pRTC_Time->Recording_Day, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x20; //''
        data_totalnumber += Float2String_Buf( pRTC_Time->Recording_Hour, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x3a; //:
        data_totalnumber += Float2String_Buf( pRTC_Time->Recording_Minute, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x3a; //:
        data_totalnumber += Float2String_Buf( pRTC_Time->Recording_Second, txbuf, 0, data_totalnumber );
        txbuf[data_totalnumber++] = ( uint8_t )0x2a; //*
        NMEA0183_Check_Sum( txbuf, data_totalnumber );
        txbuf[data_totalnumber++]  = c[0];
        txbuf[data_totalnumber++]  = c[1];
        txbuf[data_totalnumber++] = ( uint8_t )0x0D; //:
        txbuf[data_totalnumber++] = ( uint8_t )0x0A; //:
    }
    return data_totalnumber;
}

uint8_t Choose_Package_Data7( u8* txbuf ) //ģʽѡ��
{
    uint8_t data_totalnumber = 0;
    txbuf[data_totalnumber++] = ( uint8_t )0x24; //$
    txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x44; //D
    txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x4D; //M
    txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
    if( pHost_Cmd->Scan_Mode_Select == 1 )
    {
        txbuf[data_totalnumber++] = ( uint8_t )0x31; //100
    }
    else if( pHost_Cmd->Scan_Mode_Select == 2 )
    {
        txbuf[data_totalnumber++] = ( uint8_t )0x32; //1024
    }
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
    data_totalnumber += Float2String_Buf( pRTC_Time->year + 2000, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2f; // /
    data_totalnumber += Float2String_Buf( pRTC_Time->mon, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2f; // /
    data_totalnumber += Float2String_Buf( pRTC_Time->day, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x20; //''
    data_totalnumber += Float2String_Buf( pRTC_Time->hour, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x3a; //:
    data_totalnumber += Float2String_Buf( pRTC_Time->min, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x3a; //:
    data_totalnumber += Float2String_Buf( pRTC_Time->sec, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2a; //*
    NMEA0183_Check_Sum( txbuf, data_totalnumber );
    txbuf[data_totalnumber++]  = c[0];
    txbuf[data_totalnumber++]  = c[1];
    txbuf[data_totalnumber++] = ( uint8_t )0x0D; //:
    txbuf[data_totalnumber++] = ( uint8_t )0x0A; //:
    return data_totalnumber;
}

uint8_t Choose_Package_Data8( u8* txbuf ) //�Ƿ�������۲�
{
    uint8_t data_totalnumber = 0;
    txbuf[data_totalnumber++] = ( uint8_t )0x24; //$
    txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x44; //D
    txbuf[data_totalnumber++] = ( uint8_t )0x54; //T
    txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
    if( pWave_Data->Tradion_Sample_Enable == 1 ) //����
    {
        txbuf[data_totalnumber++] = ( uint8_t )0x31; //
    }
    else //�ر�
    {
        txbuf[data_totalnumber++] = ( uint8_t )0x30; //
    }
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
    data_totalnumber += Float2String_Buf( pRTC_Time->year + 2000, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2f; // /
    data_totalnumber += Float2String_Buf( pRTC_Time->mon, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2f; // /
    data_totalnumber += Float2String_Buf( pRTC_Time->day, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x20; //''
    data_totalnumber += Float2String_Buf( pRTC_Time->hour, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x3a; //:
    data_totalnumber += Float2String_Buf( pRTC_Time->min, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x3a; //:
    data_totalnumber += Float2String_Buf( pRTC_Time->sec, txbuf, 0, data_totalnumber );
    txbuf[data_totalnumber++] = ( uint8_t )0x2a; //*
    NMEA0183_Check_Sum( txbuf, data_totalnumber );
    txbuf[data_totalnumber++]  = c[0];
    txbuf[data_totalnumber++]  = c[1];
    txbuf[data_totalnumber++] = ( uint8_t )0x0D; //:
    txbuf[data_totalnumber++] = ( uint8_t )0x0A; //:
    return data_totalnumber;
}

/*------------------------ ����������ͬ�� --------------------------------*/
uint8_t Choose_Package_Data10( u8* txbuf ) //
{
    uint8_t data_totalnumber = 0;
    txbuf[data_totalnumber++] = ( uint8_t )0x24; //$
    txbuf[data_totalnumber++] = ( uint8_t )0x53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0x44; //D
    txbuf[data_totalnumber++] = ( uint8_t )0X47; //G
    txbuf[data_totalnumber++] = ( uint8_t )0X53; //S
    txbuf[data_totalnumber++] = ( uint8_t )0X43; //C
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; //,
    data_totalnumber += Float2String_Buf( pHost_Cmd->Scan_Mode_Select, txbuf, 0, data_totalnumber ); //ͳ�Ʒ�ʽ 1��100��2:1024
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pHost_Cmd->Routine_Time_Spacing, txbuf, 0, data_totalnumber ); //�������
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pHost_Cmd->OriData_Record_Flag, txbuf, 0, data_totalnumber ); //ԭʼ�����Ƿ��¼1���� ��2����
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pWave_Data->Tradion_Sample_Enable, txbuf, 0, data_totalnumber ); //�Ƿ����������ݹ۲�
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    data_totalnumber += Float2String_Buf( pHost_Cmd->Real_Time_Data_Flag, txbuf, 0, data_totalnumber ); //�Ƿ���ʵʱ������
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    /*----------------  ��������� ---------------------------------*/
    txbuf[data_totalnumber++] = ( uint8_t )Device_Information.Hardware_Version[1]; //
    txbuf[data_totalnumber++] = ( uint8_t )Device_Information.Software_Version[1]; //
    txbuf[data_totalnumber++] = ( uint8_t )Device_Information.Device_ID[1]; //
    txbuf[data_totalnumber++] = ( uint8_t )Device_Information.Device_ID[2]; //
    txbuf[data_totalnumber++] = ( uint8_t )Device_Information.Device_ID[3]; //
    txbuf[data_totalnumber++] = ( uint8_t )Device_Information.Device_ID[4]; //
    txbuf[data_totalnumber++] = ( uint8_t )0x2c; // ,
    txbuf[data_totalnumber++] = ( uint8_t )Device_Information.User_ID[3]; //
    txbuf[data_totalnumber++] = ( uint8_t )Device_Information.User_ID[4]; //
    //  data_totalnumber +=Float2String_Buf(pWave_Data->Effective_Cycle,txbuf,2,data_totalnumber);
    //  txbuf[data_totalnumber++] = (uint8_t)0x2c;// ,
    //
    //  data_totalnumber +=Float2String_Buf(pWave_Data->Average_Height,txbuf,2,data_totalnumber);
    //  txbuf[data_totalnumber++] = (uint8_t)0x2c;// ,
    //
    //  data_totalnumber +=Float2String_Buf(pWave_Data->Average_Cycle,txbuf,2,data_totalnumber);
    //  txbuf[data_totalnumber++] = (uint8_t)0x2c;// ,
    //
    //  data_totalnumber +=Float2String_Buf(pWave_Data->Main_heading,txbuf,1,data_totalnumber);
    //  txbuf[data_totalnumber++] = (uint8_t)0x2c;// ,
    //
    //  data_totalnumber +=Float2String_Buf(pWave_Data->Num_Of_Wave,txbuf,0,data_totalnumber);
    txbuf[data_totalnumber++] = ( uint8_t )0x2a; // *
    NMEA0183_Check_Sum( txbuf, data_totalnumber );
    txbuf[data_totalnumber++]  = c[0];
    txbuf[data_totalnumber++]  = c[1];
    txbuf[data_totalnumber++] = ( uint8_t )0x0D; //:
    txbuf[data_totalnumber++] = ( uint8_t )0x0A; //:
    return data_totalnumber;
}

/*--------------------------- ʵʱ���� -----------------------------------*/
/*
pSenser_Data->State: ϵͳ״̬
bit 0-1�� ������״̬ 00����������
                    01���Լ첻��
                   11���Ͽ�����
bit 2-3��GPS״̬  00:δ����GPS
                 01:����GPS ������
                11;����GPS ������

bit 6~7 ����۲�״̬  00 δ����
                      01 ���ڲɼ�
                      11 �ɼ����
*/

uint8_t Choose_Package_RealTime_Data( u8* txbuf )
{
    uint16_t sTemp = 0;
    uint16_t data_totalnumber = 0;
    uint16_t Checksum_TwoBytes = 0;
    txbuf[data_totalnumber++] = ( uint8_t )0xB5;
    txbuf[data_totalnumber++] = ( uint8_t )0x5B;
    txbuf[data_totalnumber++] = ( uint8_t )0xFF;              //Data_length
    /*--------------------------- Time 1-3 ------------------------------------*/
    sTemp = ( s16 )( ( pRTC_Time->year << 8 ) | pRTC_Time->mon );
    txbuf[data_totalnumber++]  = ( uint8_t )( ( sTemp & 0xFF00 ) >> 8 );
    txbuf[data_totalnumber++]  = ( uint8_t )( sTemp & 0x00FF );
    sTemp = ( s16 )( ( pRTC_Time->day << 8 ) | pRTC_Time->hour );
    txbuf[data_totalnumber++] = ( uint8_t )( ( sTemp & 0xFF00 ) >> 8 );
    txbuf[data_totalnumber++] = ( uint8_t )( sTemp & 0x00FF );
    sTemp = ( s16 )( ( pRTC_Time->min << 8 ) | pRTC_Time->sec );
    txbuf[data_totalnumber++] = ( uint8_t )( ( sTemp & 0xFF00 ) >> 8 );
    txbuf[data_totalnumber++] = ( uint8_t )( sTemp & 0x00FF );
    /*--------------------------- Wave Data 4-6 ------------------------------*/
    sTemp = ( s16 )( pSenser_Data->Yaw * 100 );
    txbuf[data_totalnumber++] = ( uint8_t )( ( sTemp & 0xFF00 ) >> 8 );
    txbuf[data_totalnumber++] = ( uint8_t )( sTemp & 0x00FF );
    sTemp = ( s16 )( pWave_Data->Height ); // ��ֵ
    txbuf[data_totalnumber++] = ( uint8_t )( ( sTemp & 0xFF00 ) >> 8 );
    txbuf[data_totalnumber++]  = ( uint8_t )( sTemp & 0x00FF );
    sTemp = ( s16 )( pWave_Data->Cycle * 1000 );
    txbuf[data_totalnumber++] = ( uint8_t )( ( sTemp & 0xFF00 ) >> 8 );
    txbuf[data_totalnumber++]  = ( uint8_t )( sTemp & 0x00FF );
    /*--------------------------- Senser State 7 -----------------------------*/
    sTemp = ( s8 )( pSenser_Data->State );
    txbuf[data_totalnumber++] = ( uint8_t )( ( sTemp & 0x00FF ) );
    /*--------------------------- �������� 7-8 -----------------------------*/
    sTemp = ( s16 )( pWave_Data->Absolute_Height * 1000 ); // �߶�
    txbuf[data_totalnumber++] = ( uint8_t )( ( sTemp & 0xFF00 ) >> 8 );
    txbuf[data_totalnumber++]  = ( uint8_t )( sTemp & 0x00FF );
    sTemp = ( s16 )( pSenser_Data->Roll * 100 ); // �߶�
    txbuf[data_totalnumber++] = ( uint8_t )( ( sTemp & 0xFF00 ) >> 8 );
    txbuf[data_totalnumber++]  = ( uint8_t )( sTemp & 0x00FF );
    sTemp = ( s16 )( pSenser_Data->Pitch * 100 ); // �߶�
    txbuf[data_totalnumber++] = ( uint8_t )( ( sTemp & 0xFF00 ) >> 8 );
    txbuf[data_totalnumber++]  = ( uint8_t )( sTemp & 0x00FF );
    txbuf[2] = ( uint8_t )( data_totalnumber + 2 );
    /*------------------------------ Check_Sum -------------------------------*/
    Checksum_TwoBytes = TxData_CheckSum_twoBytes( txbuf + 2, txbuf[2] - 4 );
    txbuf[data_totalnumber++]  = ( u8 )( ( Checksum_TwoBytes >> 8 ) & 0x00FF );
    txbuf[data_totalnumber++]  = ( u8 )( Checksum_TwoBytes & 0x00FF );
    return data_totalnumber;
}



/*----------------------------------------------------------------------------
------------------------- �ַ�ת�� ������У�� --------------------------------
----------------------------------------------------------------------------*/
uint16_t TxData_CheckSum_twoBytes( uint8_t* pData, uint8_t nLength )
{
    uint16_t checksum = 0;
    uint8_t i = 0;
    for( i = 0; i < nLength; i++ )
    {
        checksum += ( ( uint16_t )pData[i] & 0x00FF );
    }
    return checksum;
}


//MEA-0183��ϢУ���
//buf:MEA-0183���ݻ������׵�ַ
void NMEA0183_Check_Sum( u8* pData, u8 nLength ) //, u8 *c1, u8 *c2)
{
    u8  sum, i, temp1, temp2;
    static u8* strbuf = "0123456789ABCDEF";
    sum = 0;
    i = 1;
    if( pData[0] != '$' )
    {
        return ;
    }
    while( pData[i] != '*' && nLength )
    {
        nLength--;
        sum ^= pData[i];
        i++;
    }
    temp1 = sum / 16;
    temp2 = sum % 16;
    c[0] = strbuf[temp1];             /*  ��������16����У���  */
    c[1] = strbuf[temp2];
}

uint8_t Get_Num_Length( u16 Data )
{
    uint8_t length = 0;
    while( Data != 0 )
    {
        Data = Data / 10;
        length ++;
    }
    return length;
}

// ������ת�����ַ���
// ������data ��Ҫת��������
//         str ת�����ַ�
//             n2 С����λ��

// ���أ�str

void float_to_string( float data, char* str, u8 n2 )
{
    int i, j, k;
    int temp, tempoten;
    float ori_data;
    u8 intpart[5];
    //1.ȷ������λ
    ori_data = data;
    if( data < 0 )
    {
        str[0] = '-';
        data = -data;
    }
    //2.ȷ����������
    temp = ( int )data;
    i = 0;
    tempoten = temp / 10;
    while( tempoten != 0 )
    {
        intpart[i] = temp - 10 * tempoten + 48; //to ascii code
        temp = tempoten;
        tempoten = temp / 10;
        i++;
    }
    intpart[i] = temp + 48;
    if( ori_data < 0 )
    {
        for( k = 1; k <= i + 1; k++ )
        {
            str[k] = intpart[i + 1 - k];
        }
        i += 1;
    }
    else
    {
        for( k = 0; k <= i + 1; k++ )
        {
            str[k] = intpart[i - k];
        }
    }
    //3.ȷ��С������,ȡ��1λС��
    if( n2 > 0 )
    {
        str[i + 1] = '.';
        data = data - ( int )data;
        ori_data = data - ( int )data;
        for( j = 0; j < n2; j++ )
        {
            str[i + 2 + j] = ( int )( data * 10 ) + 48;
            data = data * 10.0;
            data = data - ( int )data;
            ori_data = data - ( int )data;
        }
    }
}


uint8_t Float2String_Buf( float data, uint8_t* str, uint8_t potlength, uint8_t num )
{
    uint8_t i, j, k;
    if( ( int )data )
    {
        if( potlength )
        {
            j = Get_Num_Length( ( int )data ) + 1 + potlength;
        }
        else
        {
            j = Get_Num_Length( ( int )data );
        }
    }
    else
    {
        if( potlength )
        {
            j = 2 + potlength;
        }
        else
        {
            j = 1;
        }
    }
    k = num;
    float_to_string( data, str_return, potlength );
    for( i = 0; i < j; i++ )
    {
        str[k++] = str_return[i];
    }
    str[k + 1] = 0x00;
    return j;
}

void Float2String_CharBuf( float data, char* str, uint8_t potlength )
{
    uint8_t i;
    float j;
    j = data > 0 ? data : -data;
    i = Get_Num_Length( ( int )j );
    i = i < 1 ? 1 : i;//
    i = data < 0 ? i + 1 : i;
    float_to_string( data, str, potlength );
    str[i + potlength + 1] = 0x20;
    str[i + potlength + 2] = 0x00;
}


