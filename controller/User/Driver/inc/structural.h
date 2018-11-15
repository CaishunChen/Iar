/**
******************************************************************************
* @file    structural.h
* @author  UAV Application Team
* @version V2.1.0
* @date    2-September-2014
* @brief   ���ļ���Ҫ�ṹ�嶨���ļ���
******************************************************************************
* @device  Master---STM32F405RGT6
******************************************************************************
*/

#ifndef _STRUCTUAL_H
#define _STRUCTUAL_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx.h"

/* Private define ------------------------------------------------------------*/
/* Private structure ---------------------------------------------------------*/


/*--------------------------     BNO055����        -------------------------*/
typedef struct
{
    /*--------------------------     BNO055ԭʼ����  -------------------------*/
    float BNOax;
    float BNOay;
    float BNOaz;
    float BNOgx;
    float BNOgy;
    float BNOgz;
    float BNOmx;
    float BNOmy;
    float BNOmz;

    //    float BNOax1;
    //    float BNOay1;
    //    float BNOaz1;
    //    float BNOgx1;
    //    float BNOgy1;
    //    float BNOgz1;
    //    float BNOmx1;
    //    float BNOmy1;
    //    float BNOmz1;


    float ICMAX;
    float ICMAY;
    float ICMAZ;
    float ICMGX;
    float ICMGY;
    float ICMGZ;

    /*------------------------------    BNO055�˲�����  ----------------------*/
    float BNOXacc;
    float BNOYacc;
    float BNOZacc;

    /*------------------------------   ADIԭʼ����    ------------------------*/
    //   float ADIax;
    //   float ADIay;
    //   float ADIaz;
    //
    //   /*------------------------------    ADI�˲�����   ------------------------*/
    //   float ADIXacc;
    //   float ADIYacc;
    //   float ADIZacc;
    //
    /*------------------------------    �������˲�����  ----------------------*/
    float Xacc;
    float Yacc;
    float Zacc;
    float RollRate;
    float PitchRate;
    float YawRate;
    float magX;
    float magY;
    float magZ;
    float Zaccf;
    //   float BNOr;
    //   float BNOp;
    //   float BNOy;

    float Roll;
    float Pitch;
    float Yaw;
    float mYaw;

    /*------------------------   ������״̬  ----------------------------------*/
    uint8_t Senser_Mode;//�Ƿ�ʹ�ñ��ô����� 0��bno ;1:icm

    uint8_t State;//�ⲿͨ��ʹ��
    uint8_t ICM_flag;//���� ��1  ���� �� 0
    uint8_t BNO_flag;

    /*--------------------------     ���������    ----------------------------*/

    //   uint8_t Senser_ID;/*bit 7   Hardware_Version
    //                       bit 6   Software_Version
    //                       bit5~2  Device_ID
    //                       bit1~0  User_ID
    //                       */
    //   uint8_t User_ID;

} d_Senser_Data;

/*--------------------------     GPSԭʼ����    ----------------------------*/
typedef struct
{

    float Star_Num;
    float State;
    float Heading;

    float Lon;
    float Lat;
    float GPS_Alt;
    float HDop;

    uint16_t UTC_Year;//UTCʱ��
    uint8_t UTC_Month;
    uint8_t UTC_Day;
    uint8_t UTC_Hour;
    uint8_t UTC_Minute;
    uint8_t UTC_Second;

    uint16_t Beijing_Year;//����ʱ��
    uint8_t Beijing_Month;
    uint8_t Beijing_Day;
    uint8_t Beijing_Hour;
    uint8_t Beijing_Minute;
    uint8_t Beijing_Second;

    float Log_Lon;//�����ļ�
    float Log_Lat;

} d_Gps_Data;

/*--------------------------     ������������   --------------------------*/
typedef struct
{


    float Height;
    float Cycle;

    float Absolute_Height;
    float Main_heading;
    uint8_t Wave_Cycle_Times;

    float Max_Height;
    float Max_Cycle;

    float One_Tenth_Height;
    float One_Tenth_Cycle;

    float Effective_Height;
    float Effective_Cycle;

    float Average_Height;
    float Average_Cycle;

    float Num_Of_Wave;

    uint8_t Heading_count_flag;
    uint8_t Cycle_Cal_Mode;
    uint8_t Maxheight_Flag;

    uint8_t Tradion_Sample_Start;//����۲⿪ʼ��־λ  0������  1���ر�
    uint8_t Tradion_Sample_Enable;//�Ƿ������������ݹ۲�  0����  1����
    uint8_t Tradion_Sample_complete;//һ�γ���۲����
    //    uint8_t Wave_Heading_N;//
    //    uint8_t Wave_Heading_NNE;
    //    uint8_t Wave_Heading_NE;
    //    uint8_t Wave_Heading_ENE;
    //
    //    uint8_t Wave_Heading_E;
    //    uint8_t Wave_Heading_ESE;
    //    uint8_t Wave_Heading_SE;
    //    uint8_t Wave_Heading_SSE;
    //
    //    uint8_t Wave_Heading_S;//
    //    uint8_t Wave_Heading_SSW;
    //    uint8_t Wave_Heading_SW;
    //    uint8_t Wave_Heading_WSW;
    //
    //    uint8_t Wave_Heading_W;
    //    uint8_t Wave_Heading_WNW;
    //    uint8_t Wave_Heading_NW;
    //    uint8_t Wave_Heading_NNW;

} d_Wave_Data;

/*--------------------------     ��λ��ָ��     ----------------------------*/
typedef struct
{
    uint8_t Senser_Cail_CMD ;//�ش�У׼
    uint8_t Time_Adj_Mode;//ʱ��У׼

    uint8_t Routine_Collect_Flag;//��ȡ����۲�����
    uint8_t Routine_Data_flag ;//�ı䳣�����ݲɼ����

    uint8_t OriData_Record_Flag;//ԭʼ���ݼ�¼

    uint8_t Scan_Mode_Select;//�ı�������㷽ʽ��1Ϊ100���󲨣�2Ϊ1024��

    uint8_t System_Restart_Flag;//ϵͳ����

    uint8_t Min_WaveHeught ;//��С�˸�����

    int Routine_Time_Spacing;//����۲���ʱ��
    int Routine_Time_Spacing_pre;//ԭ�ȵĳ���۲�ʱ��
    uint8_t Receive_CMD_Flag;

    uint8_t Real_Time_Data_Flag;

    uint8_t Host_Data_Package;

} d_Host_Cmd;

/*--------------------------     ʵʱʱ��     ----------------------------*/
typedef struct
{

    uint8_t sec ;
    uint8_t min ;
    uint8_t hour ;
    uint8_t day;
    uint8_t week ;
    uint8_t mon;
    uint8_t year;
    uint8_t Vel_flag;

    uint16_t Local_Year;//��λ��ʱ��
    uint8_t Local_Month;
    uint8_t Local_Day;
    uint8_t Local_Hour;
    uint8_t Local_Minute;
    uint8_t Local_Second;

    uint16_t Delayed_Year;//��ʱ����ʱ��
    uint8_t Delayed_Month;
    uint8_t Delayed_Day;
    uint8_t Delayed_Hour;
    uint8_t Delayed_Minute;
    uint8_t Delayed_Second;

    uint16_t Recording_Year;//ԭʼ���ݼ�¼ʱ��
    uint8_t Recording_Month;
    uint8_t Recording_Day;
    uint8_t Recording_Hour;
    uint8_t Recording_Minute;
    uint8_t Recording_Second;

    uint8_t Routinue_year;//1024sģʽ�����б��Ƿ񵽴�ʱ��
    uint8_t Routinue_month;
    uint8_t Routinue_day;
    uint8_t Routinue_hour;
    uint8_t Routinue_min;
    uint8_t Routinue_sec;

    uint8_t Interval_year;//�������ʱ��
    uint8_t Interval_month;
    uint8_t Interval_day;
    uint8_t Interval_hour;
    uint8_t Interval_min;
    uint8_t Interval_sec;


} d_RTC_Time;

#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT AOYIWEIYENG *****END OF FILE***********/