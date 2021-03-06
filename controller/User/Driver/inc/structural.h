/**
******************************************************************************
* @file    structural.h
* @author  UAV Application Team
* @version V2.1.0
* @date    2-September-2014
* @brief   此文件主要结构体定义文件。
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


/*--------------------------     BNO055数据        -------------------------*/
typedef struct
{
    /*--------------------------     BNO055原始数据  -------------------------*/
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

    /*------------------------------    BNO055滤波数据  ----------------------*/
    float BNOXacc;
    float BNOYacc;
    float BNOZacc;

    /*------------------------------   ADI原始数据    ------------------------*/
    //   float ADIax;
    //   float ADIay;
    //   float ADIaz;
    //
    //   /*------------------------------    ADI滤波数据   ------------------------*/
    //   float ADIXacc;
    //   float ADIYacc;
    //   float ADIZacc;
    //
    /*------------------------------    传感器滤波数据  ----------------------*/
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

    /*------------------------   传感器状态  ----------------------------------*/
    uint8_t Senser_Mode;//是否使用备用传感器 0：bno ;1:icm

    uint8_t State;//外部通信使用
    uint8_t ICM_flag;//正常 ：1  故障 ： 0
    uint8_t BNO_flag;

    /*--------------------------     传感器编号    ----------------------------*/

    //   uint8_t Senser_ID;/*bit 7   Hardware_Version
    //                       bit 6   Software_Version
    //                       bit5~2  Device_ID
    //                       bit1~0  User_ID
    //                       */
    //   uint8_t User_ID;

} d_Senser_Data;

/*--------------------------     GPS原始数据    ----------------------------*/
typedef struct
{

    float Star_Num;
    float State;
    float Heading;

    float Lon;
    float Lat;
    float GPS_Alt;
    float HDop;

    uint16_t UTC_Year;//UTC时间
    uint8_t UTC_Month;
    uint8_t UTC_Day;
    uint8_t UTC_Hour;
    uint8_t UTC_Minute;
    uint8_t UTC_Second;

    uint16_t Beijing_Year;//北京时间
    uint8_t Beijing_Month;
    uint8_t Beijing_Day;
    uint8_t Beijing_Hour;
    uint8_t Beijing_Minute;
    uint8_t Beijing_Second;

    float Log_Lon;//配置文件
    float Log_Lat;

} d_Gps_Data;

/*--------------------------     波浪特征数据   --------------------------*/
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

    uint8_t Tradion_Sample_Start;//常规观测开始标志位  0：开启  1：关闭
    uint8_t Tradion_Sample_Enable;//是否启动常规数据观测  0：否  1：是
    uint8_t Tradion_Sample_complete;//一次常规观测完成
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

/*--------------------------     上位机指令     ----------------------------*/
typedef struct
{
    uint8_t Senser_Cail_CMD ;//地磁校准
    uint8_t Time_Adj_Mode;//时间校准

    uint8_t Routine_Collect_Flag;//提取常规观测数据
    uint8_t Routine_Data_flag ;//改变常规数据采集间隔

    uint8_t OriData_Record_Flag;//原始数据记录

    uint8_t Scan_Mode_Select;//改变滚动计算方式，1为100个大波，2为1024秒

    uint8_t System_Restart_Flag;//系统重启

    uint8_t Min_WaveHeught ;//最小浪高设置

    int Routine_Time_Spacing;//常规观测间隔时间
    int Routine_Time_Spacing_pre;//原先的常规观测时间
    uint8_t Receive_CMD_Flag;

    uint8_t Real_Time_Data_Flag;

    uint8_t Host_Data_Package;

} d_Host_Cmd;

/*--------------------------     实时时钟     ----------------------------*/
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

    uint16_t Local_Year;//上位机时间
    uint8_t Local_Month;
    uint8_t Local_Day;
    uint8_t Local_Hour;
    uint8_t Local_Minute;
    uint8_t Local_Second;

    uint16_t Delayed_Year;//延时启动时间
    uint8_t Delayed_Month;
    uint8_t Delayed_Day;
    uint8_t Delayed_Hour;
    uint8_t Delayed_Minute;
    uint8_t Delayed_Second;

    uint16_t Recording_Year;//原始数据记录时间
    uint8_t Recording_Month;
    uint8_t Recording_Day;
    uint8_t Recording_Hour;
    uint8_t Recording_Minute;
    uint8_t Recording_Second;

    uint8_t Routinue_year;//1024s模式用于判别是否到达时间
    uint8_t Routinue_month;
    uint8_t Routinue_day;
    uint8_t Routinue_hour;
    uint8_t Routinue_min;
    uint8_t Routinue_sec;

    uint8_t Interval_year;//采样间隔时间
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
