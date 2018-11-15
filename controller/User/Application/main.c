/* Includes ------------------------------------------------------------------*/
#include "main.h"

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

extern u8 BNO_TxData[20];
extern uint8_t Mag_Calibration_Step;
extern uint8_t send_time ;
extern int fft_i;
extern uint8_t send_flag;
extern uint8_t Senser_Cail_CMD;
extern uint8_t log_time;


uint16_t cnt_5hz = 0;
uint8_t time_count = 0;




float angle_roll, angle_dot_roll, f_angle_roll, f_angle_dot_roll;
float angle_pitch, angle_dot_pitch, f_angle_pitch, f_angle_dot_pitch;

static Init_RES Recorder_Config();
void SD_POWER_Control();
void SENSER_POWER_Control();
void System_Status_Init();
void System_Status();
void Log_Data_Record();
void OriData_AutoClose_Enable();

int main( void )
{
    SystemInit();
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );
    Time_init();
    System_Status_Init();//系统状态指示
    /*---------------------------------- 232串口  ------------------------------*/
    Communication_232_Configure();//上位机通信
    Communication_OriData_Configure();//原始数据传输
    UART4_init( 115200 );
    BNO_Configure();
    /*---------------------------    RS8010sj   --------------------------------*/
    RS8010sj_Init();
    /*---------------------------    GPS     -----------------------------------*/
    GPS_Configure();
    /*---------------------------    SD          -------------------------------*/
    SD_POWER_Control();//使能电源
    while( Recorder_Config() != OK )
    {
        delay_ms( 200 );
    }
    EEPROM_Loading_Config();//读取校准系数
    Read_CfgFile();//读取配置文件
    EEPROM_Loading_DeviceInformation();//读取版本信息、设备ID
    Add_LocalFile();//添加记录
    Creat_Log_File();//建立日志文件
    /*----------------------------------BNO055初始化---------------------------*/
    delay_ms( 100 );
    SENSER_POWER_Control();//使能传感器电源
    delay_ms( 100 );
    /*---------------------------    ICM传感器   ------------------------------*/
    ICM2060x_Init();
    /****************************************************************************/
    /*******************************程序自检*************************************/
    /****************************************************************************/
    while( 1 )
    {
        if( BNO055_IsReadyToRead() )
        {
            if( send_flag == 1 )
            {
                BNO055_Update();
            }
            else
            {
                BNO055_Reg_Check1();
            }
        }
        if( Communication_IsReadyToRead() )
        {
            Communication_Update();
        }
        if( Time3_Flag_100hz )
        {
            Time3_Flag_100hz = 0;
            if( pSenser_Data->ICM_flag == 1 )
            {
                ICM2060x_GetACC();
                ICM2060x_GetGYRO();
            }
            /*---------------------------    BNO数据读取  ------------------------------*/
            if( send_flag == 1 )
            {
                BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_ReadData( BNO_TxData ) );
            }
            //    Get_local_gravity();//计算当地重力
        }
        if( Time3_Flag_50hz )
        {
            Time3_Flag_50hz = 0;
            System_Status();//系统自检不通过，快闪
            Senser_Data_SelfCheck();
            if( IsSenser_Ready() )
            {
                FFT_init();//FFT算法系数
                //       AHRS_EKF_Init();
                GPIO_ResetBits( GPIOC, GPIO_Pin_6 ); //自检结束，LED开
                //       IWDG_Init(4,500);//设置看门狗 溢出时间1s
                break;
            }
        }
        if( Time3_Flag_5hz )
        {
            Time3_Flag_5hz = 0;
            Data_Transmit_232();//232发送数据
            Record_Data( &file_loc ); //5Hz 数据存储
            f_sync( &file_loc );
        }
        if( Time3_Flag_2hz )
        {
            Time3_Flag_2hz = 0;
            if( send_flag != 1 )
            {
                BNO055_Init();
            }
        }
    }
    /****************************************************************************/
    /*******************************主程序***************************************/
    /****************************************************************************/
    while( 1 )
    {
        if( BNO055_IsReadyToRead() )
        {
            BNO055_Update();
        }
        if( Communication_IsReadyToRead() )
        {
            Communication_Update();
        }
        if( GPS_IsReadyToRead() )
        {
            GPS_Update();
        }
        /*---------------------------- 频率控制执行代码区 ------------------------*/
        if( Time3_Flag_400hz )
        {
            Time3_Flag_400hz = 0;
            if( pSenser_Data->Senser_Mode == 1 )
            {
                ICM2060x_GetACC();
                ICM2060x_GetGYRO();
            }
        }
        if( Time3_Flag_100hz )
        {
            Time3_Flag_100hz = 0;
            BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_ReadData( BNO_TxData ) );
            Senser_data_process();//传感器数据选择
            Senserdata_filter_H();//100;8hz zacc二次
        }
        if( Time3_Flag_50hz )
        {
            Time3_Flag_50hz = 0;
            angle_dot_roll = -pSenser_Data->RollRate ;
            angle_roll = atan( pSenser_Data->Yacc  / sqrt( pSenser_Data->Xacc  * pSenser_Data->Xacc  + pSenser_Data->Zacc  * pSenser_Data->Zacc ) );
            angle_dot_pitch = pSenser_Data->PitchRate ;
            angle_pitch = atan( pSenser_Data->Xacc  / sqrt( pSenser_Data->Yacc  * pSenser_Data->Yacc  + pSenser_Data->Zacc  * pSenser_Data->Zacc ) );
            Angle_fuse_roll( angle_roll, angle_dot_roll, &f_angle_roll, &f_angle_dot_roll );
            Angle_fuse_pitch( angle_pitch, angle_dot_pitch, &f_angle_pitch, &f_angle_dot_pitch );
            //      AHRS_EKF();
            System_Check();//传感器状态检测
            Tradion_Sample_Flinish();
        }
        if( Time3_Flag_20hz )
        {
            Time3_Flag_20hz = 0;
            if( pHost_Cmd->Senser_Cail_CMD == 0x01 )
            {
                Mag_Calibration_Operation();
                if( Mag_Calibration_Step == 3 )
                {
                    System_Status();
                }
                if( Mag_Calibration_Step == 5 )
                {
                    GPIO_ResetBits( GPIOC, GPIO_Pin_6 );
                }
            }
            if( pHost_Cmd->OriData_Record_Flag == 1 ) //原始数据是否记录1：是 ；2：否
            {
                Record_Data( &file_loc );
            }
            OriData__Communication_Send();//原始数据实时发送
        }
        if( Time3_Flag_5hz )
        {
            Time3_Flag_5hz = 0;
            if( ( Mag_Calibration_Step != 5 ) && ( Mag_Calibration_Step != 3 ) )
            {
                System_Status();//系统正常工作，慢闪
            }
            Tradion_Sample();//判断何时开启常规观测
            Data_Transmit_232();//上位机指令通信
            //      Record_Log_Data(&file_rcd);//记录常规观测数据日志
            OriData_AutoClose_Enable();
            //      if((pSenser_Data->State & 0x03) == 0)
            //      {
            //        IWDG_Feed();//喂狗
            //      }
            cnt_5hz++;
        }
        if( Time3_Flag_2hz )
        {
            Time3_Flag_2hz = 0;
            if( time_count % 2 == 0 )
            {
                Time_Read_Run();//读取RTC时间
            }
            if( ( pRTC_Time->hour == 0 ) && ( pRTC_Time->min == 0 ) && ( pRTC_Time->sec < 1 ) ) //count_rec>=72000)//20h每小时72000个 每小时存一个名字
            {
                Add_Ending_Message( &file_loc ); //加入文件结束标志
                f_sync( &file_loc );
                f_close( &file_loc );
                pGps_Data->Log_Lon = pGps_Data->Lon;//updata the location
                pGps_Data->Log_Lat = pGps_Data->Lat;
                Creat_Log_File();//建立日志文件
                Add_LocalFile();//添加记录
            }
            if( send_time >= 30 )
            {
                f_sync( &file_loc );
                send_time = 0;
            }//4ms 数据同步
            if( fft_i >= 1024 )
            {
                FFT_Cal();
                //        FFT_AXToVel();
                //        FFT_AYToVel();
                fft_i = 0;
            }
            RTC_Time_Adj();
            /*----------------------------   系统软件复位   ------------------------*/
            if( pHost_Cmd->System_Restart_Flag == 1 )
            {
                __set_FAULTMASK( 1 ); //关闭所有中断
                NVIC_SystemReset();//复位函数
            }
            time_count++;
        }
    }
}

static Init_RES Recorder_Config()
{
    //  float sdsize;
    mem_init();/* mem init for Fatfs */
    //  SD_Init();//SD卡初始化
    if( SD_Init() != SD_OK ) //正常会返回0
    {
        return ERR;/* SD Init Error */
    }
    //  if(SD_GetCardInfo(&SDCardInfo)==SD_OK)
    //  {
    //    sdsize = (float)((double)SDCardInfo.CardCapacity/1048576.0);/* Get SD CardCapacity  单位 M */
    //  }
    //  else
    //    return ERR;/* Get SD CardCapacity Error */
    return OK;
}
/*---------------------------- SD卡电源使能  ------------------------*/
void SD_POWER_Control() //PA15,拉高使能电源
{
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init( GPIOA, &GPIO_InitStructure );
    GPIO_ResetBits( GPIOA, GPIO_Pin_15 );
}
/*---------------------------- BNO电源使能  ------------------------*/
void SENSER_POWER_Control() //PB12,拉高使能电源
{
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
    GPIO_SetBits( GPIOB, GPIO_Pin_12 );
}
/*---------------------------- 系统状态指示  ------------------------*/
void System_Status_Init() //PC6,系统状态
{
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init( GPIOC, &GPIO_InitStructure );
    GPIO_ResetBits( GPIOC, GPIO_Pin_6 );
}

void OriData_AutoClose_Enable()//连续记录超过15天，自动关闭
{
    uint32_t Time_continuous;
    Time_continuous = ( pRTC_Time->day - pRTC_Time->Delayed_Hour ) * 3600 + ( pRTC_Time->min - pRTC_Time->Delayed_Minute ) * 60 + ( pRTC_Time->sec - pRTC_Time->Delayed_Second );
    if( ( pRTC_Time->Recording_Day - pRTC_Time->day >= 14 ) && ( Time_continuous > 86400 ) )
    {
        pHost_Cmd->OriData_Record_Flag = 2;
    }
}
void System_Status()
{
    GPIO_ToggleBits( GPIOC, GPIO_Pin_6 );
}
///*---------------------------- 常规观测数据记录  ------------------------*/
//void Log_Data_Record()
//{
//  /*---------------------------- 手动模式  ------------------------*/
//  if(pHost_Cmd->Routine_Data_flag == 0x02)//常规1024s数据记录
//  {
//    Get_Routine_Data();//计算常规观测数据
//    Record_Log_Data(&file_rcd);
//  }
//  /*---------------------------- 自动模式  ------------------------*/
////  if(pHost_Cmd->Scan_Frequence == 0x0A) //0.5小时一次
////  {
////  }else if(pHost_Cmd->Scan_Frequence == 0x01) //1小时一次
////  {
////
////  }else if(pHost_Cmd->Scan_Frequence == 0x02) //2小时一次
////  {
////
////  }else if(pHost_Cmd->Scan_Frequence == 0x04) //4小时一次
////  {
////
////  }
//
//}
/*-----------------------延时启动常规观测-------------------------------------*/
//void Observe_Routine_Delayed_Start()
//{
//  if(pRTC_Time->year == pRTC_Time->Delayed_Year
//     && pRTC_Time->mon == pRTC_Time->Delayed_Month
//       && pRTC_Time->day == pRTC_Time->Delayed_Day
//         && pRTC_Time->hour == pRTC_Time->Delayed_Hour
//           && pRTC_Time->min == pRTC_Time->Delayed_Minute
//             && pRTC_Time->sec == pRTC_Time->Delayed_Second)
//  {
//      pHost_Cmd->Routine_Data_flag = 0x02;
//  }
//}




#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif

/************************ (C) COPYRIGHT AOYIWEIYENG *****END OF FILE***********/
