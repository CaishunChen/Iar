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
    System_Status_Init();//ϵͳ״ָ̬ʾ
    /*---------------------------------- 232����  ------------------------------*/
    Communication_232_Configure();//��λ��ͨ��
    Communication_OriData_Configure();//ԭʼ���ݴ���
    UART4_init( 115200 );
    BNO_Configure();
    /*---------------------------    RS8010sj   --------------------------------*/
    RS8010sj_Init();
    /*---------------------------    GPS     -----------------------------------*/
    GPS_Configure();
    /*---------------------------    SD          -------------------------------*/
    SD_POWER_Control();//ʹ�ܵ�Դ
    while( Recorder_Config() != OK )
    {
        delay_ms( 200 );
    }
    EEPROM_Loading_Config();//��ȡУ׼ϵ��
    Read_CfgFile();//��ȡ�����ļ�
    EEPROM_Loading_DeviceInformation();//��ȡ�汾��Ϣ���豸ID
    Add_LocalFile();//��Ӽ�¼
    Creat_Log_File();//������־�ļ�
    /*----------------------------------BNO055��ʼ��---------------------------*/
    delay_ms( 100 );
    SENSER_POWER_Control();//ʹ�ܴ�������Դ
    delay_ms( 100 );
    /*---------------------------    ICM������   ------------------------------*/
    ICM2060x_Init();
    /****************************************************************************/
    /*******************************�����Լ�*************************************/
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
            /*---------------------------    BNO���ݶ�ȡ  ------------------------------*/
            if( send_flag == 1 )
            {
                BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_ReadData( BNO_TxData ) );
            }
            //    Get_local_gravity();//���㵱������
        }
        if( Time3_Flag_50hz )
        {
            Time3_Flag_50hz = 0;
            System_Status();//ϵͳ�Լ첻ͨ��������
            Senser_Data_SelfCheck();
            if( IsSenser_Ready() )
            {
                FFT_init();//FFT�㷨ϵ��
                //       AHRS_EKF_Init();
                GPIO_ResetBits( GPIOC, GPIO_Pin_6 ); //�Լ������LED��
                //       IWDG_Init(4,500);//���ÿ��Ź� ���ʱ��1s
                break;
            }
        }
        if( Time3_Flag_5hz )
        {
            Time3_Flag_5hz = 0;
            Data_Transmit_232();//232��������
            Record_Data( &file_loc ); //5Hz ���ݴ洢
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
    /*******************************������***************************************/
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
        /*---------------------------- Ƶ�ʿ���ִ�д����� ------------------------*/
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
            Senser_data_process();//����������ѡ��
            Senserdata_filter_H();//100;8hz zacc����
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
            System_Check();//������״̬���
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
            if( pHost_Cmd->OriData_Record_Flag == 1 ) //ԭʼ�����Ƿ��¼1���� ��2����
            {
                Record_Data( &file_loc );
            }
            OriData__Communication_Send();//ԭʼ����ʵʱ����
        }
        if( Time3_Flag_5hz )
        {
            Time3_Flag_5hz = 0;
            if( ( Mag_Calibration_Step != 5 ) && ( Mag_Calibration_Step != 3 ) )
            {
                System_Status();//ϵͳ��������������
            }
            Tradion_Sample();//�жϺ�ʱ��������۲�
            Data_Transmit_232();//��λ��ָ��ͨ��
            //      Record_Log_Data(&file_rcd);//��¼����۲�������־
            OriData_AutoClose_Enable();
            //      if((pSenser_Data->State & 0x03) == 0)
            //      {
            //        IWDG_Feed();//ι��
            //      }
            cnt_5hz++;
        }
        if( Time3_Flag_2hz )
        {
            Time3_Flag_2hz = 0;
            if( time_count % 2 == 0 )
            {
                Time_Read_Run();//��ȡRTCʱ��
            }
            if( ( pRTC_Time->hour == 0 ) && ( pRTC_Time->min == 0 ) && ( pRTC_Time->sec < 1 ) ) //count_rec>=72000)//20hÿСʱ72000�� ÿСʱ��һ������
            {
                Add_Ending_Message( &file_loc ); //�����ļ�������־
                f_sync( &file_loc );
                f_close( &file_loc );
                pGps_Data->Log_Lon = pGps_Data->Lon;//updata the location
                pGps_Data->Log_Lat = pGps_Data->Lat;
                Creat_Log_File();//������־�ļ�
                Add_LocalFile();//��Ӽ�¼
            }
            if( send_time >= 30 )
            {
                f_sync( &file_loc );
                send_time = 0;
            }//4ms ����ͬ��
            if( fft_i >= 1024 )
            {
                FFT_Cal();
                //        FFT_AXToVel();
                //        FFT_AYToVel();
                fft_i = 0;
            }
            RTC_Time_Adj();
            /*----------------------------   ϵͳ�����λ   ------------------------*/
            if( pHost_Cmd->System_Restart_Flag == 1 )
            {
                __set_FAULTMASK( 1 ); //�ر������ж�
                NVIC_SystemReset();//��λ����
            }
            time_count++;
        }
    }
}

static Init_RES Recorder_Config()
{
    //  float sdsize;
    mem_init();/* mem init for Fatfs */
    //  SD_Init();//SD����ʼ��
    if( SD_Init() != SD_OK ) //�����᷵��0
    {
        return ERR;/* SD Init Error */
    }
    //  if(SD_GetCardInfo(&SDCardInfo)==SD_OK)
    //  {
    //    sdsize = (float)((double)SDCardInfo.CardCapacity/1048576.0);/* Get SD CardCapacity  ��λ M */
    //  }
    //  else
    //    return ERR;/* Get SD CardCapacity Error */
    return OK;
}
/*---------------------------- SD����Դʹ��  ------------------------*/
void SD_POWER_Control() //PA15,����ʹ�ܵ�Դ
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
/*---------------------------- BNO��Դʹ��  ------------------------*/
void SENSER_POWER_Control() //PB12,����ʹ�ܵ�Դ
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
/*---------------------------- ϵͳ״ָ̬ʾ  ------------------------*/
void System_Status_Init() //PC6,ϵͳ״̬
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

void OriData_AutoClose_Enable()//������¼����15�죬�Զ��ر�
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
///*---------------------------- ����۲����ݼ�¼  ------------------------*/
//void Log_Data_Record()
//{
//  /*---------------------------- �ֶ�ģʽ  ------------------------*/
//  if(pHost_Cmd->Routine_Data_flag == 0x02)//����1024s���ݼ�¼
//  {
//    Get_Routine_Data();//���㳣��۲�����
//    Record_Log_Data(&file_rcd);
//  }
//  /*---------------------------- �Զ�ģʽ  ------------------------*/
////  if(pHost_Cmd->Scan_Frequence == 0x0A) //0.5Сʱһ��
////  {
////  }else if(pHost_Cmd->Scan_Frequence == 0x01) //1Сʱһ��
////  {
////
////  }else if(pHost_Cmd->Scan_Frequence == 0x02) //2Сʱһ��
////  {
////
////  }else if(pHost_Cmd->Scan_Frequence == 0x04) //4Сʱһ��
////  {
////
////  }
//
//}
/*-----------------------��ʱ��������۲�-------------------------------------*/
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
