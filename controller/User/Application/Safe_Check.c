#include "Safe_Check.h"

extern d_Senser_Data*  pSenser_Data;

#define MAG_NORMAL      0
#define MAG_ABNORMAL    1
#define MAG_DISCONNECT  2

#define BNO_ACC_NORMAL      0
#define BNO_ACC_ABNORMAL    1
#define BNO_ACC_DISCONNECT  2

#define ICM_ACC_NORMAL      0
#define ICM_ACC_ABNORMAL    1
#define ICM_ACC_DISCONNECT  2

#define GYRO_NORMAL      0
#define GYRO_ABNORMAL    1
#define GYRO_DISCONNECT  2

uint8_t    mag_Healthstate = 1;
uint8_t    BNOacc_Healthstate = 1;
uint8_t    ICMacc_Healthstate = 1;
uint8_t    gyro_Healthstate = 1;

uint16_t   mag_abnormal_count = 0;
uint16_t   mag_normal_count = 0;
uint16_t   mag_disconnect_count = 0;
uint16_t   bno_acc_normal_count = 0;
uint16_t   bno_acc_abnormal_count = 0;
uint16_t   bno_acc_disconnect_count = 0;
uint16_t   icm_acc_normal_count = 0;
uint16_t   icm_acc_abnormal_count = 0;
uint16_t   icm_acc_disconnect_count = 0;
uint16_t   gyro_normal_count = 0;
uint16_t   gyro_abnormal_count = 0;
uint16_t   gyro_disconnect_count = 0;

float zacc_temp = 0;
void Senser_Data_SelfCheck()
{
    BNO_Acc_Health_Check();
    Mag_Health_Check();
    Gyro_Health_Check();
    ICM_Acc_Health_Check();
}

uint8_t  IsSenser_Ready()
{
    if( ( BNOacc_Healthstate == 0 ) && ( ICMacc_Healthstate == 0 ) && ( gyro_Healthstate == 0 ) && mag_Healthstate == 0 )
    {
        pSenser_Data->BNO_flag = 1; //BNO正常
        pSenser_Data->State = 0 & ( pSenser_Data->State ); //自检通过
        return 1;
    }
    else
    {
        pSenser_Data->State = 1 | ( pSenser_Data->State );
    }
    return 0;
}

void Mag_Health_Check()
{
    if( fabs( pSenser_Data->BNOmx ) + fabs( pSenser_Data->BNOmy ) + fabs( pSenser_Data->BNOmz ) < 0.01 )
    {
        mag_abnormal_count = 0;
        mag_normal_count = 0;
        mag_disconnect_count++;
        if( mag_disconnect_count >= 20000 )
        {
            mag_disconnect_count = 20000;
        }
    }
    else if( ( fabs( pSenser_Data->BNOmx ) > 0.65 ) || ( fabs( pSenser_Data->BNOmy ) > 0.65 ) || ( fabs( pSenser_Data->BNOmz ) > 0.8 ) )
    {
        mag_disconnect_count = 0;
        mag_normal_count = 0;
        mag_abnormal_count++;
        if( mag_abnormal_count >= 20000 )
        {
            mag_abnormal_count = 20000;
        }
    }
    else
    {
        mag_disconnect_count = 0;
        mag_abnormal_count = 0;
        mag_normal_count++;
        if( mag_normal_count >= 20000 )
        {
            mag_normal_count = 20000;
        }
    }
    if( mag_normal_count >= 50 )
    {
        mag_Healthstate = MAG_NORMAL;
    }
    else if( mag_abnormal_count >= 50 )
    {
        mag_Healthstate = MAG_ABNORMAL;
    }
    else
    {
        mag_Healthstate = MAG_DISCONNECT;
    }
}

void BNO_Acc_Health_Check()
{
    if( ( ( fabs( pSenser_Data->BNOax ) ) + ( fabs( pSenser_Data->BNOay ) ) + ( fabs( pSenser_Data->BNOaz ) ) ) <= 0.01 )
    {
        bno_acc_abnormal_count = 0;
        bno_acc_normal_count = 0;
        bno_acc_disconnect_count++;
        if( bno_acc_disconnect_count >= 20000 )
        {
            bno_acc_disconnect_count = 20000;
        }
    }
    else if( ( ( ( fabs( pSenser_Data->BNOax ) ) + ( fabs( pSenser_Data->BNOay ) ) + ( fabs( pSenser_Data->BNOaz ) ) ) < 18.0 ) &&
             ( fabs( pSenser_Data->BNOax ) < 8 ) && ( fabs( pSenser_Data->BNOay ) < 8 ) && ( fabs( pSenser_Data->BNOaz + 9.7949 ) < 5.0 ) )
    {
        bno_acc_abnormal_count = 0;
        bno_acc_normal_count++;
        if( bno_acc_normal_count >= 20000 )
        {
            bno_acc_normal_count = 20000;
        }
    }
    else if( ( fabs( pSenser_Data->BNOax ) > 8 ) || ( fabs( pSenser_Data->BNOay ) > 8 ) )
    {
        bno_acc_normal_count = 0;
        bno_acc_abnormal_count++;
        if( bno_acc_abnormal_count >= 20000 )
        {
            bno_acc_abnormal_count = 20000;
        }
    }
    if( bno_acc_normal_count >= 50 )
    {
        BNOacc_Healthstate = BNO_ACC_NORMAL;
    }
    else if( bno_acc_abnormal_count >= 50 )
    {
        BNOacc_Healthstate = BNO_ACC_ABNORMAL;
    }
    else
    {
        BNOacc_Healthstate = BNO_ACC_DISCONNECT;
    }
}

//void BNO_Gyro_Health_Check()
//{
//  if((fabs(pSenser_Data->BNOgy)<0.2)&&(fabs(pSenser_Data->BNOgx)<0.2)&&(fabs(pSenser_Data->BNOgz)<0.2))
//  {
//    gyro_abnormal_count=0;
//    gyro_normal_count++;
//    if(gyro_normal_count >= 20000)   gyro_normal_count = 20000;
//  }
//  else if((fabs(pSenser_Data->BNOgy))+(fabs(pSenser_Data->BNOgx))+(fabs(pSenser_Data->BNOgz)) == 0.0)
//  {
//    gyro_abnormal_count=0;
//    gyro_normal_count=0;
//    gyro_disconnect_count++;
//    if(gyro_disconnect_count >= 20000)  gyro_disconnect_count = 20000;
//  }
//  else
//  {
//    gyro_normal_count=0;
//    gyro_abnormal_count++;
//    if(gyro_abnormal_count >= 20000)  gyro_abnormal_count = 20000;
//  }
//
//  if(gyro_normal_count >= 50)         gyro_Healthstate = GYRO_NORMAL;
//  else if(gyro_abnormal_count >= 50)    gyro_Healthstate = GYRO_ABNORMAL;
//  else if(gyro_disconnect_count>=50)    gyro_Healthstate = GYRO_DISCONNECT;
//}
/*----------------------------ICM传感器自检-------------------------------*/
void ICM_Acc_Health_Check()
{
    if( ( ( fabs( pSenser_Data->ICMAX ) ) + ( fabs( pSenser_Data->ICMAY ) ) + ( fabs( pSenser_Data->ICMAZ ) ) ) <= 0.01 )
    {
        icm_acc_abnormal_count = 0;
        icm_acc_normal_count = 0;
        icm_acc_disconnect_count++;
        if( icm_acc_disconnect_count >= 20000 )
        {
            icm_acc_disconnect_count = 20000;
        }
    }
    else if( ( ( ( fabs( pSenser_Data->ICMAX ) ) + ( fabs( pSenser_Data->ICMAY ) ) + ( fabs( pSenser_Data->ICMAZ ) ) ) < 18.0 ) &&
             ( fabs( pSenser_Data->ICMAX ) < 8 ) && ( fabs( pSenser_Data->ICMAY ) < 8 ) && ( fabs( pSenser_Data->ICMAZ + 9.7949 ) < 5.0 ) )
    {
        icm_acc_abnormal_count = 0;
        icm_acc_normal_count++;
        if( icm_acc_normal_count >= 20000 )
        {
            icm_acc_normal_count = 20000;
        }
    }
    else if( ( fabs( pSenser_Data->ICMAX ) > 8 ) || ( fabs( pSenser_Data->ICMAY ) > 8 ) )
    {
        icm_acc_normal_count = 0;
        icm_acc_abnormal_count++;
        if( icm_acc_abnormal_count >= 20000 )
        {
            icm_acc_abnormal_count = 20000;
        }
    }
    if( icm_acc_normal_count >= 50 )
    {
        ICMacc_Healthstate = ICM_ACC_NORMAL;
    }
    else if( icm_acc_abnormal_count >= 50 )
    {
        ICMacc_Healthstate = ICM_ACC_ABNORMAL;
    }
    else
    {
        ICMacc_Healthstate = ICM_ACC_DISCONNECT;
    }
}

void Gyro_Health_Check()
{
    if( ( fabs( pSenser_Data->ICMGY ) < 2 ) && ( fabs( pSenser_Data->ICMGX ) < 2 ) && ( fabs( pSenser_Data->ICMGZ ) < 2 ) )
    {
        gyro_abnormal_count = 0;
        gyro_normal_count++;
        if( gyro_normal_count >= 20000 )
        {
            gyro_normal_count = 20000;
        }
    }
    else if( ( fabs( pSenser_Data->ICMGY ) ) + ( fabs( pSenser_Data->ICMGX ) ) + ( fabs( pSenser_Data->ICMGZ ) ) == 0.0 )
    {
        gyro_abnormal_count = 0;
        gyro_normal_count = 0;
        gyro_disconnect_count++;
        if( gyro_disconnect_count >= 20000 )
        {
            gyro_disconnect_count = 20000;
        }
    }
    else
    {
        gyro_normal_count = 0;
        gyro_abnormal_count++;
        if( gyro_abnormal_count >= 20000 )
        {
            gyro_abnormal_count = 20000;
        }
    }
    if( gyro_normal_count >= 50 )
    {
        gyro_Healthstate = GYRO_NORMAL;
    }
    else if( gyro_abnormal_count >= 50 )
    {
        gyro_Healthstate = GYRO_ABNORMAL;
    }
    else if( gyro_disconnect_count >= 50 )
    {
        gyro_Healthstate = GYRO_DISCONNECT;
    }
}

float local_gravity[20] = {0};

void Get_local_gravity()
{
    static uint8_t cnt_ave = 0;
    static uint8_t gravity_i = 0;
    if( pSenser_Data->BNOaz < -8 )
    {
        if( cnt_ave == 1 )
        {
            return;
        }
        local_gravity[gravity_i] = pSenser_Data->BNOaz;
        zacc_temp += local_gravity[gravity_i];
        if( gravity_i >= 19 )
        {
            zacc_temp *= 0.05;
            cnt_ave = 1;
        }
        gravity_i++;
    }
}

/*----------------------------   传感器状态检测   -------------------------*/
float senser_data_ori[9][2] = {0};

void System_Check()
{
    static uint16_t senser_disconnect_cnt = 0;
    static uint8_t senser_i = 0;
    static uint8_t senser_flag = 0;
    switch( senser_i )
    {
        case 0:
            if( senser_flag == 1 )
            {
                senser_data_ori[0][0] = senser_data_ori[0][1];
                senser_data_ori[1][0] = senser_data_ori[1][1];
                senser_data_ori[2][0] = senser_data_ori[2][1];
                senser_data_ori[3][0] = senser_data_ori[3][1];
                senser_data_ori[4][0] = senser_data_ori[4][1];
                senser_data_ori[5][0] = senser_data_ori[5][1];
                senser_data_ori[6][0] = senser_data_ori[6][1];
                senser_data_ori[7][0] = senser_data_ori[7][1];
                senser_data_ori[8][0] = senser_data_ori[8][1];
                senser_data_ori[0][1] = pSenser_Data->RollRate;
                senser_data_ori[1][1] = pSenser_Data->PitchRate;
                senser_data_ori[2][1] = pSenser_Data->YawRate;
                senser_data_ori[3][1] = pSenser_Data->Xacc;
                senser_data_ori[4][1] = pSenser_Data->Yacc;
                senser_data_ori[5][1] = pSenser_Data->Zacc;
                senser_data_ori[6][1] = pSenser_Data->magX;
                senser_data_ori[7][1] = pSenser_Data->magY;
                senser_data_ori[8][1] = pSenser_Data->magZ;
                senser_i = 0;
            }
            else
            {
                senser_data_ori[0][0] = pSenser_Data->RollRate;
                senser_data_ori[1][0] = pSenser_Data->PitchRate;
                senser_data_ori[2][0] = pSenser_Data->YawRate;
                senser_data_ori[3][0] = pSenser_Data->Xacc;
                senser_data_ori[4][0] = pSenser_Data->Yacc;
                senser_data_ori[5][0] = pSenser_Data->Zacc;
                senser_data_ori[6][0] = pSenser_Data->magX;
                senser_data_ori[7][0] = pSenser_Data->magY;
                senser_data_ori[8][0] = pSenser_Data->magZ;
                senser_i++;
            }
            break;
        case 1:
            senser_data_ori[0][1] = pSenser_Data->RollRate;
            senser_data_ori[1][1] = pSenser_Data->PitchRate;
            senser_data_ori[2][1] = pSenser_Data->YawRate;
            senser_data_ori[3][1] = pSenser_Data->Xacc;
            senser_data_ori[4][1] = pSenser_Data->Yacc;
            senser_data_ori[5][1] = pSenser_Data->Zacc;
            senser_data_ori[6][1] = pSenser_Data->magX;
            senser_data_ori[7][1] = pSenser_Data->magY;
            senser_data_ori[8][1] = pSenser_Data->magZ;
            senser_i--;
            break;
        default:
            break;
    }
    if( ( senser_data_ori[0][0] - senser_data_ori[0][1] == 0 ) &&
            ( senser_data_ori[1][0] - senser_data_ori[1][1] == 0 ) &&
            ( senser_data_ori[2][0] - senser_data_ori[2][1] == 0 ) &&
            ( senser_data_ori[3][0] - senser_data_ori[3][1] == 0 ) &&
            ( senser_data_ori[4][0] - senser_data_ori[4][1] == 0 ) &&
            ( senser_data_ori[5][0] - senser_data_ori[5][1] == 0 ) &&
            ( senser_data_ori[6][0] - senser_data_ori[6][1] == 0 ) &&
            ( senser_data_ori[7][0] - senser_data_ori[7][1] == 0 ) &&
            ( senser_data_ori[8][0] - senser_data_ori[8][1] == 0 ) )
    {
        senser_disconnect_cnt ++;
    }
    else
    {
        senser_disconnect_cnt = 0;
    }
    if( senser_disconnect_cnt >= 500 )
    {
        if( pSenser_Data->Senser_Mode == 1 )
        {
            pSenser_Data->ICM_flag = 0;
            pSenser_Data->Senser_Mode = 0;
        }
        else
        {
            pSenser_Data->BNO_flag = 0;
            pSenser_Data->Senser_Mode = 1;
        }
    }
    senser_flag = 1;
    if( ( pSenser_Data->BNO_flag == 0 ) && ( pSenser_Data->ICM_flag == 0 ) )
    {
        pSenser_Data->State = 0x03 | pSenser_Data->State;
    }
}

/*----------------------------   看门狗设置      -------------------------*/
/*溢出时间:Tout=((4*2^prer)*rlr)/32 (ms).
 prer = 4
 rlr = 500
大致1s*/
void IWDG_Init( u8 prer, u16 rlr )
{
    IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable ); //ê1?ü??IWDG->PR IWDG->RLRμ?D′
    IWDG_SetPrescaler( prer ); //éè??IWDG·??μ?μêy
    IWDG_SetReload( rlr ); //éè??IWDG×°???μ
    IWDG_ReloadCounter(); //reload
    IWDG_Enable();       //ê1?ü?′??1·
}

void IWDG_Feed( void )
{
    IWDG_ReloadCounter();//reload
}