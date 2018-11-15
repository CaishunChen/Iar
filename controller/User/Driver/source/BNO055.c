/************************************************************************************************************************/
#include "BNO055.h"
#include "stm32f4xx.h"
#include "uart_init.h"
#include "Library_Filter.h"
#include "filter.h"
#include "eeprom.h"
#include <string.h>

IMU_DataAD_TypeDef IMU_DataAD;
IMU_DataAD_TypeDef* pIMU_DataAD = &IMU_DataAD;

d_Senser_Data Senser_Data;
d_Senser_Data* pSenser_Data = &Senser_Data;


extern d_Senser_Data*  pSenser_filtered;


u8 BNO055_RX_BUF[20];
RingBuffer BNO055_RX_RingBuff;

u8 RX_WorkingBuff_Count = 0;
u8 BNO055_RX_WorkingBuff[100];

int config_step = 0;
int send_step = 1;
u8 BNO_TxData[20];
uint8_t send_flag = 0;

void BNO_Configure()
{
    rbInitialize( &BNO055_RX_RingBuff, BNO055_RX_BUF, sizeof( BNO055_RX_BUF ) );
}

/*---------------------------- BNO配置寄存器初始化 ------------------------*/
void BNO055_Init()
{
    switch( config_step )
    {
        case 0:
            BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_WriteData( BNO_TxData, Page_ID, 0x01 ) ); //07 01 进入page 1
            break;
        case 1:
            BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_WriteData( BNO_TxData, ACC_Config, 0x09 ) ); //08 09 配置+/-4g 31.25hz截止
            break;
        case 2:
            BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_WriteData( BNO_TxData, MAG_Config, 0x09 ) ); //09 09 配置高精确模式  30hz输出
            break;
        case 3:
            BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_WriteData( BNO_TxData, GYR_Config_0, 0x1C ) ); //   125  0A 1A 配置500 47Hz
            break;
        case 4:
            BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_WriteData( BNO_TxData, Page_ID, 0x00 ) ); //07 00 进入page 0
            break;
        //  case 5:
        //
        //    BNO055_UARTDMAWriteData(BNO_TxData,Chose_BNO_WriteData(BNO_TxData,0X3F,0x80));// 外部晶振
        ////        BNO055_UARTDMAWriteData(BNO_TxData,Chose_BNO_WriteData(BNO_TxData,OPR_MODE,0x08));//3d 08 进入IMU-mode
        //
        //    break;
        //  case 6:
        //
        //    BNO055_UARTDMAWriteData(BNO_TxData,Chose_BNO_WriteData(BNO_TxData,PWR_MODE,0x00));// 3e 00 入normal-mode
        //
        //    break;
        //  case 7:
        //
        //    BNO055_UARTDMAWriteData(BNO_TxData,Chose_BNO_WriteData(BNO_TxData,OPR_MODE,0x07));//3d 07 进入AMG-mode
        ////        BNO055_UARTDMAWriteData(BNO_TxData,Chose_BNO_WriteData(BNO_TxData,OPR_MODE,0x08));//3d 08 进入IMU-mode
        //
        //    break;
        case 5:
            BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_WriteData( BNO_TxData, PWR_MODE, 0x00 ) ); // 3e 00 入normal-mode
            break;
        case 6:
            BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_WriteData( BNO_TxData, OPR_MODE, 0x07 ) ); //3d 07 进入AMG-mode
            //        BNO055_UARTDMAWriteData(BNO_TxData,Chose_BNO_WriteData(BNO_TxData,OPR_MODE,0x08));//3d 08 进入IMU-mode
            break;
        //  case 0:
        //
        //    BNO055_UARTDMAWriteData(BNO_TxData,Chose_BNO_WriteData(BNO_TxData,Page_ID,0x00));//07 00 进入page 0
        //    break;
        //
        //  case 1:
        //
        //    BNO055_UARTDMAWriteData(BNO_TxData,Chose_BNO_WriteData(BNO_TxData,PWR_MODE,0x00));// 3e 00 入normal-mode
        //    break;
        //
        //  case 2:
        //
        //    BNO055_UARTDMAWriteData(BNO_TxData,Chose_BNO_WriteData(BNO_TxData,OPR_MODE,0x00));
        //    break;
        //  case 3:
        //
        //    BNO055_UARTDMAWriteData(BNO_TxData,Chose_BNO_WriteData(BNO_TxData,OPR_MODE,0x27));
        //    break;
        //
        default:
            send_flag = 1;
            break;
    }
}

void BNO055_Reg_Check1( void )
{
    while( !rbIsEmpty( &BNO055_RX_RingBuff ) )
    {
        uint8_t cur = rbPop( &BNO055_RX_RingBuff );
        if( RX_WorkingBuff_Count == 0 )
        {
            BNO055_RX_WorkingBuff[0] = BNO055_RX_WorkingBuff[1];
            BNO055_RX_WorkingBuff[1] =  cur;
            if( ( BNO055_RX_WorkingBuff[0] == 0xEE ) && ( BNO055_RX_WorkingBuff[1] == 0x01 ) ) //发送正确时 返回0XEE  0X01
            {
                RX_WorkingBuff_Count = 2;
                config_step++;
                RX_WorkingBuff_Count = 0;
                memset( BNO055_RX_WorkingBuff, 0, 2 );
            }
        }
    }
}

/*---------------------------- BNO配置寄存器校验 ---------------------------*/
void BNO055_Reg_Check( u8 regaddr, u8 dat )
{
    while( !rbIsEmpty( &BNO055_RX_RingBuff ) )
    {
        uint8_t cur = rbPop( &BNO055_RX_RingBuff );
        if( RX_WorkingBuff_Count == 0 )
        {
            BNO055_RX_WorkingBuff[0] = BNO055_RX_WorkingBuff[1];
            BNO055_RX_WorkingBuff[1] =  cur;
            if( ( BNO055_RX_WorkingBuff[0] == 0xEE ) && ( BNO055_RX_WorkingBuff[1] == 0x01 ) ) //发送正确时 返回0XEE  0X01
            {
                RX_WorkingBuff_Count = 2;
            }
            else
            {
                BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_WriteData( BNO_TxData, PWR_MODE, ( uint8_t )dat ) );
                //send_flag--;
            }
        }
        else
        {
            BNO055_RX_WorkingBuff[RX_WorkingBuff_Count++] = cur;
        }
        if( RX_WorkingBuff_Count >= 2 )
        {
            config_step++;
            RX_WorkingBuff_Count = 0;
        }
    }
}


u8 BNO055_IsReadyToRead()
{
    return !rbIsEmpty( &BNO055_RX_RingBuff );
}

//  static
uint16_t err_send_cnt = 0;
static uint8_t corr_flag = 0;
void BNO055_Update()
{
    while( !rbIsEmpty( &BNO055_RX_RingBuff ) )
    {
        uint8_t cur = rbPop( &BNO055_RX_RingBuff );
        if( RX_WorkingBuff_Count == 0 )
        {
            BNO055_RX_WorkingBuff[0] = BNO055_RX_WorkingBuff[1];
            BNO055_RX_WorkingBuff[1] = cur;
            if( ( BNO055_RX_WorkingBuff[0] == 0xBB ) && ( BNO055_RX_WorkingBuff[1] == 0x12 ) )
            {
                RX_WorkingBuff_Count = 2;
            }
            if( ( BNO055_RX_WorkingBuff[0] == 0xEE ) && ( ( BNO055_RX_WorkingBuff[1] == 0x07 ) || ( BNO055_RX_WorkingBuff[1] == 0x06 ) ) )
            {
                BNO055_UARTDMAWriteData( BNO_TxData, Chose_BNO_ReadData( BNO_TxData ) );
                memset( BNO055_RX_WorkingBuff, 0, 2 );
            }
        }
        else
        {
            BNO055_RX_WorkingBuff[RX_WorkingBuff_Count++] = cur;
        }
        if( RX_WorkingBuff_Count >= 20 )
        {
            if( ( BNO055_RX_WorkingBuff[19] == 0XBB ) || ( BNO055_RX_WorkingBuff[19] == 0XEE ) )
            {
                BNO055_RX_WorkingBuff[19] = corr_flag ;
                BNO055_Decode();
                RX_WorkingBuff_Count = 0;
                memset( BNO055_RX_WorkingBuff, 0, 20 );
            }
            else
            {
                BNO055_Decode();
                RX_WorkingBuff_Count = 0;
                memset( BNO055_RX_WorkingBuff, 0, 20 );
                corr_flag = BNO055_RX_WorkingBuff[19];
                err_send_cnt++;
            }
        }
    }
}

//void BNO055_Update()
//{
//    while(!rbIsEmpty(&BNO055_RX_RingBuff))
//    {
//
//      uint8_t cur = rbPop(&BNO055_RX_RingBuff);
//      if(RX_WorkingBuff_Count==0)
//      {
//        if( corr_flag ==1)
//        {
//        BNO055_RX_WorkingBuff[0] = BNO055_RX_WorkingBuff[20];
//        BNO055_RX_WorkingBuff[1] = BNO055_RX_WorkingBuff[21];
//        }else{
//        BNO055_RX_WorkingBuff[0] = BNO055_RX_WorkingBuff[1];
//        BNO055_RX_WorkingBuff[1] = cur;
//      }
//        if((BNO055_RX_WorkingBuff[0]==0xBB)&&(BNO055_RX_WorkingBuff[1]==0x12))
//        {
//          RX_WorkingBuff_Count = 2;
//
//        }
//        if((BNO055_RX_WorkingBuff[0]==0xEE)&&((BNO055_RX_WorkingBuff[1]==0x07)||(BNO055_RX_WorkingBuff[1]==0x06)))
//        {
//
//          BNO055_UARTDMAWriteData(BNO_TxData,Chose_BNO_ReadData(BNO_TxData));
//          corr_flag = 0;
//          memset(BNO055_RX_WorkingBuff,0,2);
//
//        }
//
//      }
//      else
//      {
//        BNO055_RX_WorkingBuff[RX_WorkingBuff_Count++] = cur;
//      }
//      if(RX_WorkingBuff_Count>=22)
//      {
//        if(((BNO055_RX_WorkingBuff[20] == 0XBB)&&(BNO055_RX_WorkingBuff[21] == 0X12))
//           ||((BNO055_RX_WorkingBuff[20] == 0XEE)&&((BNO055_RX_WorkingBuff[21]==0x07)||(BNO055_RX_WorkingBuff[21]==0x06)))){
//
//        BNO055_Decode();
//        RX_WorkingBuff_Count = 0;
//        memset(BNO055_RX_WorkingBuff,0,20);
//        corr_flag = 1;
//        err_send_cnt++;
//     }
//     else
//     {
//       corr_flag = 0;
//             RX_WorkingBuff_Count = 0;
//        memset(BNO055_RX_WorkingBuff,0,20);
//     }
//      }
//    }
//}


void BNO055_Decode()
{
    pIMU_DataAD->ay   = ( s16 )( ( ( u16 )BNO055_RX_WorkingBuff[3] << 8 | ( u16 )BNO055_RX_WorkingBuff[2] & 0x00ff ) );
    pIMU_DataAD->ax   = ( s16 )( ( ( u16 )BNO055_RX_WorkingBuff[5] << 8 | ( u16 )BNO055_RX_WorkingBuff[4] & 0x00ff ) );
    pIMU_DataAD->az   = ( s16 )( ( ( u16 )BNO055_RX_WorkingBuff[7] << 8 | ( u16 )BNO055_RX_WorkingBuff[6] & 0x00ff ) );
    pIMU_DataAD->mx   = ( s16 )( ( u16 )BNO055_RX_WorkingBuff[9] << 8  | ( u16 )BNO055_RX_WorkingBuff[8] & 0x00ff );
    pIMU_DataAD->my   = ( s16 )( ( u16 )BNO055_RX_WorkingBuff[11] << 8 | ( u16 )BNO055_RX_WorkingBuff[10] & 0x00ff );
    pIMU_DataAD->mz   = ( s16 )( ( u16 )BNO055_RX_WorkingBuff[13] << 8 | ( u16 )BNO055_RX_WorkingBuff[12] & 0x00ff );
    pIMU_DataAD->gx   = ( s16 )( ( u16 )BNO055_RX_WorkingBuff[15] << 8 | ( u16 )BNO055_RX_WorkingBuff[14] & 0x00ff );
    pIMU_DataAD->gy   = ( s16 )( ( u16 )BNO055_RX_WorkingBuff[17] << 8 | ( u16 )BNO055_RX_WorkingBuff[16] & 0x00ff );
    pIMU_DataAD->gz   = ( s16 )( ( u16 )BNO055_RX_WorkingBuff[19] << 8 | ( u16 )BNO055_RX_WorkingBuff[18] & 0x00ff );
    /*------------------------------     读取欧拉角   --------------------------*/
    //  pSenser_Data->BNOy   = (s16)((u16)BNO055_RX_WorkingBuff[21]<<8 | (u16)BNO055_RX_WorkingBuff[20] & 0x00ff);
    //  pSenser_Data->BNOr   = (s16)((u16)BNO055_RX_WorkingBuff[23]<<8 | (u16)BNO055_RX_WorkingBuff[22] & 0x00ff);
    //  pSenser_Data->BNOp   = (s16)((u16)BNO055_RX_WorkingBuff[25]<<8 | (u16)BNO055_RX_WorkingBuff[24] & 0x00ff);
    /*--------------------------------------------------------------------------*/
    pSenser_Data->BNOax     = -( ( pIMU_DataAD->ax / 100.0 ) - ACC_X_OFFSET ) * K_ACC_X; //unit:m/s2
    pSenser_Data->BNOay     = ( ( pIMU_DataAD->ay / 100.0 ) - ACC_Y_OFFSET ) * K_ACC_Y;
    pSenser_Data->BNOaz     = -( ( pIMU_DataAD->az / 100.0 ) - ACC_Z_OFFSET ) * K_ACC_Z;
    pSenser_Data->BNOgx     = -pIMU_DataAD->gx / 3600.0;//unit:rps
    pSenser_Data->BNOgy     =  pIMU_DataAD->gy / 3600.0;
    pSenser_Data->BNOgz     =  pIMU_DataAD->gz / 6560.0;
    pSenser_Data->BNOmx     = ( pIMU_DataAD->mx - Config.dMx_offset ) / 1600.0;
    pSenser_Data->BNOmy     = -( pIMU_DataAD->my - Config.dMy_offset ) * Config.MAGXY_factor / 1600.0; //unit:Gs
    pSenser_Data->BNOmz     = -( pIMU_DataAD->mz - Config.dMz_offset ) * Config.MAGXZ_factor / 1600.0;
    /*------------------------------ BNO原始数据滤波 ---------------------------*/
    //  if(pSenser_Data->ICM_flag != 1)
    //  {
    //    if(BNO_Flag == 1)
    //    {
    //      Threshold_filter();//去除数据跳变
    //    }
    BNOdata_filter_Acc();
    BNOdata_filter_Gyro();
    //    butterworth_filter();
    //  }
}
/*---------------------------- BNO发送 ------------------------------------*/
uint8_t Chose_BNO_WriteData( u8* txbuf, u8 regaddr, u8 dat )
{
    uint16_t data_totalnumber = 0;
    txbuf[data_totalnumber++] = ( uint8_t )0xAA;
    txbuf[data_totalnumber++] = ( uint8_t )0x00;
    txbuf[data_totalnumber++] = ( uint8_t )( regaddr );
    txbuf[data_totalnumber++] = ( uint8_t )0X01;
    txbuf[data_totalnumber++] = ( uint8_t )( dat );
    return data_totalnumber;
}
/*---------------------------- BNO读取 -------------------------------------*/
uint8_t Chose_BNO_ReadData( u8* trxbuf )
{
    uint16_t data_totalnumber = 0;
    trxbuf[data_totalnumber++] = ( uint8_t )0xAA;
    trxbuf[data_totalnumber++] = ( uint8_t )0x01;
    trxbuf[data_totalnumber++] = ( uint8_t )0x08;
    trxbuf[data_totalnumber++] = ( uint8_t )0x12;
    //    trxbuf[data_totalnumber++] = (uint8_t)0xAA;
    //  trxbuf[data_totalnumber++] = (uint8_t)0x01;
    //  trxbuf[data_totalnumber++] = (uint8_t)0x3F;
    //  trxbuf[data_totalnumber++] = (uint8_t)0x01;
    return data_totalnumber;
}






