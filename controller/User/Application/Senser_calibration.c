#include <math.h>
#include <stdio.h>
#include "Senser_calibration.h"
#include "BNO055.h"

uint8_t Mag_Calibration_Step = 0;
uint8_t Acc_Calibration_Step = 0;
float BNO055_maxx, BNO055_maxy, BNO055_maxz;
float BNO055_minx, BNO055_miny, BNO055_minz;
extern d_Senser_Data*  pSenser_Data;

/*---------------------------- 地磁校准  ------------------------*/
/*
校准步骤：
1. 水平放置，等待绿灯慢闪，转动；
2.竖直放置，绿灯快闪，转动；
3.水平放置，若绿灯常亮，表示校准成功；慢闪校准不成功，重新校准。
*/
void Mag_Calibration_Operation()
{
    Mag_Calibration_Step_Update();
    switch( Mag_Calibration_Step )
    {
        case 0:
            Mag_calibration_init();
            Mag_Calibration_Step++;
            break;
        case 1:
            Traverse_magvalue_XY();
            break;
        case 2:
            Cali_parameter_caculation_XY();
            Mag_Calibration_Step++;
            break;
        case 3:
            Traverse_magvalue_Z();
            break;
        case 4:
            Cali_parameter_caculation_Z();
            if( isCali_XYFactor_Check_Succeed() && isCali_XZFactor_Check_Succeed() ) //校验完成，保存数据
            {
                Save_Cali_parameter();
                Mag_Calibration_Step++;
            }
            else
            {
                Mag_calibration_offset();
                Mag_Calibration_Step = 0;
            }
            break;
        default:
            break;
    };
}
static uint8_t step1_cnt = 0, step3_cnt = 0;
void Mag_Calibration_Step_Update()
{
    switch( Mag_Calibration_Step )
    {
        case 1:
            if( fabs( pSenser_Data->Zacc ) < 1.0 )
            {
                step1_cnt++;
            }
            if( step1_cnt > 50 )
            {
                Mag_Calibration_Step = 2;
            }
            break;
        case 3:
            //    if((((fabs(pSenser_Data->BNOaz)-9.8) <1.0))&&((fabs(pSenser_Data->BNOax) <1.0)||(fabs(pSenser_Data->BNOay)  <1.0)))
            if( ( pSenser_Data->Zacc  < -9 ) && ( ( ( fabs( pSenser_Data->BNOax ) ) < 1.0 ) || ( ( fabs( pSenser_Data->BNOay ) ) < 1.0 ) ) )
            {
                step3_cnt++;
            }
            if( step3_cnt > 50 )
            {
                Mag_Calibration_Step = 4;
            }
            break;
        default:
            break;
    };
}

void Mag_calibration_init()
{
    Mag_calibration_init_XY();
    Mag_calibration_init_Z();
}
void Mag_calibration_init_XY( void )
{
    BNO055_maxx = -100;
    BNO055_maxy = -100;
    BNO055_minx = 99;
    BNO055_miny = 99;
}
void Mag_calibration_init_Z( void )
{
    BNO055_maxz = -100;
    BNO055_minz = 99;
}

void Traverse_magvalue_XY( void )
{
    if( BNO055_minx > pIMU_DataAD->mx )
    {
        BNO055_minx = pIMU_DataAD->mx ;
    }
    if( BNO055_miny > pIMU_DataAD->my )
    {
        BNO055_miny = pIMU_DataAD->my ;
    }
    if( BNO055_maxx < pIMU_DataAD->mx )
    {
        BNO055_maxx = pIMU_DataAD->mx ;
    }
    if( BNO055_maxy < pIMU_DataAD->my )
    {
        BNO055_maxy = pIMU_DataAD->my ;
    }
}
void Cali_parameter_caculation_XY()
{
    Config.dMx_offset = ( BNO055_maxx + BNO055_minx ) / 2;
    Config.dMy_offset = ( BNO055_maxy + BNO055_miny ) / 2;
    Config.dMx_min = BNO055_minx;
    Config.dMy_min = BNO055_miny;
    Config.dMx_max = BNO055_maxx;
    Config.dMy_max = BNO055_maxy;
}
uint8_t isCali_XYFactor_Check_Succeed( void )
{
    float magx_range, magy_range;
    magx_range = BNO055_maxx - BNO055_minx;
    magy_range = BNO055_maxy - BNO055_miny;
    if( ( magx_range == 0 ) || ( magx_range == 959 ) )
    {
        return 0;
    }
    if( magy_range == 0 )
    {
        return 0;
    }
    Config.MAGXY_factor = magx_range / magy_range;
    if( fabs( Config.MAGXY_factor - 1.0 ) <= 0.2 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
void Traverse_magvalue_Z()
{
    if( BNO055_minz > pIMU_DataAD->mz )
    {
        BNO055_minz = pIMU_DataAD->mz;
    }
    if( BNO055_maxz < pIMU_DataAD->mz )
    {
        BNO055_maxz = pIMU_DataAD->mz;
    }
}
void Cali_parameter_caculation_Z()
{
    Config.dMz_offset = ( BNO055_maxz + BNO055_minz ) / 2;
    Config.dMz_min = BNO055_minz;
    Config.dMz_max = BNO055_maxz;
}
uint8_t isCali_XZFactor_Check_Succeed( void )
{
    float magx_range, magz_range;
    magx_range = BNO055_maxx - BNO055_minx;
    magz_range = BNO055_maxz - BNO055_minz;
    if( magx_range == 0 )
    {
        return 0;
    }
    if( magz_range == 0 )
    {
        return 0;
    }
    Config.MAGXZ_factor = magx_range / magz_range;
    if( fabs( Config.MAGXZ_factor - 1.0 ) <= 0.2 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
void Save_Cali_parameter()
{
    Config.Mag_is_good = 0xA66A;
    EEPROM_Write_Config();
}

void Mag_calibration_offset( void )
{
    Config.Mag_is_good = 0x00;
    Config.dMx_offset = 0.0;
    Config.dMy_offset = 0.0;
    Config.dMz_offset = 0.0;
    Config.dMx_min = 0.0;
    Config.dMy_min = 0.0;
    Config.dMz_min = 0.0;
    Config.dMx_max = 0.0;
    Config.dMy_max = 0.0;
    Config.dMz_max = 0.0;
    Config.MAGXY_factor = 1.0;
    Config.MAGXZ_factor = 1.0;
    Config.Acc_is_good = 0x00;
    Config.dAx_offset = 0.0;
    Config.dAy_offset = 0.0;
    Config.dAz_offset = 0.0;
    EEPROM_Write_Config();
}

/*---------------------------- 加速度校准  ------------------------*/
//  static float acc[6] = {0};
//float Acc_Average[6][2] = {0};
//uint16_t ACC_Cail_cnt = 0;
//
//void Acc_Calibration_Operation()
//{
//
//    Acc_Calibration_Step_Update();
//    switch(Acc_Calibration_Step)
//    {
//     /*---------------------------- 第一个轴  ------------------------*/
//    case 0:
//
//      Acc_Average[4][0] += pSenser_Data->Zacc;
//      if(ACC_Cail_cnt>99)
//      {
//        ACC_Cail_cnt = 0;
//        Acc_Average[4][1] = 0.01*Acc_Average[4][0] ;
//      }
//
//      break;
//    case 1:
//      Acc_Average[5][0] += pSenser_Data->Zacc;
//      if(ACC_Cail_cnt>99)
//      {
//        ACC_Cail_cnt = 0;
//        Acc_Average[5][1] = 0.01*Acc_Average[5][0] ;
//      }
//      break;
//
//      /*---------------------------- 第二个轴  ------------------------*/
//    case 2:
//      Cali_parameter_1();
//      Acc_Average[2][0] += pSenser_Data->Xacc;
//      if(ACC_Cail_cnt>99)
//      {
//        ACC_Cail_cnt = 0;
//        Acc_Average[2][1] = 0.01*Acc_Average[2][0] ;
//      }
//      break;
//    case 3:
//      Acc_Average[3][0] += pSenser_Data->Xacc;
//      if(ACC_Cail_cnt>99)
//      {
//        ACC_Cail_cnt = 0;
//        Acc_Average[3][1] = 0.01*Acc_Average[3][0] ;
//      }
//      break;
//
//      /*---------------------------- 第三个轴  ------------------------*/
//    case 4:
//      Cali_parameter_2();
//      Acc_Average[0][0] += pSenser_Data->Yacc;
//      if(ACC_Cail_cnt>99)
//      {
//        ACC_Cail_cnt = 0;
//        Acc_Average[0][1] = 0.01*Acc_Average[0][0] ;
//      }
//      break;
//
//    case 5:
//      Acc_Average[1][0] += pSenser_Data->Yacc;
//      if(ACC_Cail_cnt>99)
//      {
//        ACC_Cail_cnt = 0;
//        Acc_Average[1][1] = 0.01*Acc_Average[1][0] ;
//      }        break;
//
//    case 6:
//      Cali_parameter_3();
//      break;
//    default:
//      break;
//    };
//
//}
//static uint8_t XY_FLAG = 0;
///*
//XY_FLAG =1 ,Y轴
//XY_FLAG =2，X轴
//*/
//void Acc_Calibration_Step_Update()
//{
//  ACC_Cail_cnt++;
//  switch(Acc_Calibration_Step)
//  {
//  case 0:
//    if(fabs(Acc_Average[4][1]-Acc_Average1[2]) >15)
//    {Acc_Calibration_Step ++;
//    ACC_Cail_cnt = 0;}
//    break;
//  case 1:
//
//    if((fabs(Acc_Average1[2]) < 2) && ((fabs(Acc_Average1[0])>8)||(fabs(Acc_Average1[1]) > 8)))
//    {Acc_Calibration_Step ++;
//    ACC_Cail_cnt = 0;}
//    if(fabs(Acc_Average1[1])>8)
//    {
//      XY_FLAG =2;
//    }else if(fabs(Acc_Average1[0])>8)
//    {
//      XY_FLAG =1;
//    }
//    break;
//  case 2:
//    if((XY_FLAG==1)&&(fabs(Acc_Average[2][1] - Acc_Average1[0]) > 15))//Y轴
//    {
//      Acc_Calibration_Step ++;
//      ACC_Cail_cnt = 0;
//    }else if((XY_FLAG==2)&&(fabs(Acc_Average[2][1] - Acc_Average1[0]) > 15))
//    {Acc_Calibration_Step ++;
//    ACC_Cail_cnt = 0;}
//      break;
//  case 3:
//    if((XY_FLAG==1)&&(fabs(Acc_Average1[0]) > 9)&&(ACC_Cail_cnt==100))//Y轴
//    {
//    Acc_Calibration_Step ++;
//    ACC_Cail_cnt = 0;
//    }else if((XY_FLAG==2)&&(fabs(Acc_Average1[1]) > 9))
//    {
//    Acc_Calibration_Step ++;
//    ACC_Cail_cnt = 0;
//    }
//      break;
//  case 4:
//    if((XY_FLAG==1)&&(fabs(Acc_Average[0][1] - Acc_Average1[1]) > 15))//Y轴
//    {
//      Acc_Calibration_Step ++;
//      ACC_Cail_cnt = 0;
//    }else if((XY_FLAG==2)&&(fabs(Acc_Average[0][1] - Acc_Average1[1]) > 15))
//    {Acc_Calibration_Step ++;
//    ACC_Cail_cnt = 0;}
//
//    break;
//  case 5:
//      Acc_Calibration_Step ++;
//    break;
//  default:
//    break;
//  };
//}
//
//static float Acc_Z_OFFSET = 0,K_Acc_Z = 0,Acc_X_OFFSET = 0,K_Acc_X = 0,Acc_Y_OFFSET = 0,K_Acc_Y = 0;
//uint8_t Flag_X =0;
//void Cali_parameter_1()
//{
//  Acc_Z_OFFSET =(fabs(Acc_Average[4][1]) +fabs(Acc_Average[5][1]))/2 - 9.7949;
//  K_Acc_Z = 9.7949/(fabs((Acc_Average[4][1]) +fabs(Acc_Average[5][1]))/2);
//}
//
//void Cali_parameter_2()
//{
//  static float ACC_XY1_OFFSET = 0,K_ACC_XY1 = 0;
//
//  ACC_XY1_OFFSET =(fabs(acc[2]) +fabs(acc[3]))/2 - 9.7949;
//  K_ACC_XY1 = 9.7949/((fabs(acc[2]) +fabs(acc[3]))/2);
//
//  if((fabs(Acc_Average[1][1]) > 8)&&(fabs(Acc_Average[0][1]) < 2))
//  {
//    Acc_X_OFFSET = ACC_XY1_OFFSET;
//    K_Acc_X = K_ACC_XY1;
//    Flag_X = 1;
//  }else
//  {
//    Acc_Y_OFFSET = ACC_XY1_OFFSET;
//    K_Acc_Y = K_ACC_XY1;
//  }
//
//}
//
//void Cali_parameter_3()
//{
//  static float ACC_XY2_OFFSET = 0,K_ACC_XY2 = 0;
//
//  ACC_XY2_OFFSET =(fabs(acc[4]) +fabs(acc[5]))/2 - 9.7949;
//  K_ACC_XY2 = 9.7949/((fabs(acc[4]) +fabs(acc[5]))/2);
//
//  if(Flag_X == 1)
//  {
//    Acc_Y_OFFSET = ACC_XY2_OFFSET;
//    K_Acc_Y = K_ACC_XY2;
//  }else
//  {
//    Acc_X_OFFSET = ACC_XY2_OFFSET;
//    K_Acc_X = K_ACC_XY2;
//  }
//}