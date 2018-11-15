#include "filter.h"
#include "math.h"
#include "Library_Filter.h"



/*-----------------------------   IIR滤波  -----------------------------------*/
IIR_coeff_Typedef Level1_Filter_Gx;
IIR_coeff_Typedef* pLevel1_Filter_Gx = &Level1_Filter_Gx;
IIR_coeff_Typedef Level1_Filter_Gy;
IIR_coeff_Typedef* pLevel1_Filter_Gy = &Level1_Filter_Gy;
IIR_coeff_Typedef Level1_Filter_Gz;
IIR_coeff_Typedef* pLevel1_Filter_Gz = &Level1_Filter_Gz;
IIR_coeff_Typedef Level1_Filter_Ax;
IIR_coeff_Typedef* pLevel1_Filter_Ax = &Level1_Filter_Ax;
IIR_coeff_Typedef Level1_Filter_Ay;
IIR_coeff_Typedef* pLevel1_Filter_Ay = &Level1_Filter_Ay;
IIR_coeff_Typedef Level1_Filter_Az;
IIR_coeff_Typedef* pLevel1_Filter_Az = &Level1_Filter_Az;


/*---------------------------------  ICM数据滤波    ------------------------*/
void ICMdata_filter_Acc()
{
    cal_iir_coeff( pLevel1_Filter_Ax, 400, 30 ); //(滤波参数，采样频率，滤波频率)
    cal_iir_coeff( pLevel1_Filter_Ay, 400, 30 );
    cal_iir_coeff( pLevel1_Filter_Az, 400, 30 );
    pSenser_Data_filtered->ICMAX = get_iir_output( pLevel1_Filter_Ax, pSenser_Data->ICMAX );
    pSenser_Data_filtered->ICMAY = get_iir_output( pLevel1_Filter_Ay, pSenser_Data->ICMAY );
    pSenser_Data_filtered->ICMAZ = get_iir_output( pLevel1_Filter_Az, pSenser_Data->ICMAZ );
}

void ICMdata_filter_Gyro()
{
    cal_iir_coeff( pLevel1_Filter_Gx, 400, 30 ); //(滤波参数，采样频率，滤波频率)
    cal_iir_coeff( pLevel1_Filter_Gy, 400, 30 );
    cal_iir_coeff( pLevel1_Filter_Gz, 400, 30 );
    pSenser_Data_filtered->ICMGX = get_iir_output( pLevel1_Filter_Gx, pSenser_Data->ICMGX );
    pSenser_Data_filtered->ICMGY = get_iir_output( pLevel1_Filter_Gy, pSenser_Data->ICMGY );
    pSenser_Data_filtered->ICMGZ = get_iir_output( pLevel1_Filter_Gz, pSenser_Data->ICMGZ );
}
/*---------------------------------  BNO数据滤波    ------------------------*/
IIR_coeff_Typedef Level1_Filter_gx;
IIR_coeff_Typedef* pLevel1_Filter_gx = &Level1_Filter_gx;
IIR_coeff_Typedef Level1_Filter_gy;
IIR_coeff_Typedef* pLevel1_Filter_gy = &Level1_Filter_gy;
IIR_coeff_Typedef Level1_Filter_gz;
IIR_coeff_Typedef* pLevel1_Filter_gz = &Level1_Filter_gz;
IIR_coeff_Typedef Level1_Filter_ax;
IIR_coeff_Typedef* pLevel1_Filter_ax = &Level1_Filter_ax;
IIR_coeff_Typedef Level1_Filter_ay;
IIR_coeff_Typedef* pLevel1_Filter_ay = &Level1_Filter_ay;
IIR_coeff_Typedef Level1_Filter_az;
IIR_coeff_Typedef* pLevel1_Filter_az = &Level1_Filter_az;
void BNOdata_filter_Acc()
{
    cal_iir_coeff( pLevel1_Filter_ax, 100, 30 ); //(滤波参数，采样频率，滤波频率)
    cal_iir_coeff( pLevel1_Filter_ay, 100, 30 );
    cal_iir_coeff( pLevel1_Filter_az, 100, 30 );
    pSenser_Data_filtered->BNOax = get_iir_output( pLevel1_Filter_ax, pSenser_Data->BNOax );
    pSenser_Data_filtered->BNOay = get_iir_output( pLevel1_Filter_ay, pSenser_Data->BNOay );
    pSenser_Data_filtered->BNOaz = get_iir_output( pLevel1_Filter_az, pSenser_Data->BNOaz );
}

void BNOdata_filter_Gyro()
{
    cal_iir_coeff( pLevel1_Filter_gx, 100, 30 ); //(滤波参数，采样频率，滤波频率)
    cal_iir_coeff( pLevel1_Filter_gy, 100, 30 );
    cal_iir_coeff( pLevel1_Filter_gz, 100, 30 );
    pSenser_Data_filtered->BNOgx = get_iir_output( pLevel1_Filter_gx, pSenser_Data->BNOgx );
    pSenser_Data_filtered->BNOgy = get_iir_output( pLevel1_Filter_gy, pSenser_Data->BNOgy );
    pSenser_Data_filtered->BNOgz = get_iir_output( pLevel1_Filter_gz, pSenser_Data->BNOgz );
}
//
//  /*---------------------------------  ADI数据滤波    ------------------------*/
//  void IMUdata_filter_ADIAcc()
//{
//
//  cal_iir_coeff(pLevel1_Filter_Ax, 400, 20);//(滤波参数，采样频率，滤波频率)
//  cal_iir_coeff(pLevel1_Filter_Ay, 400, 20);
//  cal_iir_coeff(pLevel1_Filter_Az, 400, 20);
//
//  pSenser_Data_filtered->ADIax = get_iir_output(pLevel1_Filter_Ax, pSenser_Data->ADIax);
//  pSenser_Data_filtered->ADIay = get_iir_output(pLevel1_Filter_Ay, pSenser_Data->ADIay);
//  pSenser_Data_filtered->ADIaz = get_iir_output(pLevel1_Filter_Az, pSenser_Data->ADIaz);
//}
//
/*---------------------------------  滤波数据选择   ------------------------*/
void Senser_data_process()
{
    if( pSenser_Data->Senser_Mode != 1 )
    {
        pSenser_Data->RollRate    = pSenser_Data_filtered->BNOgy;
        pSenser_Data->PitchRate   = pSenser_Data_filtered->BNOgx;
        pSenser_Data->YawRate     = pSenser_Data_filtered->BNOgz;
        pSenser_Data->Xacc        = pSenser_Data_filtered->BNOax;
        pSenser_Data->Yacc        = pSenser_Data_filtered->BNOay;
        pSenser_Data->Zacc        = pSenser_Data_filtered->BNOaz;
    }
    else
    {
        pSenser_Data->RollRate    = pSenser_Data_filtered->ICMGY;
        pSenser_Data->PitchRate   = pSenser_Data_filtered->ICMGX;
        pSenser_Data->YawRate     = pSenser_Data_filtered->ICMGZ;
        pSenser_Data->Xacc        = pSenser_Data_filtered->ICMAX;
        pSenser_Data->Yacc        = pSenser_Data_filtered->ICMAY;
        pSenser_Data->Zacc        = pSenser_Data_filtered->ICMAZ;
    }
    pSenser_Data->magY        = pSenser_Data->BNOmx;
    pSenser_Data->magX        = pSenser_Data->BNOmy;
    pSenser_Data->magZ        = pSenser_Data->BNOmz;
}


/*-----------------------------   ButterWorth滤波  ---------------------------*/

/*--------------------------   100hz采样  30hz截止频率  ----------------------*/
//float Acc_b1 = 0.3913;
//float Acc_b2 = 0.7827;
//float Acc_b3 = 0.3913;
//float Acc_a2 = 0.3695;
//float Acc_a3 = 0.1958;
///*--------------------------   100hz采样  20hz截止频率  ----------------------*/
////float Acc_b1 = 0.2066;
////float Acc_b2 = 0.4131;
////float Acc_b3 = 0.2066;
////float Acc_a2 = -0.3695;
////float Acc_a3 = 0.1958;
//
//float Gyro_b1 = 0.3913;
//float Gyro_b2 = 0.7827;
//float Gyro_b3 = 0.3913;
//float Gyro_a2 = 0.3695;
//float Gyro_a3 = 0.1958;
/*--------------------------   100hz采样  16hz截止频率  ----------------------*/
float Acc_b1 = 0.1453;
float Acc_b2 = 0.2906;
float Acc_b3 = 0.1453;
float Acc_a2 = -0.6710;
float Acc_a3 = 0.2523;

//float Gyro_b1 = 0.1453;//16hz
//float Gyro_b2 = 0.2906;
//float Gyro_b3 = 0.1453;
//float Gyro_a2 = -0.6710;
//float Gyro_a3 = 0.2523;

float Gyro_b1 = 0.2066;//20hz
float Gyro_b2 = 0.4131;
float Gyro_b3 = 0.2066;
float Gyro_a2 = -0.3695;
float Gyro_a3 = 0.1958;


d_Senser_Data  Senser_Data_old;
d_Senser_Data*  pSenser_Data_old = &Senser_Data_old;

d_Senser_Data  Senser_Data_pre_old;
d_Senser_Data* pSenser_Data_pre_old = &Senser_Data_pre_old;

d_Senser_Data  Senser_Data_filtered;
d_Senser_Data*  pSenser_Data_filtered = &Senser_Data_filtered;

d_Senser_Data  Senser_Data_filtered_old;
d_Senser_Data*  pSenser_Data_filtered_old = &Senser_Data_filtered_old;

d_Senser_Data  Senser_Data_filtered_pre_old;
d_Senser_Data*  pSenser_Data_filtered_pre_old = &Senser_Data_filtered_pre_old;


void butterworth_filter()
{
    pSenser_Data_filtered->RollRate   = ( Gyro_b1 * pSenser_Data->BNOgy + Gyro_b2 * pSenser_Data_old->RollRate  + Gyro_b3 * pSenser_Data_pre_old->RollRate  - Gyro_a2 * pSenser_Data_filtered_old->RollRate  - Gyro_a3 * pSenser_Data_filtered_pre_old->RollRate );
    pSenser_Data_filtered->PitchRate  = ( Gyro_b1 * pSenser_Data->BNOgx + Gyro_b2 * pSenser_Data_old->PitchRate + Gyro_b3 * pSenser_Data_pre_old->PitchRate - Gyro_a2 * pSenser_Data_filtered_old->PitchRate - Gyro_a3 * pSenser_Data_filtered_pre_old->PitchRate );
    pSenser_Data_filtered->YawRate    = ( Gyro_b1 * pSenser_Data->BNOgz + Gyro_b2 * pSenser_Data_old->YawRate   + Gyro_b3 * pSenser_Data_pre_old->YawRate   - Gyro_a2 * pSenser_Data_filtered_old->YawRate   - Gyro_a3 * pSenser_Data_filtered_pre_old->YawRate );
    pSenser_Data_pre_old->RollRate  = pSenser_Data_old->RollRate;
    pSenser_Data_old->RollRate      = pSenser_Data->BNOgy;
    pSenser_Data_pre_old->PitchRate = pSenser_Data_old->PitchRate;
    pSenser_Data_pre_old->PitchRate = pSenser_Data->BNOgx;
    pSenser_Data_pre_old->YawRate   = pSenser_Data_old->YawRate;
    pSenser_Data_old->YawRate       = pSenser_Data->BNOgz;
    pSenser_Data_filtered_pre_old->RollRate  = pSenser_Data_filtered_old->RollRate;
    pSenser_Data_filtered_old->RollRate      = pSenser_Data_filtered->RollRate;
    pSenser_Data_filtered_pre_old->PitchRate = pSenser_Data_filtered_old->PitchRate;
    pSenser_Data_filtered_old->PitchRate     = pSenser_Data_filtered->PitchRate;
    pSenser_Data_filtered_old->YawRate       = pSenser_Data_filtered_old->YawRate;
    pSenser_Data_filtered_old->YawRate       = pSenser_Data_filtered->YawRate;
    pSenser_Data_filtered->BNOXacc   = ( Acc_b1 * pSenser_Data->BNOax + Acc_b2 * pSenser_Data_old->BNOXacc + Acc_b3 * pSenser_Data_pre_old->BNOXacc - Acc_a2 * pSenser_Data_filtered_old->BNOXacc - Acc_a3 * pSenser_Data_filtered_pre_old->BNOXacc );
    pSenser_Data_filtered->BNOYacc   = ( Acc_b1 * pSenser_Data->BNOay + Acc_b2 * pSenser_Data_old->BNOYacc + Acc_b3 * pSenser_Data_pre_old->BNOYacc - Acc_a2 * pSenser_Data_filtered_old->BNOYacc - Acc_a3 * pSenser_Data_filtered_pre_old->BNOYacc );
    pSenser_Data_filtered->BNOZacc   = ( Acc_b1 * pSenser_Data->BNOaz + Acc_b2 * pSenser_Data_old->BNOZacc + Acc_b3 * pSenser_Data_pre_old->BNOZacc - Acc_a2 * pSenser_Data_filtered_old->BNOZacc - Acc_a3 * pSenser_Data_filtered_pre_old->BNOZacc );
    pSenser_Data_pre_old->BNOXacc  = pSenser_Data_old->BNOXacc;
    pSenser_Data_old->BNOXacc      = pSenser_Data->BNOax;
    pSenser_Data_pre_old->BNOYacc  = pSenser_Data_old->BNOYacc;
    pSenser_Data_old->BNOYacc      = pSenser_Data->BNOay;
    pSenser_Data_pre_old->BNOZacc  = pSenser_Data_old->BNOZacc;
    pSenser_Data_old->BNOZacc      = pSenser_Data->BNOaz;
    pSenser_Data_filtered_pre_old->BNOXacc = pSenser_Data_filtered_old->BNOXacc;
    pSenser_Data_filtered_old->BNOXacc     = pSenser_Data_filtered->BNOXacc;
    pSenser_Data_filtered_pre_old->BNOYacc = pSenser_Data_filtered_old->BNOYacc;
    pSenser_Data_filtered_old->BNOYacc     = pSenser_Data_filtered->BNOYacc;
    pSenser_Data_filtered_pre_old->BNOZacc = pSenser_Data_filtered_old->BNOZacc;
    pSenser_Data_filtered_old->BNOZacc     = pSenser_Data_filtered->BNOZacc;
}


/*---------------------------------  Zacc二次滤波    - -----------------------*/
float Acc2_b1 = 0.0461;
float Acc2_b2 = 0.0923;
float Acc2_b3 = 0.0461;
float Acc2_a2 = -1.3073;
float Acc2_a3 = 0.4918;



extern float zacc_temp;
float Vel = 0;
int fil_cnt = 0;
float acc_map[2] = {0};
float height_a = 0, height_v = 0;
float vel_map[2] = {0};
float vel_old = 0;
uint8_t dis = 0;
float Vel_Acc = 0;
//static uint16_t fft_i = 0 ;

void Senserdata_filter_H()
{
    //   height_a =  fabs(zacc_temp) + pSenser_Data_filtered->Zaccf;//当前时刻 瞬时加速度
    pSenser_Data_filtered->Zaccf   = ( Acc2_b1 * pSenser_Data->Zacc + Acc2_b2 * pSenser_Data_old->Zaccf + Acc2_b3 * pSenser_Data_pre_old->Zaccf - Acc2_a2 * pSenser_Data_filtered_old->Zaccf - Acc2_a3 * pSenser_Data_filtered_pre_old->Zaccf );
    pSenser_Data_pre_old->Zaccf  = pSenser_Data_old->Zaccf;
    pSenser_Data_old->Zaccf      = pSenser_Data->Zacc;
    pSenser_Data_filtered_pre_old->Zaccf = pSenser_Data_filtered_old->Zaccf;
    pSenser_Data_filtered_old->Zaccf     = pSenser_Data_filtered->Zaccf;
    //   if(((height_a > 0.3)||(height_a < -0.3))&&(fil_cnt>10))
    //   {
    //     acc_map[0] = fabs(zacc_temp) + pSenser_Data_filtered->Zaccf1;
    //     acc_map[1] = fabs(zacc_temp) + pSenser_Data_filtered->Zaccf;
    //     Vel_Acc += height_a;
    //   }
    //   else{
    //     Vel_Acc =0;
    //   }
    //   pSenser_Data_filtered->Zaccf1 = pSenser_Data_filtered->Zaccf;
    //   fil_cnt++;
}


/*---------------------------------  原始数据去跳变  -------------------------*/

//float senser_data_map[9][3]={0};
//void Threshold_filter()
//{
//  static uint8_t time_i = 0;
//  static uint8_t flag =0;
//
//  switch(time_i)
//  {
//    case 0:
//      if(flag ==1)
//      {
//        senser_data_map[0][0] = senser_data_map[0][1];
//        senser_data_map[1][0] = senser_data_map[1][1];
//        senser_data_map[2][0] = senser_data_map[2][1];
//
//        senser_data_map[3][0] = senser_data_map[3][1];
//        senser_data_map[4][0] = senser_data_map[4][1];
//        senser_data_map[5][0] = senser_data_map[5][1];
//
//        senser_data_map[6][0] = senser_data_map[6][1];
//        senser_data_map[7][0] = senser_data_map[7][1];
//        senser_data_map[8][0] = senser_data_map[8][1];
//
//        senser_data_map[0][1] = senser_data_map[0][2];
//        senser_data_map[1][1] = senser_data_map[1][2];
//        senser_data_map[2][1] = senser_data_map[2][2];
//
//        senser_data_map[3][1] = senser_data_map[3][2];
//        senser_data_map[4][1] = senser_data_map[4][2];
//        senser_data_map[5][1] = senser_data_map[5][2];
//
//        senser_data_map[6][1] = senser_data_map[6][2];
//        senser_data_map[7][1] = senser_data_map[7][2];
//        senser_data_map[8][1] = senser_data_map[8][2];
//
//        senser_data_map[0][2] = pSenser_Data->BNOgx;
//        senser_data_map[1][2] = pSenser_Data->BNOgy;
//        senser_data_map[2][2] = pSenser_Data->BNOgz;
//
//        senser_data_map[3][2] = pSenser_Data->BNOax;
//        senser_data_map[4][2] = pSenser_Data->BNOay;
//        senser_data_map[5][2] = pSenser_Data->BNOaz;
//
//        senser_data_map[6][2] = pSenser_Data->BNOmx;
//        senser_data_map[7][2] = pSenser_Data->BNOmy;
//        senser_data_map[8][2] = pSenser_Data->BNOmz;
//
//        data_filling();
//
//      }else{
//        senser_data_map[0][0] = pSenser_Data->BNOgx;
//        senser_data_map[1][0] = pSenser_Data->BNOgy;
//        senser_data_map[2][0] = pSenser_Data->BNOgz;
//
//        senser_data_map[3][0] = pSenser_Data->BNOax;
//        senser_data_map[4][0] = pSenser_Data->BNOay;
//        senser_data_map[5][0] = pSenser_Data->BNOaz;
//
//        senser_data_map[6][0] = pSenser_Data->BNOmx;
//        senser_data_map[7][0] = pSenser_Data->BNOmy;
//        senser_data_map[8][0] = pSenser_Data->BNOmz;
//
//         time_i++;
//      }
//
//      break;
//  case 1:
//
//     senser_data_map[0][1] = pSenser_Data->BNOgx;
//     senser_data_map[1][1] = pSenser_Data->BNOgy;
//     senser_data_map[2][1] = pSenser_Data->BNOgz;
//
//     senser_data_map[3][1] = pSenser_Data->BNOax;
//     senser_data_map[4][1] = pSenser_Data->BNOay;
//     senser_data_map[5][1] = pSenser_Data->BNOaz;
//
//     senser_data_map[6][1] = pSenser_Data->BNOmx;
//     senser_data_map[7][1] = pSenser_Data->BNOmy;
//     senser_data_map[8][1] = pSenser_Data->BNOmz;
//
//     time_i++;
//  case 2:
//
//    senser_data_map[0][2] = pSenser_Data->BNOgx;
//    senser_data_map[1][2] = pSenser_Data->BNOgy;
//    senser_data_map[2][2] = pSenser_Data->BNOgz;
//
//    senser_data_map[3][2] = pSenser_Data->BNOax;
//    senser_data_map[4][2] = pSenser_Data->BNOay;
//    senser_data_map[5][2] = pSenser_Data->BNOaz;
//
//    senser_data_map[6][2] = pSenser_Data->BNOmx;
//    senser_data_map[7][2] = pSenser_Data->BNOmy;
//    senser_data_map[8][2] = pSenser_Data->BNOmz;
//
//     data_filling();
//     time_i = 0;
//
//     break;
//       default:
//         break;
//  }
//
//  pSenser_Data->BNOgx1 = senser_data_map[0][1];
//    pSenser_Data->BNOgy1 = senser_data_map[1][1];
//      pSenser_Data->BNOgz1 = senser_data_map[2][1];
//
//        pSenser_Data->BNOax1 = senser_data_map[3][1];
//          pSenser_Data->BNOay1 = senser_data_map[4][1];
//            pSenser_Data->BNOaz1 = senser_data_map[5][1];
//
//              pSenser_Data->BNOmx1 = senser_data_map[6][1];
//                pSenser_Data->BNOmy1 = senser_data_map[7][1];
//                  pSenser_Data->BNOmz1 = senser_data_map[8][1];
//
//                  flag=1;
//
//}
//
//extern float zacc_temp ;
//void data_filling()
//{
//  /*---------------------------------  gyro  -------------------------*/
//  if(((((senser_data_map[0][1] - senser_data_map[0][0] >  0.2)&&(senser_data_map[0][2] - senser_data_map[0][1] < -0.2))||
//  ((senser_data_map[0][1] - senser_data_map[0][0] < -0.2)&&(senser_data_map[0][2] - senser_data_map[0][1] > 0.2)))
//      && (fabs(pSenser_Data->Xacc) <1))||(fabs(senser_data_map[0][1]) > 2.0))
//  {
//    senser_data_map[0][1] = senser_data_map[0][0];
//  }
//  if(((((senser_data_map[1][1] - senser_data_map[1][0] >  0.2)&&(senser_data_map[1][2] - senser_data_map[1][1] < -0.2))||
//  ((senser_data_map[1][1] - senser_data_map[1][0] < -0.2)&&(senser_data_map[1][2] - senser_data_map[1][1] > 0.2)))
//      && (fabs(pSenser_Data->Yacc) <1))||(fabs(senser_data_map[1][1]) > 2.0))
//  {
//    senser_data_map[1][1] = senser_data_map[1][0];
//  }
//  if(((fabs(senser_data_map[2][0] - senser_data_map[2][1])) > 0.2)||(fabs(pSenser_Data->BNOgz) > 2.0))
//  {
//    senser_data_map[2][1] = senser_data_map[2][0];
//  }
//
//  /*---------------------------------  acc  -------------------------*/
//if((((senser_data_map[3][1] - senser_data_map[3][0] >  9)&&(senser_data_map[3][2] - senser_data_map[3][1] < -9))||
//  ((senser_data_map[3][1] - senser_data_map[3][0] < -9)&&(senser_data_map[3][2] - senser_data_map[3][1] > 9))) ||
//   ((fabs(senser_data_map[3][1])+fabs(zacc_temp)) > 39))
//  {
//    senser_data_map[3][1] = senser_data_map[3][0];
//  }
//if((((senser_data_map[4][1] - senser_data_map[4][0] >  9)&&(senser_data_map[4][2] - senser_data_map[4][1] < -9))||
//  ((senser_data_map[4][1] - senser_data_map[4][0] < -9)&&(senser_data_map[4][2] - senser_data_map[4][1] > 9))) ||
//   ((fabs(senser_data_map[4][1])+fabs(zacc_temp)) > 39))
//  {
//    senser_data_map[4][1] = senser_data_map[4][0];
//  }
////  if(((fabs((senser_data_map[5][0]+fabs(zacc_temp)) - (senser_data_map[5][1]+fabs(zacc_temp)))) > 9)||((fabs(senser_data_map[5][1])+fabs(zacc_temp)) > 39))
//if((((senser_data_map[5][1] - senser_data_map[5][0] >  9)&&(senser_data_map[5][2] - senser_data_map[5][1] < -9))||
//  ((senser_data_map[5][1] - senser_data_map[5][0] < -9)&&(senser_data_map[5][2] - senser_data_map[5][1] > 9))) ||
//   ((fabs(senser_data_map[5][1])+fabs(zacc_temp)) > 39))
//  {
//    senser_data_map[5][1] = senser_data_map[5][0];
//  }
//
//  /*---------------------------------  mag  -------------------------*/
//  if((fabs(senser_data_map[6][0] - senser_data_map[6][1])) > 1)
//  {
//    senser_data_map[6][1] = senser_data_map[6][0];
//  }
//  if((fabs(senser_data_map[7][0] - senser_data_map[7][1])) > 1)
//  {
//    senser_data_map[7][1] = senser_data_map[7][0];
//  }
//  if((fabs(senser_data_map[8][0] - senser_data_map[8][1])) > 1)
//  {
//    senser_data_map[8][1] = senser_data_map[8][0];
//  }
//}

//float senser_data_map[9][2]={0};
//void Threshold_filter()
//{
//  static uint8_t time_i = 0;
//  static uint8_t flag =0;
//
//  switch(time_i)
//  {
//    case 0:
//      if(flag ==1)
//      {
//        senser_data_map[0][0] = senser_data_map[0][1];
//        senser_data_map[1][0] = senser_data_map[1][1];
//        senser_data_map[2][0] = senser_data_map[2][1];
//
//        senser_data_map[3][0] = senser_data_map[3][1];
//        senser_data_map[4][0] = senser_data_map[4][1];
//        senser_data_map[5][0] = senser_data_map[5][1];
//
//        senser_data_map[6][0] = senser_data_map[6][1];
//        senser_data_map[7][0] = senser_data_map[7][1];
//        senser_data_map[8][0] = senser_data_map[8][1];
//      /*---------------------------------  ICM20602  -------------------------*/
//        if(pSenser_Data->ICM_flag == 1){
//          senser_data_map[0][1] = pSenser_Data->ICMGX;
//          senser_data_map[1][1] = pSenser_Data->ICMGY;
//          senser_data_map[2][1] = pSenser_Data->ICMGZ;
//
//          senser_data_map[3][1] = pSenser_Data->ICMAX;
//          senser_data_map[4][1] = pSenser_Data->ICMAY;
//          senser_data_map[5][1] = pSenser_Data->ICMAZ;
//        }else
//      /*---------------------------------  BNO055    -------------------------*/
//        {
//          senser_data_map[0][1] = pSenser_Data->BNOgx;
//          senser_data_map[1][1] = pSenser_Data->BNOgy;
//          senser_data_map[2][1] = pSenser_Data->BNOgz;
//
//          senser_data_map[3][1] = pSenser_Data->BNOax;
//          senser_data_map[4][1] = pSenser_Data->BNOay;
//          senser_data_map[5][1] = pSenser_Data->BNOaz;
//        }
//        senser_data_map[6][1] = pSenser_Data->BNOmx;
//        senser_data_map[7][1] = pSenser_Data->BNOmy;
//        senser_data_map[8][1] = pSenser_Data->BNOmz;
//
//        data_filling();
//
//        time_i =0;
//      }else{
//      /*---------------------------------  ICM20602  -------------------------*/
//        if(pSenser_Data->ICM_flag == 1){
//          senser_data_map[0][0] = pSenser_Data->ICMGX;
//          senser_data_map[1][0] = pSenser_Data->ICMGY;
//          senser_data_map[2][0] = pSenser_Data->ICMGZ;
//
//          senser_data_map[3][0] = pSenser_Data->ICMAX;
//          senser_data_map[4][0] = pSenser_Data->ICMAY;
//          senser_data_map[5][0] = pSenser_Data->ICMAZ;
//        }else{
//      /*---------------------------------  BNO055    -------------------------*/
//          senser_data_map[0][0] = pSenser_Data->BNOgx;
//          senser_data_map[1][0] = pSenser_Data->BNOgy;
//          senser_data_map[2][0] = pSenser_Data->BNOgz;
//
//          senser_data_map[3][0] = pSenser_Data->BNOax;
//          senser_data_map[4][0] = pSenser_Data->BNOay;
//          senser_data_map[5][0] = pSenser_Data->BNOaz;
//        }
//        senser_data_map[6][0] = pSenser_Data->BNOmx;
//        senser_data_map[7][0] = pSenser_Data->BNOmy;
//        senser_data_map[8][0] = pSenser_Data->BNOmz;
//
//         time_i++;
//      }
//
//      break;
//  case 1:
//      /*---------------------------------  ICM20602  -------------------------*/
//    if(pSenser_Data->ICM_flag == 1){
//      senser_data_map[0][1] = pSenser_Data->ICMGX;
//      senser_data_map[1][1] = pSenser_Data->ICMGY;
//      senser_data_map[2][1] = pSenser_Data->ICMGZ;
//
//      senser_data_map[3][1] = pSenser_Data->ICMAX;
//      senser_data_map[4][1] = pSenser_Data->ICMAY;
//      senser_data_map[5][1] = pSenser_Data->ICMAZ;
//    }else{
//      /*---------------------------------  BNO055    -------------------------*/
//     senser_data_map[0][1] = pSenser_Data->BNOgx;
//     senser_data_map[1][1] = pSenser_Data->BNOgy;
//     senser_data_map[2][1] = pSenser_Data->BNOgz;
//
//     senser_data_map[3][1] = pSenser_Data->BNOax;
//     senser_data_map[4][1] = pSenser_Data->BNOay;
//     senser_data_map[5][1] = pSenser_Data->BNOaz;
//    }
//     senser_data_map[6][1] = pSenser_Data->BNOmx;
//     senser_data_map[7][1] = pSenser_Data->BNOmy;
//     senser_data_map[8][1] = pSenser_Data->BNOmz;
//
//     data_filling();
//     time_i--;
//     break;
//       default:
//         break;
//  }
//
//  pSenser_Data->BNOgx1 = senser_data_map[0][1];
//    pSenser_Data->BNOgy1 = senser_data_map[1][1];
//      pSenser_Data->BNOgz1 = senser_data_map[2][1];
//
//        pSenser_Data->BNOax1 = senser_data_map[3][1];
//          pSenser_Data->BNOay1 = senser_data_map[4][1];
//            pSenser_Data->BNOaz1 = senser_data_map[5][1];
//
//              pSenser_Data->BNOmx1 = senser_data_map[6][1];
//                pSenser_Data->BNOmy1 = senser_data_map[7][1];
//                  pSenser_Data->BNOmz1 = senser_data_map[8][1];
//
//                  flag=1;
//
//}
//
//extern float zacc_temp ;
//float err_count9 = 0;
//void data_filling()
//{
//  /*---------------------------------  gyro  -------------------------*/
//  if(((fabs(senser_data_map[0][0] - senser_data_map[0][1])) > 2)||(fabs(senser_data_map[0][0]) > 2.0))
//  {
////    senser_data_map[0][1] = senser_data_map[0][0];
//  }
//  if(((fabs(senser_data_map[1][0] - senser_data_map[1][1])) > 2)||(fabs(senser_data_map[1][0]) > 2.0))
//  {
////    senser_data_map[1][1] = senser_data_map[1][0];
//  }
//  if(((fabs(senser_data_map[2][0] - senser_data_map[2][1])) > 2)||(fabs(senser_data_map[2][0]) > 2.0))
//  {
////    senser_data_map[2][1] = senser_data_map[2][0];
//    err_count9++;
//  }
//
//  /*---------------------------------  acc  -------------------------*/
//  if((fabs(senser_data_map[3][0] - senser_data_map[3][1])) > 8)
//  {
////    senser_data_map[3][1] = senser_data_map[3][0];
//  }
//  if((fabs(senser_data_map[4][0] - senser_data_map[4][1])) > 8)
//  {
////    senser_data_map[4][1] = senser_data_map[4][0];
//  }
//  if(((fabs((senser_data_map[5][0]+fabs(zacc_temp)) - (senser_data_map[5][1]+fabs(zacc_temp)))) > 9)||((fabs(senser_data_map[5][1])+fabs(zacc_temp)) > 39))
//  {
////    senser_data_map[5][1] = senser_data_map[5][0];
//  }
//
//  /*---------------------------------  mag  -------------------------*/
//  if((fabs(senser_data_map[6][0] - senser_data_map[6][1])) > 1)
//  {
////    senser_data_map[6][1] = senser_data_map[6][0];
//  }
//  if((fabs(senser_data_map[7][0] - senser_data_map[7][1])) > 1)
//  {
////    senser_data_map[7][1] = senser_data_map[7][0];
//  }
//  if((fabs(senser_data_map[8][0] - senser_data_map[8][1])) > 1)
//  {
////    senser_data_map[8][1] = senser_data_map[8][0];
//  }
//}





//void FFT_AXToVel()
//{
//
//  memset(vax,0,FFT_LENGTH*2*4);
//  arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);
//
//  arm_cfft_radix4_f32(&scfft,fftx_inputbuf);
//
//
////  GPIO_ResetBits(GPIOC,GPIO_Pin_6);
//  for(uint16_t j = 1;j<FFT_LENGTH;j++)
//  {
//    w[j] = w1[j] ;//  w=w.^1;//w的元素球平方
//
//    fftx_inputbuf[2*j]   =  fftx_inputbuf[2*j] / w[j] ;
//    fftx_inputbuf[2*j+1] =  fftx_inputbuf[2*j+1] / w[j] ;//aa = y(2:nfft-1)./w(2:nfft-1)
//
//    fftx_inputbuf[2*j]   = fftx_inputbuf[2*j] - fftx_inputbuf[2*j+1]*1024;
//    fftx_inputbuf[2*j+1] = 0;
//  }
////    GPIO_SetBits(GPIOC,GPIO_Pin_6);
//
//  memcpy(&vax[2*(ni-1)],&fftx_inputbuf[2*(ni-1)],4*2*(na-ni+1));
//
//  memcpy(&vax[2 * (FFT_LENGTH - na -1)],&fftx_inputbuf[2 * (FFT_LENGTH - na -1)],4*2*(na-ni+2));
//
//  arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,1,1);
//
//  arm_cfft_radix4_f32(&scfft,vax);
//
//  memset(fftx_inputbuf,0,FFT_LENGTH*2*4);
//
//
//}
//
//void FFT_AYToVel()
//{
//
//  memset(vay,0,FFT_LENGTH*2*4);
//  arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);
//
//  arm_cfft_radix4_f32(&scfft,ffty_inputbuf);
//
//
////  GPIO_ResetBits(GPIOC,GPIO_Pin_6);
//  for(uint16_t j = 1;j<FFT_LENGTH;j++)
//  {
//    w[j] = w1[j] ;//  w=w.^1;//w的元素球平方
//
//    ffty_inputbuf[2*j]   =  ffty_inputbuf[2*j] / w[j] ;
//    ffty_inputbuf[2*j+1] =  ffty_inputbuf[2*j+1] / w[j] ;//aa = y(2:nfft-1)./w(2:nfft-1)
//
//    ffty_inputbuf[2*j]   = ffty_inputbuf[2*j] - ffty_inputbuf[2*j+1]*1024;
//    ffty_inputbuf[2*j+1] = 0;
//  }
////    GPIO_SetBits(GPIOC,GPIO_Pin_6);
//
//  memcpy(&vay[2*(ni-1)],&ffty_inputbuf[2*(ni-1)],4*2*(na-ni+1));
//
//  memcpy(&vay[2 * (FFT_LENGTH - na -1)],&ffty_inputbuf[2 * (FFT_LENGTH - na -1)],4*2*(na-ni+2));
//
//  arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,1,1);
//
//  arm_cfft_radix4_f32(&scfft,vay);
//
//  memset(ffty_inputbuf,0,FFT_LENGTH*2*4);
//
//
//}