/*
  ******************************************************************************
  * @file    filter.c
  * @author  CF
  * @version v1.0
  * @date    2017-09-20
  * @brief   二阶IIR低通滤波器
  *          使用时，需分别定义IIR_coeff_Typedef结构体，保存参数
 ******************************************************************************
*/

#include "Library_Filter.h"

#define M_PI_F (3.1415926)
// LowPassFilter2p
/*
  * @brief  计算IIR滤波器参数
  * @param  返回的IIR_coeff_Typedef结构体，采样频率，带宽
  * @note
  * @retval void
  * @author CF
*/
void cal_iir_coeff( IIR_coeff_Typedef* coeff, float fs, float fc )
{
    float fr = 0;
    float ohm = 0;
    float c = 0;
    fr = fs / fc;
    ohm = tanf( M_PI_F / fr );
    c = 1.0f + 2.0f * cosf( M_PI_F / 4.0f ) * ohm + ohm * ohm;
    coeff->fc = fc;
    coeff->b0 = ohm * ohm / c;
    coeff->b1 = 2.0f * coeff->b0;
    coeff->b2 = coeff->b0;
    coeff->a1 = 2.0f * ( ohm * ohm - 1.0f ) / c;
    coeff->a2 = ( 1.0f - 2.0f * cosf( M_PI_F / 4.0f ) * ohm + ohm * ohm ) / c;
}

/*
  * @brief  滤波函数
  * @param  IIR_coeff_Typedef结构体，待滤波的原始数据
  * @note
  * @retval 滤波后的数据
  * @author huzhen
*/
float get_iir_output( IIR_coeff_Typedef* coeff, float sample )
{
    if( coeff->fc <= 0.0f )
    {
        return sample;
    }
    else
    {
        float y_0 = sample - coeff->y_1 * coeff->a1 - coeff->y_2 * coeff->a2;
        float output = y_0 * coeff->b0 + coeff->y_1 * coeff->b1 + coeff->y_2 * coeff->b2;
        coeff->y_2 = coeff->y_1;
        coeff->y_1 = y_0;
        return output;
    }
}