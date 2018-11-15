#include "Calculate_Wave.h"
#include "math.h"
#include "arm_math.h"
#include "filter.h"
#include "stdlib.h"
#include "Communication.h"

arm_cfft_radix4_instance_f32 scfft;
//arm_rfft_fast_instance_f32 scfft;

extern float zacc_temp ;
extern   float a[FFT_LENGTH * 2];
uint8_t acc_cycle_i = 0;
float fft_inputbuf[FFT_LENGTH * 2];
int fft_i = 0 ;

/*-----------------------  �������µ����Ƽ��㷽��   --------------------------*/
//float attitude_roll[256];
//float attitude_pitch[256];

/*-----------------------���ڼ��㲨�˵�������Ϣ-------------------------------*/
typedef struct
{
    uint16_t cj;//��ֵ
    uint8_t sub;//�±�
} PAIR; //qsort�ĸ����ȽϺ���

int compare( const void* p, const void* q )
{
    PAIR t1 = *( PAIR* )p;
    PAIR t2 = *( PAIR* )q;
    return ( int )( t2.cj * 10 - t1.cj * 10 ); //�Ӵ���С
}

PAIR Wave_Amplitude[100] = {0}; //����ʹ��30����  ʵ��ʹ��100
float Wave_Cycle[100] = {0};

/*---------------------------------  fft���ݵ��� --------------------------*/
void Get_Acc_Cycle()
{
    if( ( fft_inputbuf[2 * fft_i] < 0 ) && ( fft_inputbuf[2 * ( fft_i - 1 )] > 0 ) )
    {
        acc_cycle_i++;
    }
}

void Wave_Height_Calculate()
{
    pSenser_Data->State = pSenser_Data->State | 0x40;//���ڲɼ�
    if( fft_i < 1024 )
    {
        //        fft_inputbuf[2*fft_i] = pSenser_Data_filtered->Zaccf/9.7949+1;
        //        if(fft_i <512){
        fft_inputbuf[2 * fft_i] = 0.02 * sinf( 2 * 3.1415926 * fft_i / 125 );
        //        }else
        //        {
        //          fft_inputbuf[2*fft_i] = 0.03*sinf(2*3.1415926*fft_i/71);
        //        }
        fft_inputbuf[2 * fft_i + 1] = 0;
        /*--------- ͨ�����ٶȻ�ȡ���ڣ�����FFT��С��ֹƵ�ʲ���---------------------*/
        Get_Acc_Cycle();
        pWave_Data->Height = a[fft_i * 2] * 1000 * 9;
        Get_Cycle_Height();
        /*-----------------------  �������µ����Ƽ��㷽��   ------------------*/
        //        attitude_roll[fft_i]  = pSenser_Data->Roll;
        //        attitude_pitch[fft_i] = pSenser_Data->Pitch;
        Yaw_Combine();
        fft_i++;
    }
}

/*---------------------------------  FFT�㷨   -------------------------------*/

//  uint8_t c = 9.8;

//  float w1[FFT_LENGTH] = {0};
//  float w2[FFT_LENGTH/2] = {0};
float w[FFT_LENGTH] = {0};
float a[FFT_LENGTH * 2] = {0};
//    float vax[FFT_LENGTH*2] = {0};
//    float vay[FFT_LENGTH*2] = {0};

uint8_t ni_fft = 0;
uint8_t na_fft = 0;
float dw = 0;
//  float fft = 0;
//  float nfft =0;

void FFT_init()
{
    uint16_t time_i = 0;
    float sf = 5;
    float filmin = 0.029;
    float filmax = 0.45;
    double df = 0;
    df = sf / FFT_LENGTH; //0.0977
    ni_fft = ( uint8_t )round( filmin / df + 1 ); //4
    na_fft = ( uint8_t )round( filmax / df + 1 ); //165
    dw = 2 * pi * df; //0.6136
    //    nfft = 2*pi*(0.5*sf-df);//313.5457
    //    for(uint16_t time_i =0;time_i < FFT_LENGTH/2;time_i++)
    //    {
    //      w1[time_i] = dw * time_i;
    //      w2[time_i] = nfft - dw * time_i;
    //    }
    //    memcpy(&w1[FFT_LENGTH/2],w2,FFT_LENGTH/2*4);//w=[w1,w2];
    for( time_i = 0; time_i < FFT_LENGTH / 2; time_i++ )
    {
        w[time_i] = dw * time_i;
        w[time_i] = pow( w[time_i], 2 );
    }
    for( time_i = FFT_LENGTH / 2; time_i < FFT_LENGTH - 1; time_i++ )
    {
        w[time_i] = -dw * ( FFT_LENGTH / 2 - 1 ) + dw * ( time_i - FFT_LENGTH / 2 );
        w[time_i] = pow( w[time_i], 2 );
    }
}

void FFT_Cal()
{
    //filmin = acc_cycle_i/204.8 - 0.01;
    //filmin = filmin > 0 ? filmin:0.03;
    //  filmin = 0.029;
    //ni_fft=(uint8_t)round(filmin/df+1);
    memset( a, 0, FFT_LENGTH * 2 * 4 );
    arm_cfft_radix4_init_f32( &scfft, FFT_LENGTH, 0, 1 );
    arm_cfft_radix4_f32( &scfft, fft_inputbuf );
    //  arm_rfft_fast_init_f32(&scfft,FFT_LENGTH);
    //  arm_rfft_fast_f32(&scfft,fft_inputbuf,fft_inputbuf,0);
    for( uint16_t j = 1; j < FFT_LENGTH; j++ )
    {
        fft_inputbuf[2 * j]   = - fft_inputbuf[2 * j] / w[j] ;
        fft_inputbuf[2 * j + 1] = - fft_inputbuf[2 * j + 1] / w[j] ; //aa = y(2:nfft-1)./w(2:nfft-1)
    }
    //    GPIO_SetBits(GPIOC,GPIO_Pin_6);
    fft_inputbuf[0] = 0;
    fft_inputbuf[1] = 0;
    fft_inputbuf[510] = 0;
    fft_inputbuf[511] = 0;
    memcpy( &a[2 * ( ni_fft - 1 )], &fft_inputbuf[2 * ( ni_fft - 1 )], 4 * 2 * ( na_fft - ni_fft + 1 ) );
    memcpy( &a[2 * ( FFT_LENGTH - na_fft )], &fft_inputbuf[2 * ( FFT_LENGTH - na_fft )], 4 * 2 * ( na_fft - ni_fft + 1 ) );
    arm_cfft_radix4_init_f32( &scfft, FFT_LENGTH, 1, 1 );
    arm_cfft_radix4_f32( &scfft, a );
    //    arm_rfft_fast_init_f32(&scfft,FFT_LENGTH);
    //  arm_rfft_fast_f32(&scfft,a,a,1);
    memset( fft_inputbuf, 0, FFT_LENGTH * 2 * 4 );
    acc_cycle_i = 0;
}

float current_vel_heading = 0;
uint8_t heading_count[16] = {0};
uint8_t current_heading_i = 0;
static uint8_t first_down_count = 0;
void Yaw_Combine()
{
    pSenser_Data->mYaw  = atan2( -pSenser_Data->magY, -pSenser_Data->magX ) * 57.3;
    //  Angle_fuse_yaw( pSenser_Data->mYaw, pSenser_Data->YawRate, &f_angle_yaw, &f_angle_dot_yaw);
    if( ( pSenser_Data->mYaw < 0 ) && ( pSenser_Data->mYaw > -180 ) ) //360������ת��
    {
        pSenser_Data->mYaw = fabs( fabs( pSenser_Data->mYaw ) - 180 ) + 180;
    }
    else
    {
        pSenser_Data->mYaw = pSenser_Data->mYaw;
    }
    /*----------------------------------  �������  ----------------------------*/
    /*
    ֻ����1/2���ڵ����ƣ����˵Ĵ���͵㵽��ߵ������
    ����ÿ���˵ķ������ս���ͳ��
    */
    if( pWave_Data->Heading_count_flag == 1 )
    {
        /*-------------------  ֻ�ڸ�Ư���µĹ��̼��㣬����1/4����   -----------*/
        /*---------------   �����ж�������ֻ�и�Ư����ǵ�ʱ��ż���     -------*/
        if( ( fabs( pSenser_Data->Roll ) > 3 ) || ( fabs( pSenser_Data->Pitch ) > 3 ) )
        {
            //      current_vel_heading = -atan2(-pSenser_Data->Roll,pSenser_Data->Pitch) * 57.3;//��������
            current_vel_heading += -atan2( -pSenser_Data->Roll, pSenser_Data->Pitch ) * 57.3; //��������
        }
        else
        {
            current_vel_heading = 0 ;
        }
        current_heading_i++;
    }
    else if( ( pWave_Data->Heading_count_flag == 2 ) && ( first_down_count == 1 ) )
    {
        current_vel_heading = current_vel_heading / ( current_heading_i + 1 );
        pWave_Data->Heading_count_flag = 0;
        pSenser_Data->Yaw = pSenser_Data->mYaw + current_vel_heading;
        if( pSenser_Data->Yaw > 360 )
        {
            pSenser_Data->Yaw = pSenser_Data->Yaw - 360;
        }
        else if( pSenser_Data->Yaw < 0 )
        {
            pSenser_Data->Yaw = 360 + pSenser_Data->Yaw;
        }
        else
        {
            pSenser_Data->Yaw = pSenser_Data->Yaw;
        }
        pWave_Data->Heading_count_flag = 0;
        current_heading_i = 0;
        current_vel_heading = 0;
    }
    /*--------------------------------   ���̺ϳ�    ---------------------------*/
    //   current_vel_heading = -atan2(pSenser_Data->Yacc,pSenser_Data->Xacc) * 57.3;//��������
    //  current_vel_heading = -atan2(pSenser_Data->Roll,pSenser_Data->Pitch) * 57.3;//��������
    //  pSenser_Data->Yaw = pSenser_Data->mYaw + current_vel_heading;
    //  if(pSenser_Data->Yaw > 360)
    //  {
    //    pSenser_Data->Yaw = pSenser_Data->Yaw -360;
    //  }else if(pSenser_Data->Yaw < 0)
    //  {
    //  pSenser_Data->Yaw = 360 + pSenser_Data->Yaw;
    //  }else
    //  {
    //  pSenser_Data->Yaw = pSenser_Data->Yaw;
    //  }
    //
    //��������
    if( ( pWave_Data->Tradion_Sample_Start == 1 ) && ( pWave_Data->Tradion_Sample_Enable == 1 ) )
    {
        switch( ( u8 )( ( pSenser_Data->Yaw + 11.3 ) / 22.5 ) )
        {
            case 0:
                heading_count[0]++; //N
                break;
            case 1:
                heading_count[1]++; //NNE
                break;
            case 2:
                heading_count[2]++; //NE
                break;
            case 3:
                heading_count[3]++; //ENE
                break;
            case 4:
                heading_count[4]++; //E
                break;
            case 5:
                heading_count[5]++; //ESE
                break;
            case 6:
                heading_count[6]++; //SE
                break;
            case 7:
                heading_count[7]++; //SES
                break;
            case 8:
                heading_count[8]++; //S
                break;
            case 9:
                heading_count[9]++; //SSW
                break;
            case 10:
                heading_count[10]++; //SW
                break;
            case 11:
                heading_count[11]++; //WSW
                break;
            case 12:
                heading_count[12]++; //W
                break;
            case 13:
                heading_count[13]++; //WNW
                break;
            case 14:
                heading_count[14]++; //NW
                break;
            case 15:
                heading_count[15]++; //NNW
                break;
            default:
                break;
        }
    }
    //  if(fft_i%64 == 0)//3��
    //  {
    //     uint8_t max_heading =0;
    //     uint8_t direction_count =0;
    //     max_heading = heading_count[0];
    //    for(uint8_t i=1;i<16;i++)
    //    {
    //
    //     if( max_heading < heading_count[i])
    //     { direction_count = i;
    //     max_heading = heading_count[i];
    //    }
    //    }
    //    pWave_Data->Main_heading = direction_count * 22.5 -11.3;
    //    if(pWave_Data->Main_heading < 0)
    //    {
    //      pWave_Data->Main_heading += 360;
    //    }
    //     memset(heading_count,0,16);
    //  }
    /*--------------------------------   ���ٶ�ʸ���ϳ�    ---------------------*/
    //  static float NS_Vector_Pat = 0;
    //  static float EW_Vector_Pat = 0;
    //  if(fft_i%64 == 0)//6��
    //  {
    //    current_vel_heading = -atan2(NS_Vector_Pat/64,EW_Vector_Pat/64) * 57.3;//��������
    //    NS_Vector_Pat = 0;
    //    EW_Vector_Pat = 0;
    //  }else
    //  {
    //    NS_Vector_Pat += pSenser_Data->Yacc;
    //    EW_Vector_Pat += pSenser_Data->Xacc;
    //  }
    //
    //
    //    pSenser_Data->Yaw = pSenser_Data->mYaw + current_vel_heading;
    //  if(pSenser_Data->Yaw > 360)
    //  {
    //    pSenser_Data->Yaw = pSenser_Data->Yaw -360;
    //  }else if(pSenser_Data->Yaw < 0)
    //  {
    //  pSenser_Data->Yaw = 360 + pSenser_Data->Yaw;
    //  }else
    //  {
    //  pSenser_Data->Yaw = pSenser_Data->Yaw;
    //  }
    //  pWave_Data->Main_heading = pSenser_Data->Yaw;
}

extern uint8_t Min_WaveHeught;

//void Get_Cycle_Height()
//{
//  static uint8_t len = 0;
//  static float maxheight=0,minheight=0;
//  static uint8_t cycle_i =0;
//
//  /*-------------
//  step 0��Ѱ�ҵ�һ�����µ����㣻setp++��
//  step 1��len++�� Ѱ�����/С �����step++��
//  step 2�����������������������������cycle��height
//  �������㷵��step0��
//  ----------*/
//
//    switch(cycle_i)
//    {
//    case 0:
//      // first_down
//      if(a[fft_i*2]>=0 && a[fft_i*2 +2]<0)
//      {
//        len = 0;
//        cycle_i = 1;
//        maxheight = a[fft_i*2];
//        minheight = a[fft_i*2];
//      }
//      break;
//
//    case 1://input data
//      len ++;
//      if(a[fft_i*2] > 0)
//      {
//        if(maxheight < a[fft_i*2])
//        {
//          maxheight = a[fft_i*2];//max
//
//        }else{
//          cycle_i = 2;
//          /*------------------------   ���ϵĹ��̽���  ---------------------*/
//          first_down_count++;
//            pWave_Data->Heading_count_flag = 2;
//        }
//      }else{
//        if(minheight > a[fft_i*2])
//        {
//          minheight = a[fft_i*2];//min
//
//        }else{
//          /*------------------------   ���ϵĹ���   ----------------------------*/
//          pWave_Data->Heading_count_flag =1;
//        }
//      }
//
//
//      break;
//
//    case 2://cail
//      len++;
//      if(a[fft_i*2-2]>=0 && a[fft_i*2]<0)//second down
//      {
//        if(len < 5)//ignore this  wave
//        {
//          cycle_i = 1;
//          len =0;
//          maxheight = a[fft_i*2];
//          minheight = 0;
//          //        pWave_Data->Absolute_Height = 0;//��������ʾ
//          //        pWave_Data->Cycle = 0;
//
//          /*----------------------   ���̽��� ׼����һ�� ---------------------*/
//          pWave_Data->Heading_count_flag = 0;
//        }else//meet the length
//        {
//
//          if((uint8_t)((maxheight - minheight)*100) < pHost_Cmd->Min_WaveHeught)//ignore the wave; while the height is too low
//          {
//            cycle_i = 1;
//            len =0;
//            maxheight = a[fft_i*2];
//            minheight = 0;
//            pWave_Data->Absolute_Height = 0;
//
//            pWave_Data->Cycle = 0;
//            first_down_count = 0;
//
//
//          /*----------------------   ���̽��� ׼����һ�� ---------------------*/
//            pWave_Data->Heading_count_flag = 0;
//          }else//the wave meet all the conditions                     float Max_Height;   float Max_Cycle;
//          {
//            pWave_Data->Absolute_Height = (maxheight - minheight)*9.0;
//            pWave_Data->Cycle = len *0.2;
//
//            cycle_i = 1;
//            len =0;
//            maxheight = 0;
//            minheight = 0;
//
//            /*--------------------���ݵ��룬���㲨������----------------------*/
//            Wave_Amplitude[pWave_Data->Wave_Cycle_Times].cj = (uint16_t)(pWave_Data->Absolute_Height*100);
//            Wave_Amplitude[pWave_Data->Wave_Cycle_Times].sub = pWave_Data->Wave_Cycle_Times;
//             Wave_Cycle[pWave_Data->Wave_Cycle_Times] = pWave_Data->Cycle;
//
//            pWave_Data->Wave_Cycle_Times ++;
//            first_down_count = 0;
//          }
//        }
//      }
//      break;
//    default:
//      break;
//    }
//}

void Get_Cycle_Height()
{
    static uint8_t len = 0;
    static float maxheight = 0, minheight1 = 0, minheight2 = 0, minheight = 0;
    static uint8_t cycle_i = 0;
    /*-------------
    step 0��Ѱ�ҵ�һ����͵㣻setp++��
    step 1��len++�� Ѱ����ߵ㣻step++��
    step 2��Ѱ�ҵڶ�����͵㣬���������������������������cycle��height������step 2
    �������㷵��step0��
    ----------*/
    switch( cycle_i )
    {
        case 0:
            // first min value
            if( ( a[fft_i * 2] > a[fft_i * 2 + 2] ) && ( a[fft_i * 2 + 2] < a[fft_i * 2 + 4] ) ) //a[n-1]>a[n] && a[n+1]>a[n]
            {
                len = 0;
                cycle_i = 1;
                minheight1 = a[fft_i * 2 + 2]; //first min value
            }
            break;
        case 1://max value
            len ++;
            if( ( a[fft_i * 2] < a[fft_i * 2 + 2] ) && ( a[fft_i * 2 + 2] > a[fft_i * 2 + 4] ) ) //a[m-1]>a[m] && a[m+1]>a[m]
            {
                cycle_i = 2;
                maxheight = a[fft_i * 2 + 2]; //max value
                /*------------------------   ���ϵĹ��̽���  ---------------------*/
                first_down_count++;
                pWave_Data->Heading_count_flag = 2;
            }
            /*------------------------   ���ϵĹ���   ----------------------------*/
            pWave_Data->Heading_count_flag = 1;
            break;
        case 2://second min down && cail
            len++;
            if( ( a[fft_i * 2] > a[fft_i * 2 + 2] ) && ( a[fft_i * 2 + 2] < a[fft_i * 2 + 4] ) ) //second min value
            {
                minheight2 = a[fft_i * 2 + 2];
                minheight = minheight2 > minheight1 ? minheight1 : minheight2; //find the min value
                if( len < 5 ) //ignore this  wave
                {
                    cycle_i = 1;
                    len = 0;
                    minheight1 = a[fft_i * 2 + 2];
                    //        pWave_Data->Absolute_Height = 0;//��������ʾ
                    //        pWave_Data->Cycle = 0;
                    /*----------------------   ���̽��� ׼����һ�� ---------------------*/
                    pWave_Data->Heading_count_flag = 0;
                }
                else //meet the length
                {
                    if( ( uint8_t )( ( maxheight - minheight ) * 100 ) < pHost_Cmd->Min_WaveHeught ) //ignore the wave; while the height is too low
                    {
                        cycle_i = 1;
                        len = 0;
                        minheight1 = a[fft_i * 2 + 2];
                        pWave_Data->Absolute_Height = 0;
                        pWave_Data->Cycle = 0;
                        first_down_count = 0;
                        /*----------------------   ���̽��� ׼����һ�� ---------------------*/
                        pWave_Data->Heading_count_flag = 0;
                    }
                    else //the wave meet all the conditions                     float Max_Height;   float Max_Cycle;
                    {
                        pWave_Data->Absolute_Height = ( maxheight - minheight ) * 9.0;
                        pWave_Data->Cycle = len * 0.2;
                        cycle_i = 1;
                        len = 0;
                        maxheight = 0;
                        minheight1 = a[fft_i * 2 + 2];
                        /*--------------------���ݵ��룬���㲨������----------------------*/
                        Wave_Amplitude[pWave_Data->Wave_Cycle_Times].cj = ( uint16_t )( pWave_Data->Absolute_Height * 100 );
                        Wave_Amplitude[pWave_Data->Wave_Cycle_Times].sub = pWave_Data->Wave_Cycle_Times;
                        Wave_Cycle[pWave_Data->Wave_Cycle_Times] = pWave_Data->Cycle;
                        pWave_Data->Wave_Cycle_Times ++;
                        first_down_count = 0;
                    }
                }
            }
            break;
        default:
            break;
    }
}


void Get_Routine_Data()
{
    uint8_t max_heading = 0;
    uint8_t direction_count = 0;
    max_heading = heading_count[0];
    pSenser_Data->State = pSenser_Data->State | 0xC0;//�ɼ����
    qsort( ( void* )Wave_Amplitude, pWave_Data->Wave_Cycle_Times, sizeof( PAIR ), compare );
    pWave_Data->Effective_Height = 0;
    pWave_Data->Effective_Cycle = 0;
    pWave_Data->One_Tenth_Height = 0;
    pWave_Data->One_Tenth_Cycle = 0;
    pWave_Data->Average_Height = 0;
    pWave_Data->Average_Cycle = 0;
    //  for(u8 i = 10; i<(u8)(pWave_Data->Wave_Cycle_Times/3)+10;i++)
    //  {
    //    pWave_Data->Effective_Height += Wave_Amplitude[i].cj;
    //    pWave_Data->Effective_Cycle  += Wave_Cycle[Wave_Amplitude[i].sub];
    //  }
    //
    //  for(u8 i = 10; i<(u8)(pWave_Data->Wave_Cycle_Times/10)+10;i++)
    //  {
    //    pWave_Data->One_Tenth_Height += Wave_Amplitude[i].cj;
    //    pWave_Data->One_Tenth_Cycle  += Wave_Cycle[Wave_Amplitude[i].sub];
    //  }
    //
    //  for(u8 i = 0; i<pWave_Data->Wave_Cycle_Times;i++)
    //  {
    //    pWave_Data->Average_Height += Wave_Amplitude[i].cj;
    //    pWave_Data->Average_Cycle  += Wave_Cycle[Wave_Amplitude[i].sub];
    //  }
    for( u8 i = 10; i < pWave_Data->Wave_Cycle_Times; i++ )
    {
        pWave_Data->Average_Height += Wave_Amplitude[i].cj;
        pWave_Data->Average_Cycle  += Wave_Cycle[Wave_Amplitude[i].sub];
        if( i < ( u8 )( pWave_Data->Wave_Cycle_Times / 3 + 10 ) )
        {
            pWave_Data->Effective_Height += Wave_Amplitude[i].cj;
            pWave_Data->Effective_Cycle  += Wave_Cycle[Wave_Amplitude[i].sub];
        }
        if( i < ( u8 )( pWave_Data->Wave_Cycle_Times / 10 + 10 ) )
        {
            pWave_Data->One_Tenth_Height += Wave_Amplitude[i].cj;
            pWave_Data->One_Tenth_Cycle  += Wave_Cycle[Wave_Amplitude[i].sub];
        }
    }
    pWave_Data->Max_Height = Wave_Amplitude[10].cj / 100.0;
    pWave_Data->Max_Cycle =  Wave_Cycle[Wave_Amplitude[10].sub];
    pWave_Data->One_Tenth_Height = pWave_Data->One_Tenth_Height / ( u8 )( ( pWave_Data->Wave_Cycle_Times ) / 10 ) / 100.0;
    pWave_Data->One_Tenth_Cycle = pWave_Data->One_Tenth_Cycle / ( u8 )( ( pWave_Data->Wave_Cycle_Times ) / 10 );
    pWave_Data->Effective_Height = pWave_Data->Effective_Height / ( u8 )( ( pWave_Data->Wave_Cycle_Times ) / 3 ) / 100.0;
    pWave_Data->Effective_Cycle = pWave_Data->Effective_Cycle / ( u8 )( ( pWave_Data->Wave_Cycle_Times ) / 3 );
    pWave_Data->Average_Height = pWave_Data->Average_Height / ( pWave_Data->Wave_Cycle_Times - 10 ) / 100.0;
    pWave_Data->Average_Cycle = pWave_Data->Average_Cycle / ( pWave_Data->Wave_Cycle_Times - 10 );
    if( pWave_Data->Effective_Cycle > 25.5 )
    {
        pWave_Data->Effective_Height = pWave_Data->Effective_Height * 0.9;
        pWave_Data->One_Tenth_Height = pWave_Data->One_Tenth_Height * 0.9;
        pWave_Data->Average_Height = pWave_Data->Average_Height * 0.9;
        pWave_Data->Effective_Cycle = 24.9;//pWave_Data->Effective_Cycle *0.9;
        pWave_Data->One_Tenth_Cycle = 25.2;//pWave_Data->One_Tenth_Cycle*0.9;
        //                  pWave_Data->Average_Cycle = pWave_Data->Average_Cycle*0.9;
    }
    /*ͳ����Ҫ����*/
    for( uint8_t i = 1; i < 16; i++ )
    {
        if( max_heading < heading_count[i] )
        {
            direction_count = i;
            max_heading = heading_count[i];
        }
    }
    pWave_Data->Main_heading = direction_count * 22.5 - 11.3;
    if( pWave_Data->Main_heading < 0 )
    {
        pWave_Data->Main_heading += 360;
    }
    memset( heading_count, 0, 16 );
    pWave_Data->Num_Of_Wave = pWave_Data->Wave_Cycle_Times;
    pWave_Data->Wave_Cycle_Times = 0;
}

void Tradion_Sample_Flinish()//����۲��Ƿ����
{
    if( ( pHost_Cmd->Scan_Mode_Select == 1 ) && ( pWave_Data->Tradion_Sample_Start == 1 ) && ( pWave_Data->Tradion_Sample_Enable == 1 ) ) //������ʽΪ100��
    {
        if( pWave_Data->Wave_Cycle_Times >= 100 )
        {
            Get_Routine_Data();
            pHost_Cmd->Receive_CMD_Flag = 1; //ÿ�γ���۲�����Զ�����ͳ�ƽ��
            pHost_Cmd->Host_Data_Package = 4;
            pWave_Data->Wave_Cycle_Times = 0;
            pWave_Data->Tradion_Sample_Start = 0;
            pWave_Data->Tradion_Sample_complete = 1;
            pRTC_Time->Routinue_year = pRTC_Time->year;
            pRTC_Time->Routinue_month = pRTC_Time->mon;
            pRTC_Time->Routinue_day = pRTC_Time->day;
            pRTC_Time->Routinue_hour = pRTC_Time->hour;
            pRTC_Time->Routinue_min = pRTC_Time->min;
            pRTC_Time->Routinue_sec = pRTC_Time->sec;
        }
    }
    else if( ( pHost_Cmd->Scan_Mode_Select == 2 ) && ( pWave_Data->Tradion_Sample_Start == 1 ) && ( pWave_Data->Tradion_Sample_Enable == 1 ) ) //1024s����
    {
        uint16_t Routinue_count = 0;
        Routinue_count = Is_Reaching_IntervalTime( pRTC_Time->Routinue_year,
                         pRTC_Time->Routinue_month,
                         pRTC_Time->Routinue_day,
                         pRTC_Time->Routinue_hour,
                         pRTC_Time->Routinue_min,
                         pRTC_Time->Routinue_sec );
        if( Routinue_count > 1000 ) //mode 1024s  ��Ϊ1000s
        {
            Get_Routine_Data();
            pRTC_Time->Routinue_year = pRTC_Time->year;
            pRTC_Time->Routinue_month = pRTC_Time->mon;
            pRTC_Time->Routinue_day = pRTC_Time->day;
            pRTC_Time->Routinue_hour = pRTC_Time->hour;
            pRTC_Time->Routinue_min = pRTC_Time->min;
            pRTC_Time->Routinue_sec = pRTC_Time->sec;
            pHost_Cmd->Receive_CMD_Flag = 1; //ÿ�γ���۲�����Զ�����ͳ�ƽ��
            pHost_Cmd->Host_Data_Package = 4;
            pWave_Data->Wave_Cycle_Times = 0;
            pWave_Data->Tradion_Sample_Start = 0;
            pWave_Data->Tradion_Sample_complete = 1;
            pRTC_Time->Routinue_year = pRTC_Time->year;
            pRTC_Time->Routinue_month = pRTC_Time->mon;
            pRTC_Time->Routinue_day = pRTC_Time->day;
            pRTC_Time->Routinue_hour = pRTC_Time->hour;
            pRTC_Time->Routinue_min = pRTC_Time->min;
            pRTC_Time->Routinue_sec = pRTC_Time->sec;
        }
    }
}

void Tradion_Sample()//����۲��Ƿ�ʼ
{
    uint16_t Reaching_count = 0;
    if( pWave_Data->Tradion_Sample_complete == 1 ) //���һ�βɼ��� ��ʼ������һ�ε�ʵ��
    {
        Reaching_count = Is_Reaching_IntervalTime( pRTC_Time->Interval_year,
                         pRTC_Time->Interval_month,
                         pRTC_Time->Interval_day,
                         pRTC_Time->Interval_hour,
                         pRTC_Time->Interval_min,
                         pRTC_Time->Interval_sec );
    }
    if( Reaching_count > pHost_Cmd->Routine_Time_Spacing )
    {
        pWave_Data->Tradion_Sample_Start = 1;
        pWave_Data->Tradion_Sample_complete = 0;
        pRTC_Time->Interval_year = pRTC_Time->year;
        pRTC_Time->Interval_month = pRTC_Time->mon;
        pRTC_Time->Interval_day = pRTC_Time->day;
        pRTC_Time->Interval_hour = pRTC_Time->hour;
        pRTC_Time->Interval_min = pRTC_Time->min;
        pRTC_Time->Interval_sec = pRTC_Time->sec;
        Reaching_count = 0;
        fft_i = 0;//���һ�βɼ���  ���¿�ʼ�ɼ�
        pWave_Data->Cycle = 0; //������0
        pWave_Data->Absolute_Height = 0;
        pWave_Data->Height = 0;
    }
    if( pWave_Data->Tradion_Sample_Start == 1 )
    {
        Wave_Height_Calculate();
    }
}




/*----------------------  ����ʱ���  ---------------------------------*/
uint32_t Is_Reaching_IntervalTime( u16 year, u8 mon, u8 day, u8 hour, u8 min, u8 sec )
{
    uint32_t Interval_Time;//���ʱ�䣬���18h
    Interval_Time = YMD_To_Day( year, mon, day ) * 86400;
    Interval_Time += HMS_To_Second( day, hour, min, sec );
    return Interval_Time;
}

uint16_t YMD_To_Day( u16 year, u8 mon, u8 day ) //���һ��
{
    uint16_t day_start, day_end, YMD_day;
    uint8_t month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    for( u8 i = 0; i < mon - 1; i++ )
    {
        day_start += month[i];
    }
    day_start += day;
    day_start = ( mon >= 3 ) ? day_start + Leap_Year( year ) : day_start;
    for( u8 j = 0; j < pRTC_Time->mon - 1; j++ )
    {
        day_end += month[j];
    }
    day_end += pRTC_Time->day;
    day_end = ( pRTC_Time->mon >= 3 ) ? day_start + Leap_Year( pRTC_Time->year ) : day_start;
    if( year == pRTC_Time->year )
    {
        YMD_day = day_end - day_start;
    }
    else
    {
        YMD_day = 365 - day_start + day_end + Leap_Year( year );
    }
    return YMD_day;
}

uint32_t HMS_To_Second( u8 day, u8 hour, u8 min, u8 sec ) //�ο���ʱ��
{
    u32 second_start, second_end, HMS_second;
    second_start = sec + min * 60 + hour * 3600;
    second_end = pRTC_Time->sec + 60 * pRTC_Time->min + 3600 * pRTC_Time->hour;
    if( day == pRTC_Time->day ) //ͬһ��
    {
        HMS_second = second_end - second_start;
    }
    else
    {
        HMS_second = 86400 - second_start + second_end;
    }
    return HMS_second;
}


uint8_t Leap_Year( int year ) //�����ж�
{
    return ( ( year % 400 == 0 ) || ( ( year % 4 == 0 ) && ( year % 100 != 0 ) ) ) ? 1 : 0;
}


