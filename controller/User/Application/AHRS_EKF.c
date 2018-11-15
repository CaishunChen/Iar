#include "AHRS_EKF.h"
#include "BNO055.h"
#include "structural.h"
#include "filter.h"
#define TEST_MODE


#define g     9.7949
#define g2    19.5898
#define r2d   57.2958
#define d2r   0.01745
//#define pi    3.141592
#define pi2   6.283184

#define ANGLE_Update   25
#define MAG_Update     10
#define AHRS_Run       50

#define var_ax  0.962361*4
#define var_ay  0.962361*4
#define var_az  0.962361*4

#ifdef TEST_MODE
#define var_psi 0.504924*0.05
#else
#define var_psi 0.504924
#endif

extern d_Senser_Data*  pSenser_Data;

float x_e[7][1] = {0.0};
float P[7][7] = {0.0};
float R[3][3] = {0.0};
float F[7][7] = {0.0};
float Q1 = 0.0000001;
float Q2 = 0.0000000;
float test3;
float hx = 0.0, hy = 0.0;
float psi = 0.0;
double Yaw = 0.0;

float Compensate_Zacc;

void AHRS_EKF_Init()
{
    hx = pSenser_Data->magX;
    hy = pSenser_Data->magY;
    int i, j;
    psi = -atan2( -hy, -hx );
    for( i = 0; i < 7; i++ )
    {
        x_e[i][0] = 0.0;
    }
    x_e[0][0] = cos( psi * 0.5 );
    x_e[3][0] = sin( psi * 0.5 );
    for( i = 0; i < 7; i++ )
        for( j = 0; j < 7; j++ )
        {
            P[i][j] = 0.0;
        }
    P[0][0] = 0.000001;
    P[1][1] = 0.000001;
    P[2][2] = 0.000001;
    P[3][3] = 0.000001;
    P[4][4] = 0.0001;
    P[5][5] = 0.0001;
    P[6][6] = 0.0001;
    for( i = 0; i < 3; i++ )
        for( j = 0; j < 3; j++ )
        {
            R[i][j] = 0.0;
        }
    R[0][0] = var_ax;
    R[1][1] = var_ay;
    R[2][2] = var_az;
    for( i = 0; i < 7; i++ )
    {
        F[i][i] = 1.0;
    }
    Q1 = 0.0000001;
    Q2 = 0.000000;
}


void AHRS_EKF()
{
    //GPIO_SetBits(GPIOC, GPIO_Pin_6);
    /////////////////p1-1
    short  i = 0, j = 0;
    float K[7][3] = {0.0};
    float H[3][7] = {0.0};
    float Hpsi[1][7] = {0.0};
    float Kpsi[7][1] = {0.0};
    float pc, qc, rc;
    float cPHI, sPHI;
    float norm, Bxc, Byc, invR;
    float dt, Hdt;
    float coeff1[3] = {0.0}, temp[2] = {0.0};
    static unsigned short counter = 0;
    float dTemp[4] = {0};
    dt = 0.02;
    Hdt = 0.5 * dt;
    pc = ( ( pSenser_Data->RollRate )  - x_e[4][0] ) * Hdt;
    qc = ( ( pSenser_Data->PitchRate ) - x_e[5][0] ) * Hdt;
    rc = ( ( pSenser_Data->YawRate )   - x_e[6][0] ) * Hdt;
    F[0][1] = ( -pc );
    F[0][2] = ( -qc );
    F[0][3] = ( -rc );
    F[1][0] = pc;
    F[1][2] = rc;
    F[1][3] = ( -qc );
    F[2][0] = qc;
    F[2][1] = ( -rc );
    F[2][3] = pc;
    F[3][0] = rc;
    F[3][1] = qc;
    F[3][2] = ( -pc );
    dTemp[0] =  x_e[1][0] * Hdt;
    F[0][4] = dTemp[0];
    dTemp[0] =  x_e[2][0] * Hdt;
    F[0][5] = dTemp[0];
    dTemp[0] =  x_e[3][0] * Hdt;
    F[0][6] = dTemp[0];
    dTemp[0] = -x_e[0][0] * Hdt;
    F[1][4] = dTemp[0];
    dTemp[0] =  x_e[3][0] * Hdt;
    F[1][5] = dTemp[0];
    dTemp[0] = -F[0][5];
    F[1][6] = dTemp[0];
    dTemp[0] = -F[1][5];
    F[2][4] = dTemp[0];
    dTemp[0] =  F[1][4];
    F[2][5] = dTemp[0];
    dTemp[0] =  F[0][4];
    F[2][6] = dTemp[0];
    dTemp[0] =  F[0][5];
    F[3][4] = dTemp[0];
    dTemp[0] = -F[0][4];
    F[3][5] = dTemp[0];
    dTemp[0] =  F[1][4];
    F[3][6] = dTemp[0];
    dTemp[0] = x_e[0][0] - pc * x_e[1][0] - qc * x_e[2][0] - rc * x_e[3][0];
    dTemp[1] = x_e[1][0] + pc * x_e[0][0] - qc * x_e[3][0] + rc * x_e[2][0];
    dTemp[2] = x_e[2][0] + pc * x_e[3][0] + qc * x_e[0][0] - rc * x_e[1][0];
    dTemp[3] = x_e[3][0] - pc * x_e[2][0] + qc * x_e[1][0] + rc * x_e[0][0];
    for( i = 0; i < 4; i++ )
    {
        x_e[i][0] = dTemp[i];
    }
    float F1[7][7];
    for( i = 0; i < 7; i++ )
    {
        for( j = 0; j < 7; j++ )
        {
            F1[i][j] = F[j][i]; //F1为F的转秩
        }
    }
    float mid[7][7] = {0};
    for( i = 0; i < 7; i++ )
    {
        for( j = 0; j < 7; j++ )
        {
            mid[i][j] = P[i][0] * F1[0][j] + P[i][1] * F1[1][j] + P[i][2] * F1[2][j] + P[i][3] * F1[3][j] + P[i][4] * F1[4][j] + P[i][5] * F1[5][j] + P[i][6] * F1[6][j];
        }
    }
    for( i = 0; i < 7; i++ )
    {
        for( j = 0; j < 7; j++ )
        {
            P[i][j] = F[i][0] * mid[0][j] + F[i][1] * mid[1][j] + F[i][2] * mid[2][j] + F[i][3] * mid[3][j] + F[i][4] * mid[4][j] + F[i][5] * mid[5][j] + F[i][6] * mid[6][j];
        }
    }//算出P P=F*P*F1+Q
    for( i = 0; i < 4; i++ )
    {
        dTemp[0] = P[i][i] + Q1;
        P[i][i] = dTemp[0];
    }
    for( i = 4; i < 7; i++ )
    {
        dTemp[0] = P[i][i];
        P[i][i] = dTemp[0] + Q2;
    }
    counter++;
    if( counter >= 50 )
    {
        counter = 0;
    }
    float ax = pSenser_Data->BNOXacc;
    float ay = pSenser_Data->BNOYacc;
    float az = pSenser_Data->BNOZacc;
    ///////////////p1-2
    if( counter % ( AHRS_Run / ANGLE_Update ) == 0 )
    {
        //p2-1
        float y[3][1];
        float y_s[3][1];
        dTemp[0] = -g2 * ( x_e[1][0] * x_e[3][0] - x_e[0][0] * x_e[2][0] );
        y[0][0]  = dTemp[0];
        dTemp[0] = -g2 * ( x_e[0][0] * x_e[1][0] + x_e[2][0] * x_e[3][0] );
        y[1][0]  = dTemp[0] ;
        dTemp[0] = -g * ( x_e[0][0] * x_e[0][0] - x_e[1][0] * x_e[1][0] - x_e[2][0] * x_e[2][0] + x_e[3][0] * x_e[3][0] );
        y[2][0]  = dTemp[0];
        y_s[0][0] = ax;
        y_s[1][0] = ay;
        y_s[2][0] = az;
        dTemp[0] =  g2 * x_e[2][0];
        H[0][0] = dTemp[0];
        dTemp[0] = -g2 * x_e[3][0];
        H[0][1] = dTemp[0];
        dTemp[0] =  g2 * x_e[0][0];
        H[0][2] = dTemp[0];
        dTemp[0] = -g2 * x_e[1][0];
        H[0][3] = dTemp[0];
        dTemp[0] =  H[0][3];
        H[1][0] = dTemp[0];
        dTemp[0] = -H[0][2];
        H[1][1] = dTemp[0];
        dTemp[0] =  H[0][1];
        H[1][2] = dTemp[0];
        dTemp[0] = -H[0][0];
        H[1][3] = dTemp[0];
        dTemp[0] = -H[0][2];
        H[2][0] = dTemp[0];
        dTemp[0] = -H[0][3];
        H[2][1] = dTemp[0];
        dTemp[0] =  H[0][0];
        H[2][2] = dTemp[0];
        dTemp[0] =  H[0][1];
        H[2][3] = dTemp[0];
        float H1[7][3] = {0.0};
        for( i = 0; i < 7; i++ )
            for( j = 0; j < 3; j++ )
            {
                H1[i][j] = H[j][i];    //H1为H的转秩
            }
        float mid2[7][3] = {0.0};
        for( i = 0; i < 7; i++ )
        {
            for( j = 0; j < 3; j++ )
            {
                mid2[i][j] = P[i][0] * H1[0][j] + P[i][1] * H1[1][j] + P[i][2] * H1[2][j] + P[i][3] * H1[3][j] + P[i][4] * H1[4][j] + P[i][5] * H1[5][j] + P[i][6] * H1[6][j];
            }
        }
        float mid3[3][3] = {0.0};
        for( i = 0; i < 3; i++ )
        {
            for( j = 0; j < 3; j++ )
            {
                mid3[i][j] = H[i][0] * mid2[0][j] + H[i][1] * mid2[1][j] + H[i][2] * mid2[2][j] + H[i][3] * mid2[3][j] + H[i][4] * mid2[4][j] + H[i][5] * mid2[5][j] + H[i][6] * mid2[6][j];
            }
        }
        for( i = 0; i < 3; i++ )
        {
            for( j = 0; j < 3; j++ )
            {
                mid3[i][j] += R[i][j];
            }
        }
        static float invmid3[3][3] = {0.0};
        float judge = 0.0;
        judge = mid3[0][0] * mid3[1][1] * mid3[2][2] + mid3[0][1] * mid3[1][2] * mid3[2][0] + mid3[0][2] * mid3[1][0] * mid3[2][1] - mid3[0][0] * mid3[1][2] * mid3[2][1] - mid3[0][1] * mid3[1][0] * mid3[2][2] - mid3[0][2] * mid3[1][1] * mid3[2][0];
        if( judge != 0 )
        {
            MatrixOpp( mid3, invmid3 ); //求逆
        }
        float midd[7][3] = {0.0};
        for( i = 0; i < 7; i++ )
        {
            for( j = 0; j < 3; j++ )
            {
                midd[i][j] = H1[i][0] * invmid3[0][j] + H1[i][1] * invmid3[1][j] + H1[i][2] * invmid3[2][j];
            }
        }
        for( i = 0; i < 7; i++ )
        {
            for( j = 0; j < 3; j++ )
            {
                K[i][j] = P[i][0] * midd[0][j] + P[i][1] * midd[1][j] + P[i][2] * midd[2][j] + P[i][3] * midd[3][j] + P[i][4] * midd[4][j] + P[i][5] * midd[5][j] + P[i][6] * midd[6][j];
            }
        }//求出K
        float e[3][1] = {0.0};
        for( i = 0; i < 3; i++ )
        {
            e[i][0] = y_s[i][0] - y[i][0];
        }
        float mid4[7][1] = {0.0};
        for( i = 0; i < 7; i++ )
        {
            mid4[i][0] = K[i][0] * e[0][0] + K[i][1] * e[1][0] + K[i][2] * e[2][0];
        }
        for( i = 0; i < 7; i++ )
        {
            x_e[i][0] = x_e[i][0] + mid4[i][0];
        }
        //////////p2-2
    }
    if( counter % ( AHRS_Run / MAG_Update ) == 1 )
    {
        //p3-1
        float calibrated_hx = pSenser_Data->magX;
        float calibrated_hy = pSenser_Data->magY;
        cPHI = cos( pSenser_Data->Roll );
        sPHI = sin( pSenser_Data->Roll );
        Bxc = calibrated_hx * cos( pSenser_Data->Pitch ) + ( calibrated_hy * sPHI + pSenser_Data->magZ * cPHI ) * sin( pSenser_Data->Pitch );
        Byc = calibrated_hy * cPHI - pSenser_Data->magZ * sPHI;
        norm = 1.0 / sqrt( x_e[0][0] * x_e[0][0] + x_e[1][0] * x_e[1][0] + x_e[2][0] * x_e[2][0] + x_e[3][0] * x_e[3][0] );
        for( i = 0; i < 4; i++ )
        {
            dTemp[0] = x_e[i][0] * norm;
            x_e[i][0] = dTemp[0];
        }
        coeff1[0] = 2 * ( x_e[1][0] * x_e[2][0] + x_e[0][0] * x_e[3][0] );
        coeff1[1] = 1 - 2 * ( x_e[2][0] * x_e[2][0] + x_e[3][0] * x_e[3][0] );
        coeff1[2] = 2 / ( coeff1[0] * coeff1[0] + coeff1[1] * coeff1[1] );
        temp[0] = coeff1[1] * coeff1[2];
        temp[1] = coeff1[0] * coeff1[2];
        dTemp[0] = x_e[3][0] * temp[0];
        Hpsi[0][0] = dTemp[0];
        dTemp[0] = x_e[2][0] * temp[0];
        Hpsi[0][1] = dTemp[0];
        dTemp[0] = x_e[1][0] * temp[0] + 2 * x_e[2][0] * temp[1];
        Hpsi[0][2] = dTemp[0];
        dTemp[0] = x_e[0][0] * temp[0] + 2 * x_e[3][0] * temp[1];
        Hpsi[0][3] = dTemp[0];
        float Hpsi1[7][1];
        for( i = 0; i < 7; i++ )
        {
            Hpsi1[i][0] = Hpsi[0][i];
        }
        float mid4[7][1];
        for( i = 0; i < 7; i++ )
        {
            mid4[i][0] = P[i][0] * Hpsi1[0][0] + P[i][1] * Hpsi1[1][0] + P[i][2] * Hpsi1[2][0] + P[i][3] * Hpsi1[3][0] + P[i][4] * Hpsi1[4][0] + P[i][5] * Hpsi1[5][0] + P[i][6] * Hpsi1[6][0];
        }
        invR = 1.0 / ( Hpsi[0][0] * mid4[0][0] + Hpsi[0][1] * mid4[1][0] + Hpsi[0][2] * mid4[2][0] + Hpsi[0][3] * mid4[3][0] + var_psi );
        dTemp[1] = atan2( coeff1[0], coeff1[1] );
        for( i = 0; i < 7; i++ )
        {
            dTemp[0] = mid4[i][0] * invR;
            Kpsi[i][0] = dTemp[0];
        }
        dTemp[0] = wraparound( atan2( Byc, -Bxc ) - dTemp[1] );
        for( i = 0; i < 7; i++ )
        {
            dTemp[2] = x_e[i][0] + Kpsi[i][0] * dTemp[0];
            x_e[i][0] = dTemp[2];
        }
        float mid5[3][7];
        for( i = 0; i < 3; i++ )
            for( j = 0; j < 7; j++ )
            {
                mid5[i][j] = H[i][0] * P[0][j] + H[i][1] * P[1][j] + H[i][2] * P[2][j] + H[i][3] * P[3][j] + H[i][4] * P[4][j] + H[i][5] * P[5][j] + H[i][6] * P[6][j];
            }
        float mid6[7][7];
        for( i = 0; i < 7; i++ )
            for( j = 0; j < 7; j++ )
            {
                mid6[i][j] = K[i][0] * mid5[0][j] + K[i][1] * mid5[1][j] + K[i][2] * mid5[2][j];
            }
        for( i = 0; i < 7; i++ )
            for( j = 0; j < 7; j++ )
            {
                P[i][j] = P[i][j] - mid6[i][j];
            }
        float mid7[1][7];
        for( i = 0; i < 1; i++ )
            for( j = 0; j < 7; j++ )
            {
                mid7[i][j] = Hpsi[i][0] * P[0][j] + Hpsi[i][1] * P[1][j] + Hpsi[i][2] * P[2][j] + Hpsi[i][3] * P[3][j] + Hpsi[i][4] * P[4][j] + Hpsi[i][5] * P[5][j] + Hpsi[i][6] * P[6][j];
            }
        for( i = 0; i < 7; i++ )
            for( j = 0; j < 7; j++ )
            {
                mid6[i][j] = Kpsi[i][0] * mid7[0][j];
            }
        for( i = 0; i < 7; i++ )
            for( j = 0; j < 7; j++ )
            {
                P[i][j] = P[i][j] - mid6[i][j];
            }
        //p3-2
    }
    //p4-1
    norm = 1.0 / sqrt( x_e[0][0] * x_e[0][0] + x_e[1][0] * x_e[1][0] + x_e[2][0] * x_e[2][0] + x_e[3][0] * x_e[3][0] );
    for( i = 0; i < 4; i++ )
    {
        dTemp[0] = x_e[i][0] * norm;
        x_e[i][0] = dTemp[0];
    }
    pSenser_Data->Pitch = asin( -2 * ( x_e[1][0] * x_e[3][0] - x_e[0][0] * x_e[2][0] ) );
    pSenser_Data->Roll  = atan2( 2 * ( x_e[0][0] * x_e[1][0] + x_e[2][0] * x_e[3][0] ), 1 - 2 * ( x_e[1][0] * x_e[1][0] + x_e[2][0] * x_e[2][0] ) );
    //  pSenser_Data->Yaw   = atan2(2*(x_e[1][0]*x_e[2][0]+x_e[0][0]*x_e[3][0]),1-2*(x_e[2][0]*x_e[2][0]+x_e[3][0]*x_e[3][0]));
    //Yaw_Recalculate();
    //p4-2
    //GPIO_ResetBits(GPIOC, GPIO_Pin_6);
    //=pAHRS_Data_filtered->Yaw*57.3;
}

void  MatrixOpp( float A[][3], float B[][3] ) /*矩阵求逆*/
{
    float A1, A11, A12, A13, A21, A22, A23, A31, A32, A33;
    A1 = A[0][0] * A[1][1] * A[2][2] + A[0][1] * A[1][2] * A[2][0] + A[0][2] * A[1][0] * A[2][1] - A[0][0] * A[1][2] * A[2][1] - A[0][1] * A[1][0] * A[2][2] - A[0][2] * A[1][1] * A[2][0];
    A11 = A[1][1] * A[2][2] - A[1][2] * A[2][1];
    A12 = -( A[1][0] * A[2][2] - A[1][2] * A[2][0] );
    A13 = A[1][0] * A[2][1] - A[1][1] * A[2][0];
    A21 = -( A[0][1] * A[2][2] - A[0][2] * A[2][1] );
    A22 = A[0][0] * A[2][2] - A[0][2] * A[2][0];
    A23 = -( A[0][0] * A[2][1] - A[0][1] * A[2][0] );
    A31 = A[0][1] * A[1][2] - A[1][1] * A[0][2];
    A32 = -( A[0][0] * A[1][2] - A[0][2] * A[1][0] );
    A33 = A[0][0] * A[1][1] - A[1][0] * A[0][1];
    B[0][0] = A11 / A1;
    B[1][0] = A12 / A1;
    B[2][0] = A13 / A1;
    B[0][1] = A21 / A1;
    B[1][1] = A22 / A1;
    B[2][1] = A23 / A1;
    B[0][2] = A31 / A1;
    B[1][2] = A32 / A1;
    B[2][2] = A33 / A1;
}

float wraparound( float dta )
{
    //bound heading angle between -180 and 180
    if( dta >  pi )
    {
        dta -= pi2;
    }
    if( dta < -pi )
    {
        dta += pi2;
    }
    return dta;
}









