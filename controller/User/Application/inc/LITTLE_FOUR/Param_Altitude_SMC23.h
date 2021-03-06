#define INCLUDED_UBLOX_PARAM_ALTITUDE_SMC_H

// Header for JP_CAN_HEX 
// This header was generated by MATLAB. 
// 27-Sep-2016 17:06:04
//
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -6.9150068762, -22.0000000000, -27.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.U2AKalmanParam.Qk =    { 50.0000000000};
CtrData.U2AKalmanParam.Rk =     {  0.1000000000};
CtrData.V2PKalmanParam.Qk =    {  0.1000000000};
CtrData.V2PKalmanParam.Rk =     {  1.0000000000};
CtrData.A2PKalmanParam.Qk =    {  1.0000000000};
CtrData.A2PKalmanParam.Rk =     {  0.2000000000};
CtrData.SMC.Q= { 
    { 100.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000, 10.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.1000000000} 
};
CtrData.SMC.R =     {  1.0000000000};
CtrData.SMC.AA = { 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.1000000000} 
};
CtrData.SMC.rou =     {  5.0000000000};
CtrData.SMC.fai =     { -10.0000000000};
*/
///////////////////////////////////////////////////////////////////////////

#define MRSMC_Theta_Control  
#define Ublox_K_Alt 3
double Ublox_X_Alt_Ref[3]; 
#define Ublox_U2AKalman_K_Alt 1
double Ublox_X_Alt_U2AKalman[1]; 
#define Ublox_V2PKalman_K_Alt 2
double Ublox_X_Alt_V2PKalman[2]; 
#define Ublox_A2PKalman_K_Alt 3
double Ublox_X_Alt_A2PKalman[3]; 
//////////////////////////////U2A Kalman Model////////////////////////////////
double Ublox_Alt_U2AKalman_A[1][1] = 
    {  0.8602299398};

double Ublox_Alt_U2AKalman_B[1][2] = 
    {  0.0024873893,  0.0106073418};

double Ublox_Alt_U2AKalman_C[1][1] = 
    {  0.9878193756};

double Ublox_Alt_U2AKalman_D[1][2] = 
    {  0.0000000000,  0.0121806244};

//////////////////////////////V2Alt Kalman Model////////////////////////////////
double Ublox_Alt_V2PKalman_A[2][2] = 
{ 
    {  0.0000000000,  0.0000000000}, 
    { -0.0075817831,  0.9941751551} 
};

double Ublox_Alt_V2PKalman_B[2][2] = 
{ 
    {  0.0000010000,  0.0000000000}, 
    {  0.0000000200, -0.0000000058} 
};

double Ublox_Alt_V2PKalman_C[1][2] = 
    { -1104213.3849559545, -848220.4524166187};

double Ublox_Alt_V2PKalman_D[1][2] = 
    {  0.0000000000,  0.1517795476};

//////////////////////////////A2Alt Kalman Model////////////////////////////////
double Ublox_Alt_A2PKalman_A[3][3] = 
{ 
    {  1.0000000000, -0.0437855651,  0.0199998000}, 
    {  0.0200000000,  0.9577121431,  0.0001999987}, 
    {  0.0000000000,  0.0000000000,  0.9999800002} 
};

double Ublox_Alt_A2PKalman_B[3][2] = 
{ 
    {  0.0200000000,  0.0437855651}, 
    {  0.0002000000,  0.0422878569}, 
    {  0.0000000000,  0.0000000000} 
};

double Ublox_Alt_A2PKalman_C[2][3] = 
{ 
    {  1.0000000000, -0.0437855651,  0.0000000000}, 
    {  0.0000000000,  0.9585878544,  0.0000000000} 
};

double Ublox_Alt_A2PKalman_D[2][2] = 
{ 
    { -0.0000000000,  0.0437855651}, 
    { -0.0000000000,  0.0414121456} 
};

//////////////////////////////Reference Model////////////////////////////////
double Ublox_Alt_A_d[3][3] = 
{ 
    {  0.8708372816,  0.0000000000,  0.0000000000}, 
    {  0.0186786103,  1.0000000000,  0.0000000000}, 
    {  0.0001910902,  0.0200000000,  1.0000000000} 
};

double Ublox_Alt_B_d[3] = 
    {  0.0024873893,  0.0000254471,  0.0000001716};

double Ublox_Alt_F[3] = 
    {  0.0000000000, 165.2051112627, 202.7517274587};

double Ublox_Alt_Br= 202.7517274587; 

double Ublox_A_refmodel_Alt[3][3] = 
{ 
    {  0.8667920653, -0.4154791087, -0.5035787676}, 
    {  0.0186510655,  0.9957643112, -0.0051556684}, 
    {  0.0001909507,  0.0199714907,  0.9999652262} 
};

double Ublox_B_refmodel_Alt[3] = 
    {  0.5035787676,  0.0051556684,  0.0000347738};

double Ublox_C_refmodel_Alt[3] = 
    {  0.0000000000,  0.0000000000,  1.0000000000};

double Ublox_error_Alt[4]; 

double Ublox_Klp_Alt[4]= 
    { -57.4320659370, -238.8461164810, -444.8116657555, -336.3665372417};

double Ublox_Knp_Alt = -9.1620324363; 

double Ublox_K1_Alt[3]= 
    {  0.0000000000, -165.2051112627, -202.7517274587};

double Ublox_K2_Alt = 202.7517274587; 

double Ublox_S_Alt[4]= 
    {  4.0980662814, 18.6999203602, 33.5038510154, 18.3565458635};

