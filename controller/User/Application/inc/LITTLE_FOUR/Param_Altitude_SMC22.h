#define INCLUDED_UBLOX_PARAM_ALTITUDE_SMC_H

// Header for FRAME NAME 
// This header was generated by MATLAB. 
// 27-Sep-2016 13:09:06
//
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -6.9150068762, -25.0000000000, -27.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.U2AKalmanParam.Qk =    {  1.0000000000};
CtrData.U2AKalmanParam.Rk =     {  0.1000000000};
CtrData.V2PKalmanParam.Qk =    {  0.1000000000};
CtrData.V2PKalmanParam.Rk =     {  1.0000000000};
CtrData.A2PKalmanParam.Qk =    {  1.0000000000};
CtrData.A2PKalmanParam.Rk =     {  0.1000000000};
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
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.5000000000} 
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
    {  0.8706145449};

double Ublox_Alt_U2AKalman_B[1][2] = 
    {  0.0024873893,  0.0002227367};

double Ublox_Alt_U2AKalman_C[1][1] = 
    {  0.9997442270};

double Ublox_Alt_U2AKalman_D[1][2] = 
    {  0.0000000000,  0.0002557730};

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
    {  1.0000000000, -0.0616748860,  0.0199998000}, 
    {  0.0200000000,  0.9497143933,  0.0001999987}, 
    {  0.0000000000,  0.0000000000,  0.9999800002} 
};

double Ublox_Alt_A2PKalman_B[3][2] = 
{ 
    {  0.0200000000,  0.0616748860}, 
    {  0.0002000000,  0.0502856067}, 
    {  0.0000000000,  0.0000000000} 
};

double Ublox_Alt_A2PKalman_C[2][3] = 
{ 
    {  1.0000000000, -0.0616748860,  0.0000000000}, 
    {  0.0000000000,  0.9509478910,  0.0000000000} 
};

double Ublox_Alt_A2PKalman_D[2][2] = 
{ 
    { -0.0000000000,  0.0616748860}, 
    { -0.0000000000,  0.0490521090} 
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
    {  0.0000000000, 187.7330809803, 202.7517274587};

double Ublox_Alt_Br= 202.7517274587; 

double Ublox_A_refmodel_Alt[3][3] = 
{ 
    {  0.8662454882, -0.4713385191, -0.5034780304}, 
    {  0.0186473345,  0.9951919342, -0.0051551576}, 
    {  0.0001909318,  0.0199676289,  0.9999652283} 
};

double Ublox_B_refmodel_Alt[3] = 
    {  0.5034780304,  0.0051551576,  0.0000347717};

double Ublox_C_refmodel_Alt[3] = 
    {  0.0000000000,  0.0000000000,  1.0000000000};

double Ublox_error_Alt[4]; 

double Ublox_Klp_Alt[4]= 
    { -67.3411061788, -388.8001060125, -1240.2371432564, -951.4656291622};

double Ublox_Knp_Alt = -3.8469325055; 

double Ublox_K1_Alt[3]= 
    {  0.0000000000, -187.7330809803, -202.7517274587};

double Ublox_K2_Alt = 202.7517274587; 

double Ublox_S_Alt[4]= 
    {  9.7601442558, 57.4157404089, 175.1840337557, 123.6654955346};
