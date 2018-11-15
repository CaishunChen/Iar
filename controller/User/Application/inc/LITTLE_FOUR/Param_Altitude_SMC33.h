#define INCLUDED_UBLOX_PARAM_ALTITUDE_SMC_H

// Header for JP_CAN_HEX 
// This header was generated by MATLAB. 
// 31-Oct-2016 11:04:58
//
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -10.3122451761, -25.0000000000, -22.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.U2AKalmanParam.Qk =    { 3000.0000000000};
CtrData.U2AKalmanParam.Rk =     {  1.0000000000};
CtrData.V2PKalmanParam.Qk =    {  1.0000000000};
CtrData.V2PKalmanParam.Rk =     { 100.0000000000};
CtrData.A2PKalmanParam.Qk =    { 100.0000000000};
CtrData.A2PKalmanParam.Rk =     {  1.0000000000};
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
    {  0.7671565648};

double Ublox_Alt_U2AKalman_B[1][2] = 
    {  0.0027548935,  0.0464772253};

double Ublox_Alt_U2AKalman_C[1][1] = 
    {  0.9428769726};

double Ublox_Alt_U2AKalman_D[1][2] = 
    {  0.0000000000,  0.0571230274};

//////////////////////////////V2Alt Kalman Model////////////////////////////////
double Ublox_Alt_V2PKalman_A[2][2] = 
{ 
    {  0.9230182331, -0.0001599655}, 
    {  0.0183688118,  0.9986107190} 
};

double Ublox_Alt_V2PKalman_B[2][2] = 
{ 
    {  0.0192209134, -0.0000559187}, 
    {  0.0001947716, -0.0004856471} 
};

double Ublox_Alt_V2PKalman_C[1][2] = 
    { -1.7519512411, -2.8564111930};

double Ublox_Alt_V2PKalman_D[1][2] = 
    {  0.0000000000,  0.0014922351};

//////////////////////////////A2Alt Kalman Model////////////////////////////////
double Ublox_Alt_A2PKalman_A[3][3] = 
{ 
    {  1.0000000000, -0.1912534923,  0.0199998000}, 
    {  0.0200000000,  0.9106223881,  0.0001999987}, 
    {  0.0000000000,  0.0000000000,  0.9999800002} 
};

double Ublox_Alt_A2PKalman_B[3][2] = 
{ 
    {  0.0200000000,  0.1912534923}, 
    {  0.0002000000,  0.0893776119}, 
    {  0.0000000000,  0.0000000000} 
};

double Ublox_Alt_A2PKalman_C[2][3] = 
{ 
    {  1.0000000000, -0.1912534923,  0.0000000000}, 
    {  0.0000000000,  0.9144474579,  0.0000000000} 
};

double Ublox_Alt_A2PKalman_D[2][2] = 
{ 
    { -0.0000000000,  0.1912534923}, 
    { -0.0000000000,  0.0855525421} 
};

//////////////////////////////Reference Model////////////////////////////////
double Ublox_Alt_A_d[3][3] = 
{ 
    {  0.8136337901,  0.0000000000,  0.0000000000}, 
    {  0.0180723215,  1.0000000000,  0.0000000000}, 
    {  0.0001869310,  0.0200000000,  1.0000000000} 
};

double Ublox_Alt_B_d[3] = 
    {  0.0027548935,  0.0000284952,  0.0000001932};

double Ublox_Alt_F[3] = 
    {  0.0000000000, 164.0019997493, 144.3217597794};

double Ublox_Alt_Br= 144.3217597794; 

double Ublox_A_refmodel_Alt[3][3] = 
{ 
    {  0.8092481277, -0.4551616481, -0.3969262491}, 
    {  0.0180421022,  0.9953027094, -0.0041090924}, 
    {  0.0001867769,  0.0199681917,  0.9999721326} 
};

double Ublox_B_refmodel_Alt[3] = 
    {  0.3969262491,  0.0041090924,  0.0000278674};

double Ublox_C_refmodel_Alt[3] = 
    {  0.0000000000,  0.0000000000,  1.0000000000};

double Ublox_error_Alt[4]; 

double Ublox_Klp_Alt[4]= 
    { -43.7028499590, -361.3101585203, -555.1378769095, -214.5838305770};

double Ublox_Knp_Alt = -11.0365835614; 

double Ublox_K1_Alt[3]= 
    {  0.0000000000, -164.0019997493, -144.3217597794};

double Ublox_K2_Alt = 144.3217597794; 

double Ublox_S_Alt[4]= 
    {  2.9719704261, 20.7270678629, 30.7160839157,  9.7214789967};

