#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
// This header was generated by MATLAB. 
// 27-Sep-2016 13:30:51
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -0.7148050531, -0.4000000000,  0.0000000000,  0.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  6.6666666667, -6.6666666667,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.KalmanParam.Qk =    {  0.0200000000};
CtrData.KalmanParam.Rk = { 
    {  1.0000000000,  0.0000000000}, 
    {  0.0000000000,  8.0000000000} 
};
CtrData.SMC.Q= { 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000, 15.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.1000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.1000000000} 
};
CtrData.SMC.R =     {  1.0000000000};
CtrData.SMC.AA = { 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000} 
};
CtrData.SMC.rou =     {  3.0000000000};
CtrData.SMC.fai =     { -3.0000000000};
*/
///////////////////////////////////////////////////////////////////////////////

#define MRSMC_Control  
#define K_Pos 4
#define Ky_Pos 4
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[4][4] = 
{ 
    {  0.9857802980, -0.0025365292, -0.0138173291, -0.0000517133}, 
    {  0.0198575487,  0.9999745742, -0.0268419661, -0.0020115618}, 
    {  0.0012698669,  0.1248255863,  0.8506491324, -0.0019966416}, 
    {  0.0000085694,  0.0012759923,  0.0022799163,  0.9930573817} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.2576503174,  0.0138173291,  0.0000517133}, 
    {  0.0025826532,  0.0268419661,  0.0020115618}, 
    {  0.0001111880,  0.0245241866,  0.0019966416}, 
    {  0.0000005600,  0.0164440858,  0.0069426183} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0140849912, -0.0000576324}, 
    {  0.0000000000,  1.0000000000, -0.0265629481, -0.0020104685}, 
    {  0.0000000000,  0.0000000000,  0.9757870075, -0.0019945884}, 
    {  0.0000000000,  0.0000000000, -0.0159567069,  0.9930972942} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0140849912,  0.0000576324}, 
    {  0.0000000000,  0.0265629481,  0.0020104685}, 
    {  0.0000000000,  0.0242129925,  0.0019945884}, 
    {  0.0000000000,  0.0159567069,  0.0069027058} 
};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_d[4][4] = 
{ 
    {  0.9857802980, -0.0025365292,  0.0000000000,  0.0000000000}, 
    {  0.0198575487,  0.9999745742,  0.0000000000,  0.0000000000}, 
    {  0.0012698669,  0.1248255863,  0.8751733190,  0.0000000000}, 
    {  0.0000085694,  0.0012759923,  0.0187240021,  1.0000000000} 
};

double Pos_B_d[4] = 
    {  0.2576503174,  0.0025826532,  0.0001111880,  0.0000005600};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[4][4] = 
{ 
    {  0.9857263624, -0.0079428753,  0.0000000000,  0.0000000000}, 
    {  0.0198571883,  0.9999203809,  0.0000000000,  0.0000000000}, 
    {  0.0012698551,  0.1248232532,  0.8751733190,  0.0000000000}, 
    {  0.0000085694,  0.0012759806,  0.0187240021,  1.0000000000} 
};

double Pos_B_refmodel[4] = 
    {  0.0079428753,  0.0000796191,  0.0000034278,  0.0000000173};

double Pos_C_refmodel[4] = 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

double Pos_F[4] = 
    {  0.0000000000,  0.0209838293,  0.0000000000,  0.0000000000};

double Pos_Br=  0.0308286812; 

double Perror[4]; 

double PKlp[5]= 
    { -0.5001749783, -1.2648498946,  0.0811981641, -0.3075787034, -0.0520696543};

double PKnp= -0.1646587044; 

double PK1[4]= 
    {  0.0000000000, -0.0209838293,  0.0000000000,  0.0000000000};

double PK2=  0.0308286812; 

double PSr[4]= 
    {  1.4042082350,  5.9040506847,  0.8841704303,  1.7625679310};

double PSi = 0.316228; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
//#define N_STATE    4