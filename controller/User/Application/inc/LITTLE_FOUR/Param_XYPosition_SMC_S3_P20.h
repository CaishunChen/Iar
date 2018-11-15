#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
// This header was generated by MATLAB. 
// 22-Feb-2017 13:07:42
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -2.0182175873, -1.5000000000,  0.0000000000,  0.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  6.6666666667, -6.6666666667,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.KalmanParam.Qk =    {  0.0100000000};
CtrData.KalmanParam.Rk = { 
    {  0.0100000000,  0.0000000000}, 
    {  0.0000000000,  0.0800000000} 
};
CtrData.SMC.Q= { 
    { 10.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.1000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  3.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.5000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.1000000000} 
};
CtrData.SMC.R =     {  1.0000000000};
CtrData.SMC.AA = { 
    {  2.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.5000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.5000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000} 
};
CtrData.SMC.rou =     {  0.5000000000};
CtrData.SMC.fai =     { -1.8000000000};
*/
///////////////////////////////////////////////////////////////////////////////

#define MRSMC_Control  
#define K_Pos 4
#define Ky_Pos 4
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[4][4] = 
{ 
    {  0.9603745128, -0.0065363842, -0.1261571710,  0.0000817147}, 
    {  0.0196012973,  0.9999341957, -0.1014475654, -0.0022636101}, 
    {  0.0012588012,  0.1248238420,  0.7960577699, -0.0022561607}, 
    {  0.0000085136,  0.0012759835, -0.0008682547,  0.9929420546} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.3487674370,  0.1261571710, -0.0000817147}, 
    {  0.0035111758,  0.1014475654,  0.0022636101}, 
    {  0.0001514833,  0.0791155491,  0.0022561607}, 
    {  0.0000007640,  0.0195922569,  0.0070579454} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.1320353628,  0.0000696696}, 
    {  0.0000000000,  1.0000000000, -0.0988660068, -0.0022651248}, 
    {  0.0000000000,  0.0000000000,  0.9238910664, -0.0022549897}, 
    {  0.0000000000,  0.0000000000, -0.0180399175,  0.9929871667} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.1320353628, -0.0000696696}, 
    {  0.0000000000,  0.0988660068,  0.0022651248}, 
    {  0.0000000000,  0.0761089336,  0.0022549897}, 
    {  0.0000000000,  0.0180399175,  0.0070128333} 
};

//////////////////////////////Plant Model////////////////////////////////
double Pos_A_d[4][4] = 
{ 
    {  0.9603745128, -0.0065363842,  0.0000000000,  0.0000000000}, 
    {  0.0196012973,  0.9999341957,  0.0000000000,  0.0000000000}, 
    {  0.0012588012,  0.1248238420,  0.8751733190,  0.0000000000}, 
    {  0.0000085136,  0.0012759835,  0.0187240021,  1.0000000000} 
};

double Pos_B_d[4] = 
    {  0.3487674370,  0.0035111758,  0.0001514833,  0.0000007640};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[4][4] = 
{ 
    {  0.9601474042, -0.0293996595,  0.0000000000,  0.0000000000}, 
    {  0.0195997730,  0.9997040108,  0.0000000000,  0.0000000000}, 
    {  0.0012587515,  0.1248139108,  0.8751733190,  0.0000000000}, 
    {  0.0000085134,  0.0012759335,  0.0187240021,  1.0000000000} 
};

double Pos_B_refmodel[4] = 
    {  0.0293996595,  0.0002959892,  0.0000127701,  0.0000000644};

double Pos_C_refmodel[4] = 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

//////////////////////////////Controller Param////////////////////////////////
double Pos_F[4] = 
    {  0.0000000000,  0.0655610569,  0.0000000000,  0.0000000000};

double Pos_Br=  0.0843024401; 

double Perror[4]; 

double PKlp[5]= 
    { -0.1262465618, -0.2707019362, -0.0150047074, -0.1712597324, -0.0096860217};

double PKnp= -0.0085083028; 

double PK1[4]= 
    {  0.0000000000, -0.0655610569,  0.0000000000,  0.0000000000};

double PK2=  0.0843024401; 

double PSr[4]= 
    {  3.3027519067,  8.1397399770,  0.9316050205,  5.4155796339};

double PSi = 0.316228; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 