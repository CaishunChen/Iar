#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
// This header was generated by MATLAB. 
// 09-Feb-2017 10:52:14
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -1.7706754608, -1.4000000000,  0.0000000000,  0.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  6.6666666667, -6.6666666667,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.KalmanParam.Qk =    {  0.0200000000};
CtrData.KalmanParam.Rk = { 
    {  0.5000000000,  0.0000000000}, 
    {  0.0000000000,  5.0000000000} 
};
CtrData.SMC.Q= { 
    { 10.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.1000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.1000000000} 
};
CtrData.SMC.R =     {  1.0000000000};
CtrData.SMC.AA = { 
    {  2.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.5000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000} 
};
CtrData.SMC.rou =     {  2.0000000000};
CtrData.SMC.fai =     { -2.0000000000};
*/
///////////////////////////////////////////////////////////////////////////////

#define MRSMC_Control  
#define K_Pos 4
#define Ky_Pos 4
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[4][4] = 
{ 
    {  0.9651449523, -0.0061622834, -0.0091026120,  0.0001272465}, 
    {  0.0196495977,  0.9999380128, -0.0239228956, -0.0014832277}, 
    {  0.0012608908,  0.1248240077,  0.8530844500, -0.0014954909}, 
    {  0.0000085242,  0.0012759844,  0.0033140500,  0.9938618422} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.2935892784,  0.0091026120, -0.0001272465}, 
    {  0.0029532517,  0.0239228956,  0.0014832277}, 
    {  0.0001273616,  0.0220888690,  0.0014954909}, 
    {  0.0000006422,  0.0154099521,  0.0061381578} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0095828926,  0.0001223557}, 
    {  0.0000000000,  1.0000000000, -0.0237360670, -0.0014857240}, 
    {  0.0000000000,  0.0000000000,  0.9781598060, -0.0014970648}, 
    {  0.0000000000,  0.0000000000, -0.0149706477,  0.9938917679} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0095828926, -0.0001223557}, 
    {  0.0000000000,  0.0237360670,  0.0014857240}, 
    {  0.0000000000,  0.0218401940,  0.0014970648}, 
    {  0.0000000000,  0.0149706477,  0.0061082321} 
};

//////////////////////////////Plant Model////////////////////////////////
double Pos_A_d[4][4] = 
{ 
    {  0.9651449523, -0.0061622834,  0.0000000000,  0.0000000000}, 
    {  0.0196495977,  0.9999380128,  0.0000000000,  0.0000000000}, 
    {  0.0012608908,  0.1248240077,  0.8751733190,  0.0000000000}, 
    {  0.0000085242,  0.0012759844,  0.0187240021,  1.0000000000} 
};

double Pos_B_d[4] = 
    {  0.2935892784,  0.0029532517,  0.0001273616,  0.0000006422};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[4][4] = 
{ 
    {  0.9649327484, -0.0275074445,  0.0000000000,  0.0000000000}, 
    {  0.0196481746,  0.9997232891,  0.0000000000,  0.0000000000}, 
    {  0.0012608445,  0.1248147474,  0.8751733190,  0.0000000000}, 
    {  0.0000085240,  0.0012759377,  0.0187240021,  1.0000000000} 
};

double Pos_B_refmodel[4] = 
    {  0.0275074445,  0.0002767109,  0.0000119336,  0.0000000602};

double Pos_C_refmodel[4] = 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

//////////////////////////////Controller Param////////////////////////////////
double Pos_F[4] = 
    {  0.0000000000,  0.0727109435,  0.0000000000,  0.0000000000};

double Pos_Br=  0.0937004136; 

double Perror[4]; 

double PKlp[5]= 
    { -0.2250989292, -0.4550775882, -0.0277746337, -0.2427704813, -0.0124668098};

double PKnp= -0.0394235142; 

double PK1[4]= 
    {  0.0000000000, -0.0727109435,  0.0000000000,  0.0000000000};

double PK2=  0.0937004136; 

double PSr[4]= 
    {  3.3953780223, 10.6408828234,  0.9837555761,  5.9998983116};

double PSi = 0.316228; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
