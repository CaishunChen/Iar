#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
// This header was generated by MATLAB. 
// 27-Sep-2016 12:45:49
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -4.8000000000, -8.2000000000,  0.0000000000,  0.0000000000}, 
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
    {  0.9857350209, -0.0025527181, -0.0137858167, -0.0000502588}, 
    {  0.0198570943,  0.9999744117, -0.0268164221, -0.0020102156}, 
    {  0.0012698473,  0.1248255793,  0.8506705838, -0.0019954976}, 
    {  0.0000085693,  0.0012759923,  0.0022892770,  0.9930580141} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.2578569242,  0.0137858167,  0.0000502588}, 
    {  0.0025847439,  0.0268164221,  0.0020102156}, 
    {  0.0001112784,  0.0245027353,  0.0019954976}, 
    {  0.0000005605,  0.0164347251,  0.0069419859} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0140540414, -0.0000561891}, 
    {  0.0000000000,  1.0000000000, -0.0265380287, -0.0020091513}, 
    {  0.0000000000,  0.0000000000,  0.9758079188, -0.0019934712}, 
    {  0.0000000000,  0.0000000000, -0.0159477698,  0.9930979040} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0140540414,  0.0000561891}, 
    {  0.0000000000,  0.0265380287,  0.0020091513}, 
    {  0.0000000000,  0.0241920812,  0.0019934712}, 
    {  0.0000000000,  0.0159477698,  0.0069020960} 
};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_d[4][4] = 
{ 
    {  0.9857350209, -0.0025527181,  0.0000000000,  0.0000000000}, 
    {  0.0198570943,  0.9999744117,  0.0000000000,  0.0000000000}, 
    {  0.0012698473,  0.1248255793,  0.8751733190,  0.0000000000}, 
    {  0.0000085693,  0.0012759923,  0.0187240021,  1.0000000000} 
};

double Pos_B_d[4] = 
    {  0.2578569242,  0.0025847439,  0.0001112784,  0.0000005605};

double Pos_F[4] = 
    {  0.3144172384,  0.6215673899,  0.0000000000,  0.0000000000};

double Pos_Br=  0.6314671362; 

double Perror[4]; 

double PKlp[5]= 
    { -0.2376011047, -0.9149803909,  0.1048164821, -0.3973221359, -0.0672047399};

double PKnp= -0.2125200476; 

double PK1[4]= 
    { -0.3144172384, -0.6215673899,  0.0000000000,  0.0000000000};

double PK2=  0.6314671362; 

double PSr[4]= 
    {  1.0870718536,  5.3107814471,  0.8846694829,  1.7641655938};

double PSi = 0.316228; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
#define N_STATE    4
