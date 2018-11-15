#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
// This header was generated by MATLAB. 
// 23-Feb-2017 12:50:23
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -2.5000000000, -2.4000000000,  0.0000000000,  0.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  6.6666666667, -6.6666666667,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.KalmanParam.Qk =    {  0.0005000000};
CtrData.KalmanParam.Rk = { 
    {  0.0100000000,  0.0000000000}, 
    {  0.0000000000,  0.0800000000} 
};
CtrData.SMC.Q= { 
    { 10.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.5000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  3.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.5000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.5000000000} 
};
CtrData.SMC.R =     {  1.0000000000};
CtrData.SMC.AA = { 
    {  2.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.5000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.5000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.2000000000} 
};
CtrData.SMC.rou =     {  1.0000000000};
CtrData.SMC.fai =     { -1.8000000000};
*/
///////////////////////////////////////////////////////////////////////////////

#define MRSMC_Control  
#define K_Pos 4
#define Ky_Pos 4
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[4][4] = 
{ 
    {  0.9655674360, -0.0061871261, -0.0110663728,  0.0001394245}, 
    {  0.0196538735,  0.9999377674, -0.0263492907, -0.0018710628}, 
    {  0.0012610758,  0.1248239972,  0.8510232487, -0.0018822580}, 
    {  0.0000085251,  0.0012759843,  0.0031746568,  0.9931268733} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.2893085531,  0.0110663728, -0.0001394245}, 
    {  0.0029099804,  0.0263492907,  0.0018710628}, 
    {  0.0001254910,  0.0241500704,  0.0018822580}, 
    {  0.0000006327,  0.0155493453,  0.0068731267} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0116283906,  0.0001323897}, 
    {  0.0000000000,  1.0000000000, -0.0261223735, -0.0018737814}, 
    {  0.0000000000,  0.0000000000,  0.9761479166, -0.0018836635}, 
    {  0.0000000000,  0.0000000000, -0.0150693080,  0.9931645328} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0116283906, -0.0001323897}, 
    {  0.0000000000,  0.0261223735,  0.0018737814}, 
    {  0.0000000000,  0.0238520834,  0.0018836635}, 
    {  0.0000000000,  0.0150693080,  0.0068354672} 
};

//////////////////////////////Plant Model////////////////////////////////
double Pos_A_d[4][4] = 
{ 
    {  0.9655674360, -0.0061871261,  0.0000000000,  0.0000000000}, 
    {  0.0196538735,  0.9999377674,  0.0000000000,  0.0000000000}, 
    {  0.0012610758,  0.1248239972,  0.8751733190,  0.0000000000}, 
    {  0.0000085251,  0.0012759843,  0.0187240021,  1.0000000000} 
};

double Pos_B_d[4] = 
    {  0.2893085531,  0.0029099804,  0.0001254910,  0.0000006327};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[4][4] = 
{ 
    {  0.9507651657, -0.0468122620,  0.0000000000,  0.0000000000}, 
    {  0.0195051092,  0.9995279386,  0.0000000000,  0.0000000000}, 
    {  0.0012546631,  0.1248062983,  0.8751733190,  0.0000000000}, 
    {  0.0000084928,  0.0012758950,  0.0187240021,  1.0000000000} 
};

double Pos_B_refmodel[4] = 
    {  0.0468122620,  0.0004720614,  0.0000203827,  0.0000001028};

double Pos_C_refmodel[4] = 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

//////////////////////////////Controller Param////////////////////////////////
double Pos_F[4] = 
    {  0.0510332381,  0.1416555775,  0.0000000000,  0.0000000000};

double Pos_Br=  0.1630414856; 

double Perror[4]; 

double PKlp[5]= 
    { -0.1476235395, -0.3791870168, -0.0358141800, -0.3489739893, -0.0605764399};

double PKnp= -0.0204153337; 

double PK1[4]= 
    { -0.0510332381, -0.1416555775,  0.0000000000,  0.0000000000};

double PK2=  0.1630414856; 

double PSr[4]= 
    {  3.3275945092,  9.5603289643,  1.4026908946,  8.5807074703};

double PSi = 1.648446; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
