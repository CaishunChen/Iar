#define INCLUDED_PARAM_ROLLPITCH_SMC_H

// Header for FRAME TYPE. 
// This header was generated by MATLAB. 
// 07-Apr-2017 10:30:38
//
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -23.5000000000, -235.0000000000, -900.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.KalmanParam.Qk =    { 500000.0000000000};
CtrData.KalmanParam.Rk = { 
    { 50.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000} 
};
CtrData.SMC.Q= { 
    {  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000, 100.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000, 1000.0000000000} 
};
CtrData.SMC.R =     {  1.0000000000};
CtrData.SMC.AA = { 
    {  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  2.5000000000} 
};
CtrData.SMC.rou =     { 15.0000000000};
CtrData.SMC.fai =     { -15.0000000000};
*/
///////////////////////////////////////////////////////////////////////////

#define MRSMC_Control  
#define K_Roll 3
#define Ky_Roll 3
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A[3][3] = 
{ 
    {  0.5848525888, -3.4710801928,  0.7158693822}, 
    {  0.0156329206,  0.9407030979, -0.0722422389}, 
    {  0.0001701536,  0.0175778369,  0.9414974334} 
};

double Kalman_B[3][3] = 
{ 
    {  0.0178811035, -0.0635219383, -0.7158693822}, 
    {  0.0001946236,  0.0208251752,  0.0722422389}, 
    {  0.0000013518,  0.0021549409,  0.0585025666} 
};

double Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0202888561,  0.7010608039}, 
    {  0.0000000000,  0.9786714528, -0.0865308583}, 
    {  0.0000000000, -0.0017306172,  0.9430856395} 
};

double Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0202888561, -0.7010608039}, 
    {  0.0000000000,  0.0213285472,  0.0865308583}, 
    {  0.0000000000,  0.0017306172,  0.0569143605} 
};

//////////////////////////////Plant  Model////////////////////////////////
double Ati_A_d[3][3] = 
{ 
    {  0.5848525888, -3.5346021311,  0.0000000000}, 
    {  0.0156329206,  0.9615282731,  0.0000000000}, 
    {  0.0001701536,  0.0197327778,  1.0000000000} 
};

double Ati_B_d[3] = 
    {  0.0178811035,  0.0001946236,  0.0000013518};

double Ati_F[3] = 
    { -0.5202167817,  7.7810752177, 786.8434151535};

double Ati_Br= 786.8434151535; 

//////////////////////////////Reference Model////////////////////////////////
double A_refmodel[3][3] = 
{ 
    {  0.5897623213, -3.8441872824, -14.1340239689}, 
    {  0.0157044711,  0.9588173916, -0.1536365794}, 
    {  0.0001707073,  0.0197160929,  0.9989336096} 
};

double B_refmodel[3] = 
    { 14.1340239689,  0.1536365794,  0.0010663904};

double C_refmodel[3] = 
    {  0.0000000000,  0.0000000000,  1.0000000000};

//////////////////////////////Controller Param////////////////////////////////
double error[3]; 

double Klp[3]= 
    { -5.6349891276, -50.2907319056, -109.4061285419};

double Knp= -174.2185186734; 

double K1[3]= 
    {  0.5202167817, -7.7810752177, -786.8434151535};

double K2= 786.8434151535; 

double S[3]= 
    {  0.0752736105,  1.1249913168,  5.1443988304};

double X_Roll_Ref[3] , X_Roll_smc[3] , theta_error_roll; 
double X_Pitch_Ref[3] , X_Pitch_smc[3] , theta_error_pitch; 