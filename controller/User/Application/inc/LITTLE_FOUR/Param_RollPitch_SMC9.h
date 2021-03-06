#define INCLUDED_PARAM_ROLLPITCH_SMC_H

// Header for FRAME TYPE. 
// This header was generated by MATLAB. 
// 06-Feb-2017 13:42:59
//
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -23.0000000000, -248.0000000000, -1030.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.KalmanParam.Qk =    { 100000.0000000000};
CtrData.KalmanParam.Rk = { 
    {  1.0000000000,  0.0000000000}, 
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
    {  0.5799407298, -5.1702282742,  0.0907502250}, 
    {  0.0156656463,  0.7998671229, -0.0047084110}, 
    {  0.0001706715,  0.0102017288,  0.9847440673} 
};

double Kalman_B[3][3] = 
{ 
    {  0.0213426394, -0.4555904532, -0.0907502250}, 
    {  0.0002325203,  0.1388416195,  0.0047084110}, 
    {  0.0000016150,  0.0093725650,  0.0152559327} 
};

double Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.5587599448,  0.0928012789}, 
    {  0.0000000000,  0.8614178414, -0.0065645527}, 
    {  0.0000000000, -0.0065645527,  0.9848567252} 
};

double Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.5587599448, -0.0928012789}, 
    {  0.0000000000,  0.1385821586,  0.0065645527}, 
    {  0.0000000000,  0.0065645527,  0.0151432748} 
};

//////////////////////////////Plant  Model////////////////////////////////
double Ati_A_d[3][3] = 
{ 
    {  0.5799407298, -5.6258187274,  0.0000000000}, 
    {  0.0156656463,  0.9387087424,  0.0000000000}, 
    {  0.0001706715,  0.0195742938,  1.0000000000} 
};

double Ati_B_d[3] = 
    {  0.0213426394,  0.0002325203,  0.0000016150};

double Ati_F[3] = 
    {  0.0722427805, -81.5615359289, 756.0271873947};

double Ati_Br= 756.0271873947; 

//////////////////////////////Reference Model////////////////////////////////
double A_refmodel[3][3] = 
{ 
    {  0.5937891450, -4.0854264590, -16.2355125002}, 
    {  0.0157626335,  0.9563297154, -0.1762933522}, 
    {  0.0001711586,  0.0196992812,  0.9987770468} 
};

double B_refmodel[3] = 
    { 16.2355125002,  0.1762933522,  0.0012229532};

double C_refmodel[3] = 
    {  0.0000000000,  0.0000000000,  1.0000000000};

//////////////////////////////Controller Param////////////////////////////////
double error[3]; 

double Klp[3]= 
    { -4.4579668845, -17.3951941952, 88.8484256141};

double Knp= -129.0557609210; 

double K1[3]= 
    { -0.0722427805, 81.5615359289, -756.0271873947};

double K2= 756.0271873947; 

double S[3]= 
    {  0.0853127717,  1.2006464456,  5.1696937589};

double X_Roll_Ref[3] , X_Roll_smc[3] , theta_error_roll; 
double X_Pitch_Ref[3] , X_Pitch_smc[3] , theta_error_pitch; 
