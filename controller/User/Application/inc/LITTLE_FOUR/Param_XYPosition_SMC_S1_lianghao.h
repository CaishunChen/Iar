#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
// This header was generated by MATLAB. 
// 20-Mar-2017 13:46:14
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -1.4124237335, -0.8000000000,  0.0000000000,  0.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  6.6666666667, -6.6666666667,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.KalmanParam.Qk =    {  0.0002000000};
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
    {  0.0000000000,  0.4000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.5000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.4000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.3000000000} 
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
    {  0.9721172491, -0.0029673422, -0.0049545352,  0.0000447612}, 
    {  0.0197199587,  0.9999701867, -0.0162615282, -0.0017167231}, 
    {  0.0012639300,  0.1248253960,  0.8598498570, -0.0017181851}, 
    {  0.0000085395,  0.0012759914,  0.0046721307,  0.9932661465} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.2243220508,  0.0049545352, -0.0000447612}, 
    {  0.0022537930,  0.0162615282,  0.0017167231}, 
    {  0.0000971401,  0.0153234621,  0.0017181851}, 
    {  0.0000004896,  0.0140518714,  0.0067338535} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0051459729,  0.0000408022}, 
    {  0.0000000000,  1.0000000000, -0.0161605316, -0.0017175789}, 
    {  0.0000000000,  0.0000000000,  0.9848033380, -0.0017183331}, 
    {  0.0000000000,  0.0000000000, -0.0137466644,  0.9933005119} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0051459729, -0.0000408022}, 
    {  0.0000000000,  0.0161605316,  0.0017175789}, 
    {  0.0000000000,  0.0151966620,  0.0017183331}, 
    {  0.0000000000,  0.0137466644,  0.0066994881} 
};

//////////////////////////////Plant Model////////////////////////////////
double Pos_A_d[4][4] = 
{ 
    {  0.9721172491, -0.0029673422,  0.0000000000,  0.0000000000}, 
    {  0.0197199587,  0.9999701867,  0.0000000000,  0.0000000000}, 
    {  0.0012639300,  0.1248253960,  0.8751733190,  0.0000000000}, 
    {  0.0000085395,  0.0012759914,  0.0187240021,  1.0000000000} 
};

double Pos_B_d[4] = 
    {  0.2243220508,  0.0022537930,  0.0000971401,  0.0000004896};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[4][4] = 
{ 
    {  0.9719897686, -0.0157752838,  0.0000000000,  0.0000000000}, 
    {  0.0197191048,  0.9998415002,  0.0000000000,  0.0000000000}, 
    {  0.0012639022,  0.1248198494,  0.8751733190,  0.0000000000}, 
    {  0.0000085394,  0.0012759634,  0.0187240021,  1.0000000000} 
};

double Pos_B_refmodel[4] = 
    {  0.0157752838,  0.0001584998,  0.0000068315,  0.0000000344};

double Pos_C_refmodel[4] = 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

//////////////////////////////Controller Param////////////////////////////////
double Pos_F[4] = 
    {  0.0000000000,  0.0570992673,  0.0000000000,  0.0000000000};

double Pos_Br=  0.0703273124; 

double Perror[4]; 

double PKlp[5]= 
    { -0.2649737644, -0.5687421673, -0.0582569428, -0.4588018840, -0.1085978429};

double PKnp= -0.0254261402; 

double PK1[4]= 
    {  0.0000000000, -0.0570992673,  0.0000000000,  0.0000000000};

double PK2=  0.0703273124; 

double PSr[4]= 
    {  3.4574316000,  9.0812945461,  1.3182028172,  8.7064761467};

double PSi = 2.372839; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
