#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
// This header was generated by MATLAB. 
// 14-Mar-2017 11:35:49
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -2.5000000000, -2.4000000000,  0.0000000000,  0.0000000000}, 
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
    {  0.9677828667, -0.0045502291, -0.0061804219,  0.0000982496}, 
    {  0.0196762245,  0.9999542494, -0.0189450289, -0.0017461293}, 
    {  0.0012620411,  0.1248247084,  0.8574508801, -0.0017541106}, 
    {  0.0000085300,  0.0012759879,  0.0043302871,  0.9932315472} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.3015758080,  0.0061804219, -0.0000982496}, 
    {  0.0030322175,  0.0189450289,  0.0017461293}, 
    {  0.0001307381,  0.0177224389,  0.0017541106}, 
    {  0.0000006591,  0.0143937150,  0.0067684528} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0064746448,  0.0000933015}, 
    {  0.0000000000,  1.0000000000, -0.0188184933, -0.0017480451}, 
    {  0.0000000000,  0.0000000000,  0.9824431865, -0.0017551142}, 
    {  0.0000000000,  0.0000000000, -0.0140409138,  0.9932666396} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0064746448, -0.0000933015}, 
    {  0.0000000000,  0.0188184933,  0.0017480451}, 
    {  0.0000000000,  0.0175568135,  0.0017551142}, 
    {  0.0000000000,  0.0140409138,  0.0067333604} 
};

//////////////////////////////Plant Model////////////////////////////////
double Pos_A_d[4][4] = 
{ 
    {  0.9677828667, -0.0045502291,  0.0000000000,  0.0000000000}, 
    {  0.0196762245,  0.9999542494,  0.0000000000,  0.0000000000}, 
    {  0.0012620411,  0.1248247084,  0.8751733190,  0.0000000000}, 
    {  0.0000085300,  0.0012759879,  0.0187240021,  1.0000000000} 
};

double Pos_B_d[4] = 
    {  0.3015758080,  0.0030322175,  0.0001307381,  0.0000006591};

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
    {  0.0564341640,  0.1414991142,  0.0000000000,  0.0000000000};

double Pos_Br=  0.1565872909; 

double Perror[4]; 

double PKlp[5]= 
    { -0.1420437331, -0.3649342156, -0.0344185743, -0.3356271669, -0.0582804970};

double PKnp= -0.0196450887; 

double PK1[4]= 
    { -0.0564341640, -0.1414991142,  0.0000000000,  0.0000000000};

double PK2=  0.1565872909; 

double PSr[4]= 
    {  3.3211712797,  9.5553157687,  1.4021402105,  8.5757683054};

double PSi = 1.648150; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
