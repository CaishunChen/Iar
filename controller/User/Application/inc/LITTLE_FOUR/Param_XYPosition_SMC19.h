#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
// This header was generated by MATLAB. 
// 08-Feb-2017 15:08:06
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
    {  1.0000000000,  0.0000000000}, 
    {  0.0000000000,  8.0000000000} 
};
CtrData.SMC.Q= { 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000}, 
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
    {  0.9651449523, -0.0061622834, -0.0046792124,  0.0001568917}, 
    {  0.0196495977,  0.9999380128, -0.0168582824, -0.0016333114}, 
    {  0.0012608908,  0.1248240077,  0.8592987385, -0.0016501362}, 
    {  0.0000085242,  0.0012759844,  0.0051892844,  0.9933291799} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.2935892784,  0.0046792124, -0.0001568917}, 
    {  0.0029532517,  0.0168582824,  0.0016333114}, 
    {  0.0001273616,  0.0158745805,  0.0016501362}, 
    {  0.0000006422,  0.0135347177,  0.0066708201} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0049552187,  0.0001521095}, 
    {  0.0000000000,  1.0000000000, -0.0167619533, -0.0016364018}, 
    {  0.0000000000,  0.0000000000,  0.9842590742, -0.0016523193}, 
    {  0.0000000000,  0.0000000000, -0.0132185544,  0.9933622046} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0049552187, -0.0001521095}, 
    {  0.0000000000,  0.0167619533,  0.0016364018}, 
    {  0.0000000000,  0.0157409258,  0.0016523193}, 
    {  0.0000000000,  0.0132185544,  0.0066377954} 
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
    { -0.2235334239, -0.4240575908, -0.0018388983, -0.1637292522, -0.0362164560};

double PKnp= -0.1145264898; 

double PK1[4]= 
    {  0.0000000000, -0.0727109435,  0.0000000000,  0.0000000000};

double PK2=  0.0937004136; 

double PSr[4]= 
    {  1.1687927744,  3.6355779822,  0.2655840230,  1.2715051727};

double PSi = 0.316228; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
