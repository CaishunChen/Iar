#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
// This header was generated by MATLAB. 
// 13-Feb-2017 16:27:56
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -1.7706754608, -1.4000000000,  0.0000000000,  0.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  6.6666666667, -6.6666666667,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.KalmanParam.Qk =    {  0.0400000000};
CtrData.KalmanParam.Rk = { 
    {  0.4000000000,  0.0000000000}, 
    {  0.0000000000,  0.1000000000} 
};
CtrData.SMC.Q= { 
    { 10.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000} 
};
CtrData.SMC.R =     {  1.0000000000};
CtrData.SMC.AA = { 
    {  2.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.5000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.5000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.1000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.1000000000} 
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
    {  0.9651449523, -0.0061622834, -0.0210024848, -0.0080259710}, 
    {  0.0196495977,  0.9999380128, -0.0307996388, -0.0404951995}, 
    {  0.0012608908,  0.1248240077,  0.8483616750, -0.0386081635}, 
    {  0.0000085242,  0.0012759844,  0.0086055216,  0.9654988653} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.2935892784,  0.0210024848,  0.0080259710}, 
    {  0.0029532517,  0.0307996388,  0.0404951995}, 
    {  0.0001273616,  0.0268116440,  0.0386081635}, 
    {  0.0000006422,  0.0101184805,  0.0345011347} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0219548722, -0.0085733145}, 
    {  0.0000000000,  1.0000000000, -0.0303701170, -0.0403292372}, 
    {  0.0000000000,  0.0000000000,  0.9737274422, -0.0383504566}, 
    {  0.0000000000,  0.0000000000, -0.0095876142,  0.9662684719} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0219548722,  0.0085733145}, 
    {  0.0000000000,  0.0303701170,  0.0403292372}, 
    {  0.0000000000,  0.0262725578,  0.0383504566}, 
    {  0.0000000000,  0.0095876142,  0.0337315281} 
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
    { -0.1658671919, -0.3046368155, -0.0256271200, -0.2157878871, -0.0566281107};

double PKnp= -0.0401674598; 

double PK1[4]= 
    {  0.0000000000, -0.0727109435,  0.0000000000,  0.0000000000};

double PK2=  0.0937004136; 

double PSr[4]= 
    {  3.3324918816,  7.4945620680,  0.7267054640,  4.6673061376};

double PSi = 1.409801; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 