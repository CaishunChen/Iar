#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
// This header was generated by MATLAB. 
// 10-Feb-2017 17:21:14
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -5.0000000000, -4.0000000000,  0.0000000000,  0.0000000000}, 
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
    {  0.9560033749, -0.0071615234, -0.1092622426,  0.0001045552}, 
    {  0.0195569787,  0.9999278477, -0.0947798589, -0.0022475821}, 
    {  0.0012568826,  0.1248235669,  0.8003538460, -0.0022433947}, 
    {  0.0000085039,  0.0012759821, -0.0006868240,  0.9929451442} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.3392142353,  0.1092622426, -0.0001045552}, 
    {  0.0034175789,  0.0947798589,  0.0022475821}, 
    {  0.0001474997,  0.0748194731,  0.0022433947}, 
    {  0.0000007440,  0.0194108261,  0.0070548558} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.1149838558,  0.0000925153}, 
    {  0.0000000000,  1.0000000000, -0.0925377989, -0.0022495537}, 
    {  0.0000000000,  0.0000000000,  0.9278725112, -0.0022426571}, 
    {  0.0000000000,  0.0000000000, -0.0179412565,  0.9929900053} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.1149838558, -0.0000925153}, 
    {  0.0000000000,  0.0925377989,  0.0022495537}, 
    {  0.0000000000,  0.0721274888,  0.0022426571}, 
    {  0.0000000000,  0.0179412565,  0.0070099947} 
};

//////////////////////////////Plant Model////////////////////////////////
double Pos_A_d[4][4] = 
{ 
    {  0.9560033749, -0.0071615234,  0.0000000000,  0.0000000000}, 
    {  0.0195569787,  0.9999278477,  0.0000000000,  0.0000000000}, 
    {  0.0012568826,  0.1248235669,  0.8751733190,  0.0000000000}, 
    {  0.0000085039,  0.0012759821,  0.0187240021,  1.0000000000} 
};

double Pos_B_d[4] = 
    {  0.3392142353,  0.0034175789,  0.0001474997,  0.0000007440};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[4][4] = 
{ 
    {  0.9040889041, -0.0761097692,  0.0000000000,  0.0000000000}, 
    {  0.0190274423,  0.9992261156,  0.0000000000,  0.0000000000}, 
    {  0.0012338907,  0.1247931310,  0.8751733190,  0.0000000000}, 
    {  0.0000083875,  0.0012758281,  0.0187240021,  1.0000000000} 
};

double Pos_B_refmodel[4] = 
    {  0.0761097692,  0.0007738844,  0.0000335500,  0.0000001697};

double Pos_C_refmodel[4] = 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

//////////////////////////////Controller Param////////////////////////////////
double Pos_F[4] = 
    {  0.1587799542,  0.2095029749,  0.0000000000,  0.0000000000};

double Pos_Br=  0.2306150707; 

double Perror[4]; 

double PKlp[5]= 
    {  0.0373746785, -0.1484255687, -0.0162422995, -0.1851462040, -0.0104628370};

double PKnp= -0.0091906655; 

double PK1[4]= 
    { -0.1587799542, -0.2095029749,  0.0000000000,  0.0000000000};

double PK2=  0.2306150707; 

double PSr[4]= 
    {  3.1365393390,  8.0036303293,  0.9321633935,  5.4201587343};

double PSi = 0.316228; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
