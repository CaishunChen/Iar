#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
// This header was generated by MATLAB. 
// 27-Aug-2016 11:05:34
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -4.5958053328, -6.0000000000,  0.0000000000,  0.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  6.6666666667, -6.6666666667,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.KalmanParam.Qk =    {  0.0500000000};
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
    {  0.9119781246, -0.0206706551, -0.0042709842,  0.0003344441}, 
    {  0.0191069875,  0.9997901197, -0.0224132485, -0.0016445044}, 
    {  0.0012373249,  0.1248175883,  0.8543359943, -0.0016841736}, 
    {  0.0000084048,  0.0012759519,  0.0047944719,  0.9932889439} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.5723547770,  0.0042709842, -0.0003344441}, 
    {  0.0058114273,  0.0224132485,  0.0016445044}, 
    {  0.0002517688,  0.0208373247,  0.0016841736}, 
    {  0.0000012730,  0.0139295302,  0.0067110561} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0051890807,  0.0003292995}, 
    {  0.0000000000,  1.0000000000, -0.0223187850, -0.0016511429}, 
    {  0.0000000000,  0.0000000000,  0.9793810817, -0.0016893675}, 
    {  0.0000000000,  0.0000000000, -0.0135149402,  0.9933226796} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0051890807, -0.0003292995}, 
    {  0.0000000000,  0.0223187850,  0.0016511429}, 
    {  0.0000000000,  0.0206189183,  0.0016893675}, 
    {  0.0000000000,  0.0135149402,  0.0066773204} 
};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_d[4][4] = 
{ 
    {  0.9119781246, -0.0206706551,  0.0000000000,  0.0000000000}, 
    {  0.0191069875,  0.9997901197,  0.0000000000,  0.0000000000}, 
    {  0.0012373249,  0.1248175883,  0.8751733190,  0.0000000000}, 
    {  0.0000084048,  0.0012759519,  0.0187240021,  1.0000000000} 
};

double Pos_B_d[4] = 
    {  0.5723547770,  0.0058114273,  0.0002517688,  0.0000012730};

double Pos_F[4] = 
    {  0.0000000000,  0.1641836037,  0.0000000000,  0.0000000000};

double Pos_Br=  0.2002987126; 

double Pos_A_refmodel[4][4] = 
{ 
    {  0.9110529587, -0.1146043444,  0.0000000000,  0.0000000000}, 
    {  0.0191007241,  0.9988361682,  0.0000000000,  0.0000000000}, 
    {  0.0012371198,  0.1247762570,  0.8751733190,  0.0000000000}, 
    {  0.0000084040,  0.0012757429,  0.0187240021,  1.0000000000} 
};

double Pos_B_refmodel[4]= 
    {  0.1146043444,  0.0011638318,  0.0000504240,  0.0000002550};

double Pos_C_refmodel[4]= 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

double Perror[4]; 

double PKlp[5]= 
    { -0.1156330994, -0.5178482727,  0.0606212792, -0.1770470940, -0.0301601843};

double PKnp= -0.0953748771; 

double PK1[4]= 
    {  0.0000000000, -0.1641836037,  0.0000000000,  0.0000000000};

double PK2=  0.2002987126; 

double PSr[4]= 
    {  1.0500601350,  5.3129103862,  0.9975684113,  1.7509191555};

double PSi = 0.316228; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
//#define N_STATE    4