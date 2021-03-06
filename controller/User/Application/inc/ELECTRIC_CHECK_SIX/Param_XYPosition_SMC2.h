#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 18-May-2015 06:51:46

#define MRSMC_Control  
#define K_Pos 3
#define Ky_Pos 3
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[3][3] = 
{ 
    {  0.9834394874, -0.0048592995, -0.0010137382}, 
    {  0.0198340008,  0.9924448968, -0.0091605521}, 
    {  0.0001988920,  0.0133212258,  0.9824214820} 
};

double Pos_Kalman_B[3][3] = 
{ 
    {  0.1563241316,  0.0028714039,  0.0010137382}, 
    {  0.0015675921,  0.0075351689,  0.0091605521}, 
    {  0.0000104651,  0.0066786411,  0.0175785180} 
};

double Pos_Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0029348706, -0.0010492841}, 
    {  0.0000000000,  0.9925228922, -0.0091399227}, 
    {  0.0000000000, -0.0065285162,  0.9826044880} 
};

double Pos_Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0029348706,  0.0010492841}, 
    {  0.0000000000,  0.0074771078,  0.0091399227}, 
    {  0.0000000000,  0.0065285162,  0.0173955120} 
};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[3][3] = 
{ 
    {  0.9541588952, -0.0312625132,  0.0000000000}, 
    {  0.0195390707,  0.9996849300,  0.0000000000}, 
    {  0.0001969187,  0.0199978914,  1.0000000000} 
};

double Pos_B_refmodel[3] = 
    {  0.0312625132,  0.0003150700,  0.0000021086};

double Pos_C_refmodel[3] = 
    {  0.0000000000,  1.0000000000,  0.0000000000};

double Perror[3]; 

double PKlp[4]= 
    { -0.5486320264, -1.2844696516, -0.7249634046, -0.2240453557};

double PKnp= -0.0841889696; 

double PK1[3]= 
    { -0.1898148620, -0.1902873560,  0.0000000000};

double PK2=  0.2030038544; 

double PSr[3]= 
    {  1.5070550169,  7.0140016452,  3.6402674221};

double PSi = 1.330610; 

double X_PosX_Ref[3] , X_PosX_smc[3]; 
double X_PosY_Ref[3] , X_PosY_smc[3]; 
#define N_STATE    3
