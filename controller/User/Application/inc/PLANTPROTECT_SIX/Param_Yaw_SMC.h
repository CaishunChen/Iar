#define INCLUDED_PARAM_YAW_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 28-Apr-2015 14:29:41

#define K_Yaw 2
#define Ky_Yaw 2
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_AYaw[2][2] = 
{ 
    {  0.9590065791, -0.0170985791}, 
    {  0.0194029230,  0.9731908812} 
};

double Kalman_BYaw[2][3] = 
{ 
    { -0.0000725262,  0.0005933576,  0.0170985791}, 
    { -0.0000007302,  0.0001902997,  0.0268091188} 
};

double Kalman_CYaw[2][2] = 
{ 
    {  0.9993816615, -0.0178184454}, 
    { -0.0001781845,  0.9735400020} 
};

double Kalman_DYaw[2][3] = 
{ 
    {  0.0000000000,  0.0006183385,  0.0178184454}, 
    {  0.0000000000,  0.0001781845,  0.0264599980} 
};

//////////////////////////////Reference Model////////////////////////////////
double A_refmodelYaw[2][2] = 
{ 
    {  0.9412841309, -0.0485214680}, 
    {  0.0194085872,  0.9995098925} 
};

double B_refmodelYaw[2] = 
    {  0.0485214680,  0.0004901075};

double C_refmodelYaw[2] = 
    {  0.0000000000,  1.0000000000};

double errorYaw[2]; 
double KlpYaw[2]= 
    { 2377.3554581358, 4187.4081909840};

double KnpYaw= -291943.5798134460; 

double K1Yaw[2]= 
    { 253.4202317291, 675.3843582888};

double K2Yaw= -675.3843582888; 

double SYaw[2]= 
    { -0.0092536285, -0.0166566175};

double X_Yaw_Ref[2] , X_Yaw_smc[2]; 
