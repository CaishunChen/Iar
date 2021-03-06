#define INCLUDED_PARAM_YAW_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 08-May-2015 15:08:50

#define K_Yaw 2
#define Ky_Yaw 2
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_AYaw[2][2] = 
{ 
    {  0.9353023084, -0.0192947673}, 
    {  0.0191299603,  0.9711700119} 
};

double Kalman_BYaw[2][3] = 
{ 
    { -0.0001133967,  0.0009343411,  0.0192947673}, 
    { -0.0000011464,  0.0002254047,  0.0288299881} 
};

double Kalman_CYaw[2][2] = 
{ 
    {  0.9990020247, -0.0206088571}, 
    { -0.0002060886,  0.9715689039} 
};

double Kalman_DYaw[2][3] = 
{ 
    {  0.0000000000,  0.0009979753,  0.0206088571}, 
    {  0.0000000000,  0.0002060886,  0.0284310961} 
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
    { 1502.0507670239, 2645.6904187772};

double KnpYaw= -116547.7907976381; 

double K1Yaw[2]= 
    { -50.2417903734, 426.7178362573};

double K2Yaw= -426.7178362573; 

double SYaw[2]= 
    { -0.0146452484, -0.0263617889};

double X_Yaw_Ref[2] , X_Yaw_smc[2]; 
