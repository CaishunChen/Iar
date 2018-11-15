#define INCLUDED_PARAM_YAW_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 18-Dec-2015 17:20:15

#define K_Yaw 2
#define Ky_Yaw 2
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_AYaw[2][2] = 
{ 
    {  0.9144103566, -0.0131977949}, 
    {  0.0189791004,  0.9758979086} 
};

double Kalman_BYaw[2][3] = 
{ 
    { -0.0001171202,  0.0007567934,  0.0131977949}, 
    { -0.0000011885,  0.0001600389,  0.0241020914} 
};

double Kalman_CYaw[2][2] = 
{ 
    {  0.9991730545, -0.0144211851}, 
    { -0.0001442119,  0.9761739176} 
};

double Kalman_DYaw[2][3] = 
{ 
    {  0.0000000000,  0.0008269455,  0.0144211851}, 
    {  0.0000000000,  0.0001442119,  0.0238260824} 
};

//////////////////////////////Reference Model////////////////////////////////
double A_refmodelYaw[2][2] = 
{ 
    {  0.9132444549, -0.1950865162}, 
    {  0.0191261290,  0.9980196379} 
};

double B_refmodelYaw[2] = 
    {  0.1950865162,  0.0019803621};

double C_refmodelYaw[2] = 
    {  0.0000000000,  1.0000000000};

double errorYaw[2]; 
double KlpYaw[2]= 
    { 3034.2114172192, 8138.0730495841};

double KnpYaw= -6.3792253676; 

double K1Yaw[2]= 
    {  0.0000000000, 1666.8274663190};

double K2Yaw= -1666.8274663190; 

double SYaw[2]= 
    { -512.3332319030, -1537.0048792486};

double X_Yaw_Ref[2] , X_Yaw_smc[2]; 