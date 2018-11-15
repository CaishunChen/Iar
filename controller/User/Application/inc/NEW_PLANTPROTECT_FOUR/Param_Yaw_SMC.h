#define INCLUDED_PARAM_YAW_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 04-Jul-2015 17:13:45

#define K_Yaw 2
#define Ky_Yaw 2
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_AYaw[2][2] = 
{ 
    {  0.9145476048, -0.0246435926}, 
    {  0.0188475580,  0.9670620272} 
};

double Kalman_BYaw[2][3] = 
{ 
    { -0.0001668191,  0.0015267136,  0.0246435926}, 
    { -0.0000016926,  0.0003009256,  0.0329379728} 
};

double Kalman_CYaw[2][2] = 
{ 
    {  0.9983334173, -0.0269013027}, 
    { -0.0002690130,  0.9675771463} 
};

double Kalman_DYaw[2][3] = 
{ 
    {  0.0000000000,  0.0016665827,  0.0269013027}, 
    {  0.0000000000,  0.0002690130,  0.0324228537} 
};

//////////////////////////////Reference Model////////////////////////////////
double A_refmodelYaw[2][2] = 
{ 
    {  0.9221303558, -0.0999141081}, 
    {  0.0192142516,  0.9989873620} 
};

double B_refmodelYaw[2] = 
    {  0.0999141081,  0.0010126380};

double C_refmodelYaw[2] = 
    {  0.0000000000,  1.0000000000};

double errorYaw[2]; 
double KlpYaw[2]= 
    { 912.8220120958, 1644.1772573463};

double KnpYaw= -276068.1322066981; 

double K1Yaw[2]= 
    { -43.9502943100, 596.8868541530};

double K2Yaw= -596.8868541530; 

double SYaw[2]= 
    { -0.0041578842, -0.0081177936};

double X_Yaw_Ref[2] , X_Yaw_smc[2]; 