#define INCLUDED_PARAM_ROLLPITCH_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 13-May-2015 10:23:04

#define MRSMC_Control  
#define K_Roll 3
#define Ky_Roll 3
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A[3][3] = 
{ 
    {  0.1855708721, -4.1490521204,  0.3343498019}, 
    {  0.0099844820,  0.9421967117, -0.0584034186}, 
    {  0.0001261260,  0.0188495564,  0.9487056642} 
};

double Kalman_B[3][3] = 
{ 
    {  0.0135505257, -0.0209819559, -0.3343498019}, 
    {  0.0001711730,  0.0051265550,  0.0584034186}, 
    {  0.0000012767,  0.0007575395,  0.0512943358} 
};

double Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0069043167,  0.3366264310}, 
    {  0.0000000000,  0.9946611477, -0.0651989256}, 
    {  0.0000000000, -0.0006519893,  0.9499415684} 
};

double Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0069043167, -0.3366264310}, 
    {  0.0000000000,  0.0053388523,  0.0651989256}, 
    {  0.0000000000,  0.0006519893,  0.0500584316} 
};

//////////////////////////////Reference Model////////////////////////////////
double A_refmodel[3][3] = 
{ 
    {  0.7037930877, -1.9146754029, -4.8366079350}, 
    {  0.0169408334,  0.9799286720, -0.0511837292}, 
    {  0.0001792775,  0.0198630568,  0.9996491982} 
};

double B_refmodel[3] = 
    {  4.8366079350,  0.0511837292,  0.0003508018};

double C_refmodel[3] = 
    {  0.0000000000,  0.0000000000,  1.0000000000};

double error[3]; 
double Klp[3]= 
    { -14.0223662174, -175.2396948360, -673.9956523112};

double Knp= -47.3558071184; 

double K1[3]= 
    { 44.2053208890, 226.6879616433, -210.3659788127};

double K2= 210.3659788127; 

double S[3]= 
    {  0.3111903675,  4.1483465595, 16.9771178684};

double X_Roll_Ref[3] , X_Roll_smc[3] , theta_error_roll; 
double X_Pitch_Ref[3] , X_Pitch_smc[3] , theta_error_pitch; 