#define INCLUDED_PARAM_ROLLPITCH_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 06-May-2015 18:09:51

#define MRSMC_Control  
#define K_Roll 3
#define Ky_Roll 3
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A[3][3] = 
{ 
    {  0.1525837611, -7.2110793612,  1.4249326177}, 
    {  0.0096177875,  0.8829017351, -0.1334082616}, 
    {  0.0001235805,  0.0171893642,  0.9192287611} 
};

double Kalman_B[3][3] = 
{ 
    {  0.0390313111, -0.1665430054, -1.4249326177}, 
    {  0.0005015196,  0.0223020002,  0.1334082616}, 
    {  0.0000037651,  0.0020989592,  0.0807712389} 
};

double Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0659109548,  1.4617596185}, 
    {  0.0000000000,  0.9760627560, -0.1629104578}, 
    {  0.0000000000, -0.0016291046,  0.9221903857} 
};

double Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0659109548, -1.4617596185}, 
    {  0.0000000000,  0.0239372440,  0.1629104578}, 
    {  0.0000000000,  0.0016291046,  0.0778096143} 
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
    { -4.7808932293, -60.9047039823, -231.7195129254};

double Knp= -7.2370860556; 

double K1[3]= 
    { 15.2659498447, 161.9127197883, -70.3506556806};

double K2= 70.3506556806; 

double S[3]= 
    {  0.6809704958,  9.3306731765, 37.9447255536};

double X_Roll_Ref[3] , X_Roll_smc[3] , theta_error_roll; 
double X_Pitch_Ref[3] , X_Pitch_smc[3] , theta_error_pitch; 
