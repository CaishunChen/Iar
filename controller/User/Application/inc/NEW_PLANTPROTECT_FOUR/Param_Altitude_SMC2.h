#define INCLUDED_PARAM_ALTITUDE_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 18-Jul-2015 13:57:10

#define K_Alt 3
#define Ky_Alt 3
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A_Alt[3][3] = 
{ 
    {  0.7581206553,  0.0000000000, -0.0017051661}, 
    {  0.0135984584,  1.0000000000, -0.0351018388}, 
    { -0.0001061325,  0.0200000000,  0.9622027879} 
};

double Kalman_B_Alt[3][3] = 
{ 
    {  0.0019877492,  0.0377605930,  0.0017051661}, 
    {  0.0000206332,  0.0042827525,  0.0351018388}, 
    {  0.0000001401,  0.0002917427,  0.0377972121} 
};

double Kalman_C_Alt[3][3] = 
{ 
    {  0.9525549910,  0.0000000000, -0.0021424881}, 
    { -0.0034343783,  1.0000000000, -0.0350635286}, 
    { -0.0002142488,  0.0000000000,  0.9629044561} 
};

double Kalman_D_Alt[3][3] = 
{ 
    {  0.0000000000,  0.0474450090,  0.0021424881}, 
    {  0.0000000000,  0.0034343783,  0.0350635286}, 
    {  0.0000000000,  0.0002142488,  0.0370955439} 
};

//////////////////////////////Reference Model////////////////////////////////
double A_refmodel_Alt[3][3] = 
{ 
    {  0.8558068556, -0.3271698068, -0.2779867328}, 
    {  0.0185324489,  0.9966534669, -0.0028519519}, 
    {  0.0001901301,  0.0199774378,  0.9999807441} 
};

double B_refmodel_Alt[3] = 
    {  0.2779867328,  0.0028519519,  0.0000192559};

double C_refmodel_Alt[3] = 
    {  0.0000000000,  0.0000000000,  1.0000000000};

double error_Alt[4]; 
double Klp_Alt[4]= 
    { -7.4743685004, -39.0315394591, -27.1750079524, -1.1982026746};

double Knp_Alt= -5.9910133732; 

double K1_Alt[3]= 
    { 34.3210038816, -157.4248834487, -134.9356143846};

double K2_Alt= 134.9356143846; 

double S_Alt[4]= 
    { 15.0153355790, 96.5618462194, 134.7948259661,  1.0000000000};

double X_Alt_Ref[3] , X_Alt_smc[3]; 
