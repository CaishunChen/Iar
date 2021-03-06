#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 20-Jul-2015 16:15:02

#define MRSMC_Control  
#define K_Pos 3
#define Ky_Pos 3
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[3][3] = 
{ 
    {  0.9797159003, -0.0054479709, -0.0004192875}, 
    {  0.0197965764,  0.9934946361, -0.0081249119}, 
    {  0.0001986419,  0.0140735059,  0.9832590732} 
};

double Pos_Kalman_B[3][3] = 
{ 
    {  0.1629434282,  0.0021764087,  0.0004192875}, 
    {  0.0016349995,  0.0064725365,  0.0081249119}, 
    {  0.0000109186,  0.0059262749,  0.0167409268} 
};

double Pos_Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0022429353, -0.0004550707}, 
    {  0.0000000000,  0.9935716549, -0.0081161694}, 
    {  0.0000000000, -0.0057972639,  0.9834214852} 
};

double Pos_Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0022429353,  0.0004550707}, 
    {  0.0000000000,  0.0064283451,  0.0081161694}, 
    {  0.0000000000,  0.0057972639,  0.0165785148} 
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
    { -0.3802958072, -0.9536834985, -0.7368334367, -0.2444835693};

double PKnp= -0.0940535448; 

double PK1[3]= 
    { -0.1587959142, -0.1743117860,  0.0000000000};

double PK2=  0.1943896883; 

double PSr[3]= 
    {  1.2917488167,  4.4696739639,  3.2672434273};

double PSi = 1.299704; 

double X_PosX_Ref[3] , X_PosX_smc[3]; 
double X_PosY_Ref[3] , X_PosY_smc[3]; 
#define N_STATE    3
