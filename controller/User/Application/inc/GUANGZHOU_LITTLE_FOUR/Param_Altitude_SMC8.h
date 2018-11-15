#define INCLUDED_PARAM_ALTITUDE_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 09-May-2015 15:19:02

#define K_Alt 3
#define Ky_Alt 3
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A_Alt[3][3] = 
{ 
    {  0.7458828282,  0.0000000000, -0.0024332758}, 
    {  0.0160858240,  1.0000000000, -0.0454169291}, 
    {  0.0000935652,  0.0200000000,  0.9569507316} 
};

double Kalman_B_Alt[3][3] = 
{ 
    {  0.0029822083,  0.0122118947,  0.0024332758}, 
    {  0.0000311968,  0.0013836075,  0.0454169291}, 
    {  0.0000002127,  0.0000891823,  0.0430492684} 
};

double Kalman_C_Alt[3][3] = 
{ 
    {  0.9838913340,  0.0000000000, -0.0032097253}, 
    { -0.0011021983,  1.0000000000, -0.0453608570}, 
    { -0.0000641945,  0.0000000000,  0.9578585354} 
};

double Kalman_D_Alt[3][3] = 
{ 
    {  0.0000000000,  0.0161086660,  0.0032097253}, 
    {  0.0000000000,  0.0011021983,  0.0453608570}, 
    {  0.0000000000,  0.0000641945,  0.0421414646} 
};

//////////////////////////////Reference Model////////////////////////////////
double A_refmodel_Alt[3][3] = 
{ 
    {  0.8558080912, -0.3269797868, -0.2594543718}, 
    {  0.0185324551,  0.9966547502, -0.0026618221}, 
    {  0.0001901302,  0.0199774443,  0.9999820279} 
};

double B_refmodel_Alt[3] = 
    {  0.2594543718,  0.0026618221,  0.0000179721};

double C_refmodel_Alt[3] = 
    {  0.0000000000,  0.0000000000,  1.0000000000};

double error_Alt[4]; 
double Klp_Alt[4]= 
    { -6.6695726660, -40.6773522269, -53.0969318853, -13.2431127648};

double Knp_Alt= -4.3670920308; 

double K1_Alt[3]= 
    { 36.5962362103, -102.5129785853, -82.0103828683};

double K2_Alt= 82.0103828683; 

double S_Alt[4]= 
    { 13.4136960002, 90.3890404304, 147.1067674619, 15.1623925846};

double X_Alt_Ref[3] , X_Alt_smc[3]; 
