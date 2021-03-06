#define INCLUDED_PARAM_ALTITUDE_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 14-May-2015 07:29:05

#define K_Alt 3
#define Ky_Alt 3
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A_Alt[3][3] = 
{ 
    {  0.8849047089,  0.0000000000, -0.0076064208}, 
    {  0.0180435275,  1.0000000000, -0.0528106646}, 
    {  0.0000911980,  0.0200000000,  0.9535873659} 
};

double Kalman_B_Alt[3][3] = 
{ 
    {  0.0013016157,  0.0042156879,  0.0076064208}, 
    {  0.0000132710,  0.0008259634,  0.0528106646}, 
    {  0.0000000893,  0.0001011921,  0.0464126341} 
};

double Kalman_C_Alt[3][3] = 
{ 
    {  0.9952585859,  0.0000000000, -0.0085549953}, 
    { -0.0007364953,  1.0000000000, -0.0526492362}, 
    { -0.0000855500,  0.0000000000,  0.9546419965} 
};

double Kalman_D_Alt[3][3] = 
{ 
    {  0.0000000000,  0.0047414141,  0.0085549953}, 
    {  0.0000000000,  0.0007364953,  0.0526492362}, 
    {  0.0000000000,  0.0000855500,  0.0453580035} 
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
    { -36.1447845901, -288.8004534451, -433.1720751687, -40.8921457761};

double Knp_Alt= -1.6732929641; 

double K1_Alt[3]= 
    { -24.9908850839, -253.6970781157, -202.9576624925};

double K2_Alt= 202.9576624925; 

double S_Alt[4]= 
    { 43.3187021131, 350.5896818471, 919.8683045454, 61.0953172164};

double X_Alt_Ref[3] , X_Alt_smc[3]; 
