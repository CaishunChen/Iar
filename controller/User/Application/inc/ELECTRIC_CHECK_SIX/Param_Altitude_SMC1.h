#define INCLUDED_PARAM_ALTITUDE_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 13-May-2015 20:35:14

#define K_Alt 3
#define Ky_Alt 3
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A_Alt[3][3] = 
{ 
    {  0.8869884289,  0.0000000000, -0.0042094817}, 
    {  0.0184400699,  1.0000000000, -0.0383834333}, 
    {  0.0001369009,  0.0200000000,  0.9604804642} 
};

double Kalman_B_Alt[3][3] = 
{ 
    {  0.0013016157,  0.0021319679,  0.0042094817}, 
    {  0.0000132710,  0.0004294209,  0.0383834333}, 
    {  0.0000000893,  0.0000554892,  0.0395195358} 
};

double Kalman_C_Alt[3][3] = 
{ 
    {  0.9976021606,  0.0000000000, -0.0047344339}, 
    { -0.0003841749,  1.0000000000, -0.0382940970}, 
    { -0.0000473443,  0.0000000000,  0.9612472570} 
};

double Kalman_D_Alt[3][3] = 
{ 
    {  0.0000000000,  0.0023978394,  0.0047344339}, 
    {  0.0000000000,  0.0003841749,  0.0382940970}, 
    {  0.0000000000,  0.0000473443,  0.0387527430} 
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