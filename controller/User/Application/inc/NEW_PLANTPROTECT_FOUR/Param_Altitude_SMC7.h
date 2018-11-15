#define INCLUDED_PARAM_ALTITUDE_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 04-Sep-2015 11:14:13

#define K_Alt 4
#define Ky_Alt 4
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A_Alt[4][4] = 
{ 
    {  0.7860800933,  0.0000000000, -0.0108415465,  0.0000008098}, 
    {  0.0169117093,  1.0000000000, -0.0542558711, -0.0001735816}, 
    {  0.0003750228,  0.0593772993,  0.9022775130, -0.0001893453}, 
    {  0.0000016547,  0.0005993019, -0.0003397035,  0.9980010521} 
};

double Kalman_B_Alt[4][4] = 
{ 
    {  0.0034968739,  0.0076641352,  0.0108415465, -0.0000008098}, 
    {  0.0000363138,  0.0009463971,  0.0542558711,  0.0001735816}, 
    {  0.0000007425,  0.0001804850,  0.0433825041,  0.0001893453}, 
    {  0.0000000038,  0.0000021371,  0.0197912437,  0.0019989479} 
};

double Kalman_C_Alt[4][4] = 
{ 
    {  0.9903443264,  0.0000000000, -0.0136587406,  0.0000010202}, 
    { -0.0007739651,  1.0000000000, -0.0540119518, -0.0001735998}, 
    { -0.0001365874,  0.0000000000,  0.9575240234, -0.0001893260}, 
    {  0.0000010202,  0.0000000000, -0.0189325993,  0.9980048388} 
};

double Kalman_D_Alt[4][4] = 
{ 
    {  0.0000000000,  0.0096556736,  0.0136587406, -0.0000010202}, 
    {  0.0000000000,  0.0007739651,  0.0540119518,  0.0001735998}, 
    {  0.0000000000,  0.0001365874,  0.0424759766,  0.0001893260}, 
    {  0.0000000000, -0.0000010202,  0.0189325993,  0.0019951612} 
};

//////////////////////////////Reference Model////////////////////////////////
double A_refmodel_Alt[4][4] = 
{ 
    {  0.7264957964, -1.5033036144, -0.0095844362, -0.9276290189}, 
    {  0.0171783152,  0.9841705238, -0.0000658578, -0.0097684166}, 
    {  0.0005418046,  0.0590520089,  0.9456591607, -0.0002010376}, 
    {  0.0000037229,  0.0005976484,  0.0194515378,  0.9999989766} 
};

double B_refmodel_Alt[4] = 
    {  0.9276290189,  0.0097684166,  0.0002010376,  0.0000010234};

double C_refmodel_Alt[4] = 
    {  0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000};

double error_Alt[5]; 
double Klp_Alt[5]= 
    { -10.1709432519, -152.8220971191, -292.6166798535, -817.2637836191, -551.2327819328};

double Knp_Alt= -4.9940506778; 

double K1_Alt[4]= 
    { -17.6202592073, -446.8517753922,  0.0000000000, -275.7713813849};

double K2_Alt= 275.7713813849; 

double S_Alt[5]= 
    { 10.2259223274, 153.3031096812, 292.9208828277, 818.3925502272, 551.8894555681};

double X_Alt_Ref[4] , X_Alt_smc[4]; 