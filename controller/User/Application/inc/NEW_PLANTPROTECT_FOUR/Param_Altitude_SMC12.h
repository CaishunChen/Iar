#define INCLUDED_PARAM_ALTITUDE_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 29-Sep-2015 16:54:22

#define K_Alt 4
#define Ky_Alt 4
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A_Alt[4][4] = 
{ 
    {  0.7595139806,  0.0000000000, -0.0262952679,  0.0000023512}, 
    {  0.0149649442,  1.0000000000, -0.0900447676, -0.0001728211}, 
    {  0.0000866648,  0.0593772993,  0.8816442150, -0.0001897919}, 
    { -0.0000011958,  0.0005993019, -0.0007938119,  0.9980004318} 
};

double Kalman_B_Alt[4][4] = 
{ 
    {  0.0040914000,  0.0251497670,  0.0262952679, -0.0000023512}, 
    {  0.0000425660,  0.0027947467,  0.0900447676,  0.0001728211}, 
    {  0.0000008711,  0.0004667894,  0.0640158021,  0.0001897919}, 
    {  0.0000000044,  0.0000049770,  0.0202453522,  0.0019995682} 
};

double Kalman_C_Alt[4][4] = 
{ 
    {  0.9679483510,  0.0000000000, -0.0335115111,  0.0000029964}, 
    { -0.0022255194,  1.0000000000, -0.0894496135, -0.0001728744}, 
    { -0.0003351151,  0.0000000000,  0.9379417788, -0.0001898449}, 
    {  0.0000029964,  0.0000000000, -0.0189844901,  0.9980042281} 
};

double Kalman_D_Alt[4][4] = 
{ 
    {  0.0000000000,  0.0320516490,  0.0335115111, -0.0000029964}, 
    {  0.0000000000,  0.0022255194,  0.0894496135,  0.0001728744}, 
    {  0.0000000000,  0.0003351151,  0.0620582212,  0.0001898449}, 
    {  0.0000000000, -0.0000029964,  0.0189844901,  0.0019957719} 
};

//////////////////////////////Reference Model////////////////////////////////
double A_refmodel_Alt[4][4] = 
{ 
    {  0.7947473790, -0.8055530224, -0.0045588664, -0.4474766989}, 
    {  0.0178990680,  0.9916371266, -0.0000311093, -0.0046457733}, 
    {  0.0005566558,  0.0592068352,  0.9456596966, -0.0000949643}, 
    {  0.0000037986,  0.0005984401,  0.0194515399,  0.9999995185} 
};

double B_refmodel_Alt[4] = 
    {  0.4474766989,  0.0046457733,  0.0000949643,  0.0000004815};

double C_refmodel_Alt[4] = 
    {  0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000};

double error_Alt[5]; 
double Klp_Alt[5]= 
    { -22.4995128705, -244.4802653226, -313.1897711058, -762.2060628122, -85.3311485108};

double Knp_Alt= -1.8320048769; 

double K1_Alt[4]= 
    {  4.8833289474, -195.3331578947,  0.0000000000, -108.5184210526};

double K2_Alt= 108.5184210526; 

double S_Alt[5]= 
    { 18.9551322571, 211.9816799006, 281.9462497426, 745.5510757112, 74.5248221448};

double X_Alt_Ref[4] , X_Alt_smc[4]; 
