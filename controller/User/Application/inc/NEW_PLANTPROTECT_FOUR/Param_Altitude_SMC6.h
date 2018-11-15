#define INCLUDED_PARAM_ALTITUDE_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 04-Sep-2015 10:59:35

#define K_Alt 4
#define Ky_Alt 4
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A_Alt[4][4] = 
{ 
    {  0.7741357799,  0.0000000000, -0.0436037521,  0.0000020938}, 
    {  0.0156156592,  1.0000000000, -0.1147022138, -0.0000857137}, 
    {  0.0001750836,  0.0593772993,  0.8691956626, -0.0000951219}, 
    { -0.0000000862,  0.0005993019, -0.0010910227,  0.9985859033} 
};

double Kalman_B_Alt[4][4] = 
{ 
    {  0.0034968739,  0.0196084486,  0.0436037521, -0.0000020938}, 
    {  0.0000363138,  0.0022424472,  0.1147022138,  0.0000857137}, 
    {  0.0000007425,  0.0003804242,  0.0764643545,  0.0000951219}, 
    {  0.0000000038,  0.0000038780,  0.0205425630,  0.0014140967} 
};

double Kalman_C_Alt[4][4] = 
{ 
    {  0.9752962630,  0.0000000000, -0.0549342604,  0.0000026379}, 
    { -0.0018012852,  1.0000000000, -0.1137211919, -0.0000857608}, 
    { -0.0002746713,  0.0000000000,  0.9263145532, -0.0000952045}, 
    {  0.0000026379,  0.0000000000, -0.0190409059,  0.9985878066} 
};

double Kalman_D_Alt[4][4] = 
{ 
    {  0.0000000000,  0.0247037370,  0.0549342604, -0.0000026379}, 
    {  0.0000000000,  0.0018012852,  0.1137211919,  0.0000857608}, 
    {  0.0000000000,  0.0002746713,  0.0736854468,  0.0000952045}, 
    {  0.0000000000, -0.0000026379,  0.0190409059,  0.0014121934} 
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