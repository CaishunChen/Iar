#define INCLUDED_PARAM_ALTITUDE_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 19-Dec-2015 15:39:54

#define K_Alt 4
#define Ky_Alt 4
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A_Alt[4][4] = 
{ 
    {  0.7343135083,  0.0000000000, -0.0091606074,  0.0000003806}, 
    {  0.0167135949,  1.0000000000, -0.0407193783, -0.0000868854}, 
    {  0.0010282359,  0.1299510868,  0.8420203232, -0.0000950649}, 
    {  0.0000063721,  0.0013269263, -0.0010045715,  0.9985864360} 
};

double Kalman_B_Alt[4][4] = 
{ 
    {  0.0032423989,  0.0050644819,  0.0091606074, -0.0000003806}, 
    {  0.0000340532,  0.0005492275,  0.0407193783,  0.0000868854}, 
    {  0.0000015587,  0.0001733991,  0.0390530851,  0.0000950649}, 
    {  0.0000000080,  0.0000019268,  0.0197902160,  0.0014135640} 
};

double Kalman_C_Alt[4][4] = 
{ 
    {  0.9931503480,  0.0000000000, -0.0123896134,  0.0000005147}, 
    { -0.0004309832,  1.0000000000, -0.0405054986, -0.0000868943}, 
    { -0.0001238961,  0.0000000000,  0.9616666859, -0.0000950812}, 
    {  0.0000010294,  0.0000000000, -0.0190162493,  0.9985883375} 
};

double Kalman_D_Alt[4][4] = 
{ 
    {  0.0000000000,  0.0068496520,  0.0123896134, -0.0000005147}, 
    {  0.0000000000,  0.0004309832,  0.0405054986,  0.0000868943}, 
    {  0.0000000000,  0.0001238961,  0.0383333141,  0.0000950812}, 
    {  0.0000000000, -0.0000010294,  0.0190162493,  0.0014116625} 
};

//////////////////////////////Reference Model////////////////////////////////
double A_refmodel_Alt[4][4] = 
{ 
    {  0.7262937103, -1.3744277144, -0.0155916315, -1.5453923284}, 
    {  0.0171710259,  0.9855298392, -0.0001077989, -0.0162746451}, 
    {  0.0011983035,  0.1292706824,  0.8809765988, -0.0007456447}, 
    {  0.0000082849,  0.0013233837,  0.0187846578,  0.9999961903} 
};

double B_refmodel_Alt[4] = 
    {  1.5453923284,  0.0162746451,  0.0007456447,  0.0000038097};

double C_refmodel_Alt[4] = 
    {  0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000};

double error_Alt[5]; 
double Klp_Alt[5]= 
    {  0.7979755946, 15.4511053609, 15.8773635427, 158.0859984173, -22.1198547506};

double Knp_Alt= -110.5992737532; 

double K1_Alt[4]= 
    {  0.0000000000, -425.9271762720,  0.0000000000, -479.1680733060};

double K2_Alt= 479.1680733060; 

double S_Alt[5]= 
    {  0.4813855935,  6.2327013676,  3.5634537760, 14.0155598221,  1.0000000000};

double X_Alt_Ref[4] , X_Alt_smc[4]; 
