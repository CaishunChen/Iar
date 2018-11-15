#define INCLUDED_PARAM_ALTITUDE_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 19-Dec-2015 14:02:17

#define K_Alt 4
#define Ky_Alt 4
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A_Alt[4][4] = 
{ 
    {  0.7262021700,  0.0000000000, -0.0211352250,  0.0000018247}, 
    {  0.0158918464,  1.0000000000, -0.0632573434, -0.0001723066}, 
    {  0.0007901821,  0.1299510868,  0.8236603515, -0.0001888840}, 
    {  0.0000038381,  0.0013269263, -0.0012436294,  0.9980007811} 
};

double Kalman_B_Alt[4][4] = 
{ 
    {  0.0032423989,  0.0131758202,  0.0211352250, -0.0000018247}, 
    {  0.0000340532,  0.0013709761,  0.0632573434,  0.0001723066}, 
    {  0.0000015587,  0.0004114529,  0.0574130568,  0.0001888840}, 
    {  0.0000000080,  0.0000044608,  0.0200292739,  0.0019992189} 
};

double Kalman_C_Alt[4][4] = 
{ 
    {  0.9821798588,  0.0000000000, -0.0285851422,  0.0000024679}, 
    { -0.0010633502,  1.0000000000, -0.0627638832, -0.0001723492}, 
    { -0.0002858514,  0.0000000000,  0.9441335165, -0.0001889627}, 
    {  0.0000024679,  0.0000000000, -0.0188962657,  0.9980045596} 
};

double Kalman_D_Alt[4][4] = 
{ 
    {  0.0000000000,  0.0178201412,  0.0285851422, -0.0000024679}, 
    {  0.0000000000,  0.0010633502,  0.0627638832,  0.0001723492}, 
    {  0.0000000000,  0.0002858514,  0.0558664835,  0.0001889627}, 
    {  0.0000000000, -0.0000024679,  0.0188962657,  0.0019954404} 
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
