#define INCLUDED_PARAM_ALTITUDE_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 28-Dec-2015 10:07:12

#define K_Alt 4
#define Ky_Alt 4
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A_Alt[4][4] = 
{ 
    {  0.6726892852,  0.0000000000, -0.0235123219, -0.0000115443}, 
    {  0.0162746847,  1.0000000000, -0.0852389061, -0.0161579126}, 
    {  0.0007974810,  0.0951625820,  0.8398468709, -0.0156414492}, 
    {  0.0000050117,  0.0009674836,  0.0021663223,  0.9803588104} 
};

double Kalman_B_Alt[4][4] = 
{ 
    {  0.0047196932,  0.0027446012,  0.0235123219,  0.0000115443}, 
    {  0.0000502757,  0.0002679307,  0.0852389061,  0.0161579126}, 
    {  0.0000016860,  0.0000540583,  0.0649905471,  0.0156414492}, 
    {  0.0000000086,  0.0000008978,  0.0168661941,  0.0196411896} 
};

double Kalman_C_Alt[4][4] = 
{ 
    {  0.9959365362,  0.0000000000, -0.0348106934, -0.0000170917}, 
    { -0.0002007104,  1.0000000000, -0.0846630462, -0.0161576299}, 
    { -0.0000348107,  0.0000000000,  0.9371111879, -0.0155871459}, 
    { -0.0000000171,  0.0000000000, -0.0155871459,  0.9806711054} 
};

double Kalman_D_Alt[4][4] = 
{ 
    {  0.0000000000,  0.0040634638,  0.0348106934,  0.0000170917}, 
    {  0.0000000000,  0.0002007104,  0.0846630462,  0.0161576299}, 
    {  0.0000000000,  0.0000348107,  0.0628888121,  0.0155871459}, 
    {  0.0000000000,  0.0000000171,  0.0155871459,  0.0193288946} 
};

//////////////////////////////Reference Model////////////////////////////////
double A_refmodel_Alt[4][4] = 
{ 
    {  0.6630943603, -1.3169105337, -0.0152874249, -1.4809269475}, 
    {  0.0164547439,  0.9859364349, -0.0001062050, -0.0158184497}, 
    {  0.0008493014,  0.0946905055,  0.9048347515, -0.0005310248}, 
    {  0.0000059003,  0.0009650648,  0.0190325055,  0.9999972790} 
};

double B_refmodel_Alt[4] = 
    {  1.4809269475,  0.0158184497,  0.0005310248,  0.0000027210};

double C_refmodel_Alt[4] = 
    {  0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000};

double error_Alt[5]; 
double Klp_Alt[5]= 
    { 10.3189299383, 13.3766117269, -28.0351080764, -34.6265994469, -194.0660482565};

double Knp_Alt= -970.3302412832; 

double K1_Alt[4]= 
    {  0.0000000000, -280.4015401540,  0.0000000000, -315.4517326733};

double K2_Alt= 315.4517326733; 

double S_Alt[5]= 
    {  0.0361219212,  0.5301237361,  0.3383299889,  1.3039133340,  1.0000000000};

double X_Alt_Ref[4] , X_Alt_smc[4]; 
