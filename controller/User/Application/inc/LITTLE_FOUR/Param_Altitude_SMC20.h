#define INCLUDED_PARAM_ALTITUDE_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 27-Aug-2016 15:53:23

#define K_Alt 4
#define Ky_Alt 4
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A_Alt[4][4] = 
{ 
    {  0.6886077169,  0.0000000000, -0.0408987118, -0.0000314402}, 
    {  0.0164626362,  1.0000000000, -0.0846290194, -0.0157955222}, 
    {  0.0015687090,  0.1812692469,  0.7460077321, -0.0156579726}, 
    {  0.0000100625,  0.0018730753,  0.0010697212,  0.9803400177} 
};

double Kalman_B_Alt[4][4] = 
{ 
    {  0.0045226538,  0.0026831451,  0.0408987118,  0.0000314402}, 
    {  0.0000480031,  0.0002607463,  0.0846290194,  0.0157955222}, 
    {  0.0000031350,  0.0000903771,  0.0727230210,  0.0156579726}, 
    {  0.0000000161,  0.0000015297,  0.0170572035,  0.0196599823} 
};

double Kalman_C_Alt[4][4] = 
{ 
    {  0.9961186452,  0.0000000000, -0.0591628126, -0.0000454804}, 
    { -0.0001958369,  1.0000000000, -0.0836396171, -0.0157947616}, 
    { -0.0000591628,  0.0000000000,  0.9298138317, -0.0156275949}, 
    { -0.0000000455,  0.0000000000, -0.0156275949,  0.9806528833} 
};

double Kalman_D_Alt[4][4] = 
{ 
    {  0.0000000000,  0.0038813548,  0.0591628126,  0.0000454804}, 
    {  0.0000000000,  0.0001958369,  0.0836396171,  0.0157947616}, 
    {  0.0000000000,  0.0000591628,  0.0701861683,  0.0156275949}, 
    {  0.0000000000,  0.0000000455,  0.0156275949,  0.0193471167} 
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
    { -31.8821518720, -523.6762535341, -225.0311363904, -479.5168083392, -115.1893874314};

double Knp_Alt= -115.1893874311; 

double K1_Alt[4]= 
    { -4.2903188663, -295.8153931469,  0.0000000000, -332.7923172903};

double K2_Alt= 332.7923172903; 

double S_Alt[5]= 
    {  0.3210098167,  5.8559173204,  2.5167616728,  6.9519441395,  1.0000000000};

double X_Alt_Ref[4] , X_Alt_smc[4]; 
