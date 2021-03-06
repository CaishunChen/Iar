#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 14-Sep-2015 17:21:43

#define MRSMC_Control  
#define K_Pos 3
#define Ky_Pos 3
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[3][3] = 
{ 
    {  0.9582157407, -0.0132186944, -0.0001850280}, 
    {  0.0195793003,  0.9811908880, -0.0050008012}, 
    {  0.0001971858,  0.0071341578,  0.9882281416} 
};

double Pos_Kalman_B[3][3] = 
{ 
    {  0.3503777733,  0.0098350366,  0.0001850280}, 
    {  0.0035287018,  0.0187750348,  0.0050008012}, 
    {  0.0000236081,  0.0128656142,  0.0117718584} 
};

double Pos_Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0103294929, -0.0002107413}, 
    {  0.0000000000,  0.9814265765, -0.0049968453}, 
    {  0.0000000000, -0.0124921132,  0.9883281190} 
};

double Pos_Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0103294929,  0.0002107413}, 
    {  0.0000000000,  0.0185734235,  0.0049968453}, 
    {  0.0000000000,  0.0124921132,  0.0116718810} 
};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[3][3] = 
{ 
    {  0.9354801307, -0.0657896505,  0.0000000000}, 
    {  0.0193498972,  0.9993347914,  0.0000000000}, 
    {  0.0001956496,  0.0199955408,  1.0000000000} 
};

double Pos_B_refmodel[3] = 
    {  0.0657896505,  0.0006652086,  0.0000044592};

double Pos_C_refmodel[3] = 
    {  0.0000000000,  1.0000000000,  0.0000000000};

double Perror[3]; 

double PKlp[4]= 
    { -0.3842722123, -1.4671400610, -1.0956742985, -0.2094348945};

double PKnp= -0.2253563983; 

double PK1[3]= 
    { -0.0652481712, -0.1803366765,  0.0000000000};

double PK2=  0.1899938469; 

double PSr[3]= 
    {  1.2398259742,  6.4181734126,  4.6760923031};

double PSi = 0.929350; 

double X_PosX_Ref[3] , X_PosX_smc[3]; 
double X_PosY_Ref[3] , X_PosY_smc[3]; 
#define N_STATE    3
