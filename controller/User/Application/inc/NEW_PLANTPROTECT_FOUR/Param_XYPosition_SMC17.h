#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 16-Sep-2015 12:05:55

#define MRSMC_Control  
#define K_Pos 3
#define Ky_Pos 3
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[3][3] = 
{ 
    {  0.9582157407, -0.0896403426, -0.0000045116}, 
    {  0.0195793003,  0.9392163777, -0.0011367026}, 
    {  0.0001971858,  0.0006171655,  0.9950095503} 
};

double Pos_Kalman_B[3][3] = 
{ 
    {  0.3503777733,  0.0862566847,  0.0000045116}, 
    {  0.0035287018,  0.0607495450,  0.0011367026}, 
    {  0.0000236081,  0.0193826065,  0.0049904497} 
};

double Pos_Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0902263093, -0.0000087218}, 
    {  0.0000000000,  0.9410150129, -0.0011365706}, 
    {  0.0000000000, -0.0181851289,  0.9950322832} 
};

double Pos_Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0902263093,  0.0000087218}, 
    {  0.0000000000,  0.0589849871,  0.0011365706}, 
    {  0.0000000000,  0.0181851289,  0.0049677168} 
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
    { -1.3836449110, -9.0218956946, -4.1048131872, -0.8591612409};

double PKnp= -0.2147903102; 

double PK1[3]= 
    { -0.0652481712, -0.1803366765,  0.0000000000};

double PK2=  0.1899938469; 

double PSr[3]= 
    {  1.3008162044, 10.4855716909,  4.7276982852};

double PSi = 1.000000; 

double X_PosX_Ref[3] , X_PosX_smc[3]; 
double X_PosY_Ref[3] , X_PosY_smc[3]; 
#define N_STATE    3
