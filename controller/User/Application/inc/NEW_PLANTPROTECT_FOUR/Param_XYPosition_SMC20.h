#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 16-Sep-2015 13:01:14

#define MRSMC_Control  
#define K_Pos 3
#define Ky_Pos 3
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[3][3] = 
{ 
    {  0.9582157407, -0.0208954852,  0.0000159281}, 
    {  0.0195793003,  0.9729576572, -0.0008420312}, 
    {  0.0001971858,  0.0026166236,  0.9955748454} 
};

double Pos_Kalman_B[3][3] = 
{ 
    {  0.3503777733,  0.0175118273, -0.0000159281}, 
    {  0.0035287018,  0.0270082655,  0.0008420312}, 
    {  0.0000236081,  0.0173831484,  0.0044251546} 
};

double Pos_Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0183695586,  0.0000136482}, 
    {  0.0000000000,  0.9733504895, -0.0008423271}, 
    {  0.0000000000, -0.0168465420,  0.9955916890} 
};

double Pos_Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0183695586, -0.0000136482}, 
    {  0.0000000000,  0.0266495105,  0.0008423271}, 
    {  0.0000000000,  0.0168465420,  0.0044083110} 
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
    { -0.3613367572, -1.5039252632, -1.8598072658, -0.2435377949};

double PKnp= -0.0974151179; 

double PK1[3]= 
    { -0.0652481712, -0.1803366765,  0.0000000000};

double PK2=  0.1899938469; 

double PSr[3]= 
    {  1.1472663463,  5.4681416885,  7.4366268610};

double PSi = 1.000000; 

double X_PosX_Ref[3] , X_PosX_smc[3]; 
double X_PosY_Ref[3] , X_PosY_smc[3]; 
#define N_STATE    3
