#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 13-Nov-2015 17:01:20

#define MRSMC_Control  
#define K_Pos 3
#define Ky_Pos 3
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[3][3] = 
{ 
    {  0.9812044967, -0.0264695876, -0.0000916383}, 
    {  0.0198115523,  0.9693351133, -0.0020657287}, 
    {  0.0001987420,  0.0028813113,  0.9930190592} 
};

double Pos_Kalman_B[3][3] = 
{ 
    {  0.1775233569,  0.0234461122,  0.0000916383}, 
    {  0.0017808475,  0.0306345563,  0.0020657287}, 
    {  0.0000118911,  0.0171184862,  0.0069809408} 
};

double Pos_Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0239881707, -0.0000997531}, 
    {  0.0000000000,  0.9698397718, -0.0020638150}, 
    {  0.0000000000, -0.0165105203,  0.9930603549} 
};

double Pos_Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0239881707,  0.0000997531}, 
    {  0.0000000000,  0.0301602282,  0.0020638150}, 
    {  0.0000000000,  0.0165105203,  0.0069396451} 
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
    { -0.2689496466, -0.7095394461, -0.5798846451, -0.1044462607};

double PKnp= -3.3028807703; 

double PK1[3]= 
    { -0.2625736158, -0.3624075370,  0.0000000000};

double PK2=  0.3794389600; 

double PSr[3]= 
    {  0.1013657790,  0.2746961600,  0.1650284291};

double PSi = 0.031623; 

double X_PosX_Ref[3] , X_PosX_smc[3]; 
double X_PosY_Ref[3] , X_PosY_smc[3]; 
#define N_STATE    3
