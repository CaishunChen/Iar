#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 14-Nov-2015 16:45:53

#define MRSMC_Control  
#define K_Pos 3
#define Ky_Pos 3
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[3][3] = 
{ 
    {  0.9812044967, -0.0058557719, -0.0003123698}, 
    {  0.0198115523,  0.9917669398, -0.0065257965}, 
    {  0.0001987420,  0.0123856984,  0.9853693662} 
};

double Pos_Kalman_B[3][3] = 
{ 
    {  0.1775233569,  0.0028322965,  0.0003123698}, 
    {  0.0017808475,  0.0082027299,  0.0065257965}, 
    {  0.0000118911,  0.0076140991,  0.0146306338} 
};

double Pos_Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0029116495, -0.0003384419}, 
    {  0.0000000000,  0.9918547074, -0.0065192892}, 
    {  0.0000000000, -0.0074506162,  0.9854998179} 
};

double Pos_Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0029116495,  0.0003384419}, 
    {  0.0000000000,  0.0081452926,  0.0065192892}, 
    {  0.0000000000,  0.0074506162,  0.0145001821} 
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
    { -0.1447109015, -0.3632462468, -0.3533356124, -0.0542544286};

double PKnp= -2.8289919615; 

double PK1[3]= 
    { -0.2625736158, -0.3624075370,  0.0000000000};

double PK2=  0.3794389600; 

double PSr[3]= 
    {  0.0788971447,  0.2048719175,  0.1153090580};

double PSi = 0.019178; 

double X_PosX_Ref[3] , X_PosX_smc[3]; 
double X_PosY_Ref[3] , X_PosY_smc[3]; 
#define N_STATE    3