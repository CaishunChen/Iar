#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 19-Dec-2015 12:29:07

#define MRSMC_Control  
#define K_Pos 4
#define Ky_Pos 4
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[4][4] = 
{ 
    {  0.9040764269, -0.0038049557, -0.0158299195,  0.0000365627}, 
    {  0.0190247786,  0.9999613111, -0.0377923448, -0.0020685558}, 
    {  0.0012337215,  0.1248250037,  0.8414760425, -0.0020640393}, 
    {  0.0000083865,  0.0012759894,  0.0015488483,  0.9930178694} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.6532157734,  0.0158299195, -0.0000365627}, 
    {  0.0066419240,  0.0377923448,  0.0020685558}, 
    {  0.0002879496,  0.0336972765,  0.0020640393}, 
    {  0.0000014566,  0.0171751539,  0.0069821306} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0176671395,  0.0000317333}, 
    {  0.0000000000,  1.0000000000, -0.0374576806, -0.0020692396}, 
    {  0.0000000000,  0.0000000000,  0.9668639064, -0.0020633462}, 
    {  0.0000000000,  0.0000000000, -0.0165067698,  0.9930591436} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0176671395, -0.0000317333}, 
    {  0.0000000000,  0.0374576806,  0.0020692396}, 
    {  0.0000000000,  0.0331360936,  0.0020633462}, 
    {  0.0000000000,  0.0165067698,  0.0069408564} 
};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[4][4] = 
{ 
    {  0.9025430213, -0.1597208071,  0.0000000000,  0.0000000000}, 
    {  0.0190143818,  0.9983755056,  0.0000000000,  0.0000000000}, 
    {  0.0012333807,  0.1247562461,  0.8751733190,  0.0000000000}, 
    {  0.0000083851,  0.0012756415,  0.0187240021,  1.0000000000} 
};

double Pos_B_refmodel[4] = 
    {  0.1597208071,  0.0016244944,  0.0000704348,  0.0000003563};

double Pos_C_refmodel[4] = 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

double Perror[4]; 

double PKlp[5]= 
    { -0.0642466272, -0.3899033854, -0.0445410788, -0.5882048253, -0.0590829326};

double PKnp= -1.2426181180; 

double PK1[4]= 
    {  0.0000000000, -0.2388233581,  0.0000000000,  0.0000000000};

double PK2=  0.2446483180; 

double PSr[4]= 
    {  0.0703147637,  0.2985500137,  0.0954481667,  0.4575102408};

double PSi = 0.047547; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
//#define N_STATE    4
