#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 09-May-2015 10:44:40

#define MRSMC_Control  
#define K_Pos 3
#define Ky_Pos 3
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[3][3] = 
{ 
    {  0.9768852524, -0.0071157636,  0.0000307333}, 
    {  0.0197681261,  0.9937468203, -0.0076384893}, 
    {  0.0001984517,  0.0144198017,  0.9836794905} 
};

double Pos_Kalman_B[3][3] = 
{ 
    {  0.1815592845,  0.0019425449, -0.0000307333}, 
    {  0.0018226693,  0.0062012458,  0.0076384893}, 
    {  0.0000121748,  0.0055798514,  0.0163205095} 
};

double Pos_Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0020211384, -0.0000089912}, 
    {  0.0000000000,  0.9938383883, -0.0076387082}, 
    {  0.0000000000, -0.0054562202,  0.9838322638} 
};

double Pos_Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0020211384,  0.0000089912}, 
    {  0.0000000000,  0.0061616117,  0.0076387082}, 
    {  0.0000000000,  0.0054562202,  0.0161677362} 
};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[3][3] = 
{ 
    {  0.9541588952, -0.0312625132,  0.0000000000}, 
    {  0.0195390707,  0.9996849300,  0.0000000000}, 
    {  0.0001969187,  0.0199978914,  1.0000000000} 
};

double Pos_B_refmodel[3] = 
    {  0.0312625132,  0.0003150700,  0.0000021086};

double Pos_C_refmodel[3] = 
    {  0.0000000000,  1.0000000000,  0.0000000000};

double Perror[3]; 

double PKlp[4]= 
    { -0.4508444881, -1.4495919917, -1.4065027748, -0.2120056986};

double PKnp= -0.0778369236; 

double PK1[3]= 
    { -0.1266634204, -0.1457142941,  0.0000000000};

double PK2=  0.1742075696; 

double PSr[3]= 
    {  1.3988185289,  6.2537774689,  8.3540038880};

double PSi = 1.361858; 

double X_PosX_Ref[3] , X_PosX_smc[3]; 
double X_PosY_Ref[3] , X_PosY_smc[3]; 
#define N_STATE    3
