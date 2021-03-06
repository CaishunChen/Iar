#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 23-Sep-2015 11:44:08

#define MRSMC_Control  
#define K_Pos 4
#define Ky_Pos 4
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[4][4] = 
{ 
    {  0.9497521058, -0.0100027637, -0.0093441201,  0.0002561097}, 
    {  0.0194935459,  0.9998991129, -0.0324074190, -0.0018808579}, 
    {  0.0006409829,  0.0644908006,  0.9085361320, -0.0019119907}, 
    {  0.0000043155,  0.0006520844,  0.0035004403,  0.9931047025} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.4759805090,  0.0093441201, -0.0002561097}, 
    {  0.0048007009,  0.0324074190,  0.0018808579}, 
    {  0.0001053722,  0.0269708531,  0.0019119907}, 
    {  0.0000005300,  0.0158474642,  0.0068952975} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0101777419,  0.0002497971}, 
    {  0.0000000000,  1.0000000000, -0.0322122685, -0.0018859176}, 
    {  0.0000000000,  0.0000000000,  0.9733973827, -0.0019139638}, 
    {  0.0000000000,  0.0000000000, -0.0153117103,  0.9931429624} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0101777419, -0.0002497971}, 
    {  0.0000000000,  0.0322122685,  0.0018859176}, 
    {  0.0000000000,  0.0266026173,  0.0019139638}, 
    {  0.0000000000,  0.0153117103,  0.0068570376} 
};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[4][4] = 
{ 
    {  0.9541588952, -0.0312625132,  0.0000000000,  0.0000000000}, 
    {  0.0195390707,  0.9996849300,  0.0000000000,  0.0000000000}, 
    {  0.0006419938,  0.0644861020,  0.9355069850,  0.0000000000}, 
    {  0.0000043206,  0.0006520607,  0.0193479045,  1.0000000000} 
};

double Pos_B_refmodel[4] = 
    {  0.0312625132,  0.0003150700,  0.0000069129,  0.0000000348};

double Pos_C_refmodel[4] = 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

double Perror[4]; 

double PKlp[5]= 
    { -0.1503734843, -0.4708026145, -0.0218793929, -0.1179025731, -0.0060180532};

double PKnp= -0.5613774164; 

double PK1[4]= 
    {  0.0099311740, -0.0445121373,  0.0000000000,  0.0000000000};

double PK2=  0.0655272072; 

double PSr[4]= 
    {  0.2188608053,  0.6569589161,  0.2685806839,  0.2064503345};

double PSi = 0.010720; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
//#define N_STATE    4
