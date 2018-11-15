#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//Q = diag([10 5 1 0.1 0.1]);
//R= 10;
//AA = diag([2 2 1 0.1 0.1]);

// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 22-Dec-2015 18:12:12

#define MRSMC_Control  
#define K_Pos 4
#define Ky_Pos 4
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[4][4] = 
{ 
    {  0.9083936670, -0.0036322661, -0.0100255464,  0.0000315339}, 
    {  0.0190693971,  0.9999630958, -0.0265652220, -0.0019450574}, 
    {  0.0018135651,  0.1812668868,  0.7936886092, -0.0019447604}, 
    {  0.0000123905,  0.0018730633,  0.0020721548,  0.9930863610} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.6235692856,  0.0100255464, -0.0000315339}, 
    {  0.0063355287,  0.0265652220,  0.0019450574}, 
    {  0.0004051707,  0.0250421439,  0.0019447604}, 
    {  0.0000020557,  0.0160547699,  0.0069136390} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0111419426,  0.0000269342}, 
    {  0.0000000000,  1.0000000000, -0.0263537245, -0.0019456428}, 
    {  0.0000000000,  0.0000000000,  0.9752728482, -0.0019446303}, 
    {  0.0000000000,  0.0000000000, -0.0155570424,  0.9931252552} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0111419426, -0.0000269342}, 
    {  0.0000000000,  0.0263537245,  0.0019456428}, 
    {  0.0000000000,  0.0247271518,  0.0019446303}, 
    {  0.0000000000,  0.0155570424,  0.0068747448} 
};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[4][4] = 
{ 
    {  0.9073412822, -0.1105611522,  0.0000000000,  0.0000000000}, 
    {  0.0190622676,  0.9988764758,  0.0000000000,  0.0000000000}, 
    {  0.0012354375,  0.1247779874,  0.8751733190,  0.0000000000}, 
    {  0.0000083954,  0.0012757516,  0.0187240021,  1.0000000000} 
};

double Pos_B_refmodel[4] = 
    {  0.1105611522,  0.0011235242,  0.0000486936,  0.0000002463};

double Pos_C_refmodel[4] = 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

double Perror[4]; 

double PKlp[5]= 
    { -0.0855546488, -0.2859067036, -0.0130470784, -0.1130809297, -0.0173819835};

double PKnp= -0.0868381927; 

double PK1[4]= 
    {  0.0000000000, -0.1715450706,  0.0000000000,  0.0000000000};

double PK2=  0.1773700306; 

double PSr[4]= 
    {  1.0564835180,  4.8593400782,  0.2140209323,  1.2354809785};

double PSi = 0.200165; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
//#define N_STATE    4
