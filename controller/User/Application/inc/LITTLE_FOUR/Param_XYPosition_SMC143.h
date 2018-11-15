#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 25-Dec-2015 15:28:46

#define MRSMC_Control  
#define K_Pos 4
#define Ky_Pos 4
//////////////////////////////Kalman Model////////////////////////////////
double Pos_Kalman_A[4][4] = 
{ 
    {  0.9040764269, -0.0038049557, -0.0083588059,  0.0000406010}, 
    {  0.0190247786,  0.9999613111, -0.0269617202, -0.0019462563}, 
    {  0.0012337215,  0.1248250037,  0.8504594897, -0.0019446034}, 
    {  0.0000083865,  0.0012759894,  0.0026782790,  0.9930867482} 
};

double Pos_Kalman_B[4][3] = 
{ 
    {  0.6532157734,  0.0083588059, -0.0000406010}, 
    {  0.0066419240,  0.0269617202,  0.0019462563}, 
    {  0.0002879496,  0.0247138293,  0.0019446034}, 
    {  0.0000014566,  0.0160457231,  0.0069132518} 
};

double Pos_Kalman_C[4][4] = 
{ 
    {  1.0000000000,  0.0000000000, -0.0093584130,  0.0000367144}, 
    {  0.0000000000,  1.0000000000, -0.0267847147, -0.0019470301}, 
    {  0.0000000000,  0.0000000000,  0.9755946838, -0.0019443128}, 
    {  0.0000000000,  0.0000000000, -0.0155545025,  0.9931256376} 
};

double Pos_Kalman_D[4][3] = 
{ 
    {  0.0000000000,  0.0093584130, -0.0000367144}, 
    {  0.0000000000,  0.0267847147,  0.0019470301}, 
    {  0.0000000000,  0.0244053162,  0.0019443128}, 
    {  0.0000000000,  0.0155545025,  0.0068743624} 
};

//////////////////////////////Reference Model////////////////////////////////
double Pos_A_refmodel[4][4] = 
{ 
    {  0.9031787212, -0.0950934613,  0.0000000000,  0.0000000000}, 
    {  0.0190186923,  0.9990329302,  0.0000000000,  0.0000000000}, 
    {  0.0018103929,  0.1812073736,  0.8187307531,  0.0000000000}, 
    {  0.0000123747,  0.0018727613,  0.0181269247,  1.0000000000} 
};

double Pos_B_refmodel[4] = 
    {  0.0950934613,  0.0009670698,  0.0000618734,  0.0000003140};

double Pos_C_refmodel[4] = 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

double Perror[4]; 

double PKlp[5]= 
    { -0.1212134291, -0.5952853271, -0.0603141931, -0.6117777325, -0.0386462962};

double PKnp= -0.0796979097; 

double PK1[4]= 
    {  0.0000000000, -0.1397990389,  0.0000000000,  0.0000000000};

double PK2=  0.1456239988; 

double PSr[4]= 
    {  1.0963198356,  6.7992255217,  0.7491737161,  7.5145714082};

double PSi = 0.484910; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
//#define N_STATE    4