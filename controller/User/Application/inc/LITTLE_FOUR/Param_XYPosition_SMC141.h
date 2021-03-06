#define INCLUDED_PARAM_XYPOSITION_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 25-Dec-2015 14:38:05

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
    {  0.0012335220,  0.1247847526,  0.8751733190,  0.0000000000}, 
    {  0.0000083857,  0.0012757858,  0.0187240021,  1.0000000000} 
};

double Pos_B_refmodel[4] = 
    {  0.0950934613,  0.0009670698,  0.0000419283,  0.0000002121};

double Pos_C_refmodel[4] = 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000};

double Perror[4]; 

double PKlp[5]= 
    { -0.0761598552, -0.3007461336, -0.0125188634, -0.1085354869, -0.0166850681};

double PKnp= -0.0833613352; 

double PK1[4]= 
    {  0.0000000000, -0.1397990389,  0.0000000000,  0.0000000000};

double PK2=  0.1456239988; 

double PSr[4]= 
    {  1.0481405925,  4.8790406129,  0.2140207626,  1.2352704911};

double PSi = 0.200154; 

double X_PosX_Ref[4] , X_PosX_smc[4]; 
double X_PosY_Ref[4] , X_PosY_smc[4]; 
#define N_STATE    4
