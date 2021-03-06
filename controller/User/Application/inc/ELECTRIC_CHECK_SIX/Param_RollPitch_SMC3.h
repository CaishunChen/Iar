#define INCLUDED_PARAM_ROLLPITCH_SMC_H

//////////////////////////////////////////////////////////
//
// This header was generated by MATLAB. 
//////////////////////////////////////////////////////////

// 27-Jul-2015 15:24:09

#define MRSMC_Control  
#define K_Roll 3
#define Ky_Roll 3
//////////////////////////////Kalman Model////////////////////////////////
double Kalman_A[3][3] = 
{ 
    {  0.5383882264, -5.1452321852,  2.3709194703}, 
    {  0.0151511587,  0.9047725194, -0.1669262492}, 
    {  0.0001668933,  0.0166953813,  0.9076601640} 
};

double Kalman_B[3][3] = 
{ 
    {  0.0310468635, -0.1664638728, -2.3709194703}, 
    {  0.0003419879,  0.0367180023,  0.1669262492}, 
    {  0.0000023882,  0.0028960362,  0.0923398360} 
};

double Kalman_C[3][3] = 
{ 
    {  1.0000000000, -0.0652246220,  2.2907986757}, 
    {  0.0000000000,  0.9620497786, -0.2141651973}, 
    {  0.0000000000, -0.0021416520,  0.9114736449} 
};

double Kalman_D[3][3] = 
{ 
    {  0.0000000000,  0.0652246220, -2.2907986757}, 
    {  0.0000000000,  0.0379502214,  0.2141651973}, 
    {  0.0000000000,  0.0021416520,  0.0885263551} 
};

//////////////////////////////Reference Model////////////////////////////////
double A_refmodel[3][3] = 
{ 
    {  0.7037930877, -1.9146754029, -4.8366079350}, 
    {  0.0169408334,  0.9799286720, -0.0511837292}, 
    {  0.0001792775,  0.0198630568,  0.9996491982} 
};

double B_refmodel[3] = 
    {  4.8366079350,  0.0511837292,  0.0003508018};

double C_refmodel[3] = 
    {  0.0000000000,  0.0000000000,  1.0000000000};

double error[3]; 
double Klp[3]= 
    { -9.3266696461, -117.0421008295, -448.7941055238};

double Knp= -22.2667205791; 

double K1[3]= 
    {  5.0291202321, 117.4053736459, -139.3266603032};

double K2= 139.3266603032; 

double S[3]= 
    {  0.4383306615,  5.8787431315, 24.0114048120};

double X_Roll_Ref[3] , X_Roll_smc[3] , theta_error_roll; 
double X_Pitch_Ref[3] , X_Pitch_smc[3] , theta_error_pitch; 
