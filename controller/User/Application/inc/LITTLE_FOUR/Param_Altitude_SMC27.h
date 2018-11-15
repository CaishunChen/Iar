#define INCLUDED_PARAM_ALTITUDE_SMC_H

// Header for FRAME TYPE 
// This header was generated by MATLAB. 
// 21-Oct-2016 11:41:49
//
///////////////////////////////////////////////////////////////////////////////
/*
CtrData.RefModel.Am = { 
    { -18.0815731168, -80.0000000000,  0.0000000000, -90.0000000000}, 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000, 10.0000000000, -10.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000} 
};
CtrData.KalmanParam.Qk =    { 10000.0000000000};
CtrData.KalmanParam.Rk = { 
    { 100.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.1000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.1000000000} 
};
CtrData.SMC.Q= { 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  2.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  2.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000} 
};
CtrData.SMC.R =     {  1.0000000000};
CtrData.SMC.AA = { 
    {  1.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  2.0000000000,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000,  0.0000000000}, 
    {  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000,  0.0000000000} 
};
CtrData.SMC.rou =     { 10.0000000000};
CtrData.SMC.fai =     { -10.0000000000};
*/
///////////////////////////////////////////////////////////////////////////

#define MRSMC_Theta_Control  
#define K_Alt 4
#define Ky_Alt 4
////////////////////////////// Kalman Model////////////////////////////////
double Kalman_A_Alt[4][4] = 
{ 
    {  0.6940242430,  0.0000000000, -0.0397141597, -0.0000482003}, 
    {  0.0165364768,  1.0000000000, -0.0824481168, -0.0157358583}, 
    {  0.0015767223,  0.1812692469,  0.7476840774, -0.0155978206}, 
    {  0.0000101205,  0.0018730753,  0.0011624074,  0.9803541875} 
};

double Kalman_B_Alt[4][4] = 
{ 
    {  0.0043305299,  0.0025147784,  0.0397141597,  0.0000482003}, 
    {  0.0000459097,  0.0002464091,  0.0824481168,  0.0157358583}, 
    {  0.0000029966,  0.0000863683,  0.0710466757,  0.0155978206}, 
    {  0.0000000154,  0.0000014927,  0.0169645173,  0.0196458125} 
};

double Kalman_C_Alt[4][4] = 
{ 
    {  0.9963896087,  0.0000000000, -0.0570164176, -0.0000691998}, 
    { -0.0001858163,  1.0000000000, -0.0814912167, -0.0157346970}, 
    { -0.0000570164,  0.0000000000,  0.9313815922, -0.0155673752}, 
    { -0.0000000692,  0.0000000000, -0.0155673752,  0.9806658492} 
};

double Kalman_D_Alt[4][4] = 
{ 
    {  0.0000000000,  0.0036103913,  0.0570164176,  0.0000691998}, 
    {  0.0000000000,  0.0001858163,  0.0814912167,  0.0157346970}, 
    {  0.0000000000,  0.0000570164,  0.0686184078,  0.0155673752}, 
    {  0.0000000000,  0.0000000692,  0.0155673752,  0.0193341508} 
};

//////////////////////////////Reference Model////////////////////////////////
double Alt_A_d[4][4] = 
{ 
    {  0.6965390214,  0.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0167828859,  1.0000000000,  0.0000000000,  0.0000000000}, 
    {  0.0016630905,  0.1812692469,  0.8187307531,  0.0000000000}, 
    {  0.0000116132,  0.0018730753,  0.0181269247,  1.0000000000} 
};

double Alt_B_d[4] = 
    {  0.0043305299,  0.0000459097,  0.0000029966,  0.0000000154};

double Alt_F[4] = 
    {  0.0000000000, 310.0384686641,  0.0000000000, 348.7932772471};

double Alt_Br= 348.7932772471; 

double A_refmodel_Alt[4][4] = 
{ 
    {  0.6839495929, -1.3365390386, -0.0149278573, -1.5024324371}, 
    {  0.0166936937,  0.9857978369, -0.0001043539, -0.0159713962}, 
    {  0.0016586508,  0.1803414452,  0.8187255982, -0.0010435390}, 
    {  0.0000115949,  0.0018683044,  0.0181269035,  0.9999946336} 
};

double B_refmodel_Alt[4] = 
    {  1.5024324371,  0.0159713962,  0.0010435390,  0.0000053664};

double C_refmodel_Alt[4] = 
    {  0.0000000000,  0.0000000000,  0.0000000000,  1.0000000000};

//////////////////////////////// controller ////////////////////////////////
double error_Alt[5]; 

double Klp_Alt[5]= 
    { -21.3177019246, -342.1332473123, -105.2078156704, -408.5432122327, -352.5833277528};

double Knp_Alt = -152.5833277526; 

double K1_Alt[4]= 
    {  0.0000000000, -310.0384686641,  0.0000000000, -348.7932772471};

double K2_Alt = 348.7932772471; 

double S_Alt[5]= 
    {  0.0368187559,  0.5000809195,  0.1195106119,  0.9995200655,  1.0000000000};

double Sr_Alt[4]= 
    {  0.0368187559,  0.5000809195,  0.1195106119,  0.9995200655};

double Si_Alt = 1.000000; 

double X_Alt_Ref[4] , X_Alt_smc[4]; 
