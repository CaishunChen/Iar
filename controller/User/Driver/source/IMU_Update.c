#include "IMU_Update.h"
#include "BNO055.h"

float PITCH = 0.0f,ROLL = 0.0f,YAW = 0.0f;
float Beta_K = 0.03f;
float BETADEF = 0.04f;
float Acceleration_Length = 0.0f;
uint8_t IMU_init_flag = 0;

extern d_Senser_Data  *pSenser_Data;

struct _Attitude_Quad
{
  float q[4];
};

struct _Attitude_Quad att =
{
  {1.0f, 0.0f, 0.0f, 0.0f}
};

float constrain_float(float value, const float min_val, const float max_val)
{
  if(value>=max_val)  value=max_val;
  if(value<=min_val)  value=min_val;
  return value;
}



float R_DCM[3][3];
float IMU_ACC[3];

float Get_Acceleration_Length(void)
{
  float Origion_acc[3];
  
  R_DCM[0][0]= cos(-YAW)*cos(-PITCH);
  R_DCM[1][0]=-sin(-YAW)*cos(ROLL)+cos(-YAW)*sin(ROLL)*sin(-PITCH);
  R_DCM[2][0]= sin(-YAW)*sin(ROLL)+cos(-YAW)*cos(ROLL)*sin(-PITCH);
  
  R_DCM[0][1]= sin(-YAW)*cos(-PITCH);
  R_DCM[1][1]= cos(-YAW)*cos(ROLL)+sin(-YAW)*sin(ROLL)*sin(-PITCH);
  R_DCM[2][1]=-cos(-YAW)*sin(ROLL)+sin(-YAW)*cos(ROLL)*sin(-PITCH);
  
  R_DCM[0][2]=-sin(-PITCH);
  R_DCM[1][2]= sin(ROLL)*cos(-PITCH);
  R_DCM[2][2]= cos(ROLL)*cos(-PITCH);
  
  IMU_ACC[0]= -pSenser_Data->ADIXacc ;
  IMU_ACC[1]=  pSenser_Data->ADIYacc ;
  IMU_ACC[2]=  pSenser_Data->ADIZacc ;
  
  for(uint8_t j=0;j<3;j++)
  {
    Origion_acc[j]=0;
    for(uint8_t k=0;k<3;k++)
      Origion_acc[j] = Origion_acc[j]+R_DCM[k][j]* IMU_ACC[k];
  }
  
  Origion_acc[2] = Origion_acc[2] + 9.8f;
  Origion_acc[0] *= 100;
  Origion_acc[1] *= 100;
  Origion_acc[2] *= 100;
  
  Acceleration_Length = sqrtf(Origion_acc[0] * Origion_acc[0] + Origion_acc[1] * Origion_acc[1]
                              + Origion_acc[2] * Origion_acc[2]);
  
  return Acceleration_Length;
}


float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  
  return y;
}

void MadgwickAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
  float initialRoll, initialPitch;
  float cosRoll, sinRoll, cosPitch, sinPitch;
  float magX, magY;
  float initialHdg, cosHeading, sinHeading;
  
  initialRoll = atan2f(ay, az);//根据加速度算角度
  initialPitch = atan2f(-ax, az);
  
  cosRoll = cosf(initialRoll);
  sinRoll = sinf(initialRoll);
  cosPitch = cosf(initialPitch);
  sinPitch = sinf(initialPitch);
  
  magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
  
  magY = my * cosRoll - mz * sinRoll;
  
  initialHdg = atan2f(-magY, magX);
  
  cosRoll = cosf(initialRoll * 0.5f);
  sinRoll = sinf(initialRoll * 0.5f);
  
  cosPitch = cosf(initialPitch * 0.5f);
  sinPitch = sinf(initialPitch * 0.5f);
  
  cosHeading = cosf(initialHdg * 0.5f);
  sinHeading = sinf(initialHdg * 0.5f);
  
  att.q[0] = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
  att.q[1] = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
  att.q[2] = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
  att.q[3] = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
}



void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float mx,float my,float mz)
{
  
  if(!IMU_init_flag)
  {
    MadgwickAHRSinit(ax,ay,az,mx,my,mz);
    IMU_init_flag = 1;
    return;
  }
  
  float recipNorm;					
  float s0, s1, s2, s3;					
  float qDot1, qDot2, qDot3, qDot4;			
  //float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  float delta;
  float dt = 0.05f;//跟新频率
  
  float hx, hy;  
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; 
  
  float Tmep_Acce_Length = 0.0f;
  
  Get_Acceleration_Length();
  
  qDot1 = 0.5f * (-att.q[1] * gx - att.q[2] * gy - att.q[3] * gz);
  qDot2 = 0.5f * (att.q[0] * gx + att.q[2] * gz - att.q[3] * gy);
  qDot3 = 0.5f * (att.q[0] * gy - att.q[1] * gz + att.q[3] * gx);
  qDot4 = 0.5f * (att.q[0] * gz + att.q[1] * gy - att.q[2] * gx);
  
  
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    recipNorm=invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);  
    mx *= recipNorm;  
    my *= recipNorm;  
    mz *= recipNorm;  
    
    //		_2q0 = 2.0f * att.q[0];
    //		_2q1 = 2.0f * att.q[1];
    //		_2q2 = 2.0f * att.q[2];
    //		_2q3 = 2.0f * att.q[3];
    //		_4q0 = 4.0f * att.q[0];
    //		_4q1 = 4.0f * att.q[1];
    //		_4q2 = 4.0f * att.q[2];
    //		_8q1 = 8.0f * att.q[1];
    //		_8q2 = 8.0f * att.q[2];
    //		q0q0 = att.q[0] * att.q[0];
    //		q1q1 = att.q[1] * att.q[1];
    //		q2q2 = att.q[2] * att.q[2];
    //		q3q3 = att.q[3] * att.q[3];
    
    _2q0mx = 2.0f * att.q[0] * mx;  
    _2q0my = 2.0f * att.q[0] * my;  
    _2q0mz = 2.0f * att.q[0] * mz;  
    _2q1mx = 2.0f * att.q[1] * mx;  
    _2q0 = 2.0f * att.q[0];  
    _2q1 = 2.0f * att.q[1];  
    _2q2 = 2.0f * att.q[2];  
    _2q3 = 2.0f * att.q[3];  
    _2q0q2 = 2.0f * att.q[0] * att.q[2];  
    _2q2q3 = 2.0f * att.q[2] * att.q[3];  
    q0q0 = att.q[0] * att.q[0];  
    q0q1 = att.q[0] * att.q[1];  
    q0q2 = att.q[0] * att.q[2];  
    q0q3 = att.q[0] * att.q[3];  
    q1q1 = att.q[1] * att.q[1];  
    q1q2 = att.q[1] * att.q[2];  
    q1q3 = att.q[1] * att.q[3];  
    q2q2 = att.q[2] * att.q[2];  
    q2q3 = att.q[2] * att.q[3];  
    q3q3 = att.q[3] * att.q[3];  
    
    //		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    //		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * att.q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    //		s2 = 4.0f * q0q0 * att.q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    //		s3 = 4.0f * q1q1 * att.q[3] - _2q1 * ax + 4.0f * q2q2 * att.q[3] - _2q2 * ay;
    
    
    hx = mx * q0q0 - _2q0my * att.q[3] + _2q0mz * att.q[2] + mx * q1q1 + _2q1 * my * att.q[2] + _2q1 * mz * att.q[3] - mx * q2q2 - mx * q3q3;  
    hy = _2q0mx * att.q[3] + my * q0q0 - _2q0mz * att.q[1] + _2q1mx * att.q[2] - my * q1q1 + my * q2q2 + _2q2 * mz * att.q[3] - my * q3q3;  
    _2bx = sqrtf(hx * hx + hy * hy);  
    _2bz = -_2q0mx * att.q[2] + _2q0my * att.q[1] + mz * q0q0 + _2q1mx * att.q[3] - mz * q1q1 + _2q2 * my * att.q[3] - mz * q2q2 + mz * q3q3;  
    _4bx = 2.0f * _2bx;  
    _4bz = 2.0f * _2bz; 
    
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * att.q[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * att.q[3] + _2bz * att.q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * att.q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);  
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * att.q[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * att.q[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * att.q[2] + _2bz * att.q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * att.q[3] - _4bz * att.q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);  
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * att.q[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * att.q[2] - _2bz * att.q[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * att.q[1] + _2bz * att.q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * att.q[0] - _4bz * att.q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);  
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * att.q[3] + _2bz * att.q[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * att.q[0] + _2bz * att.q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * att.q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);  
    
    
    recipNorm=invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    
    Tmep_Acce_Length = constrain_float(Acceleration_Length,0.0f,1000.0f);
    
    BETADEF = Beta_K - 0.02 * Tmep_Acce_Length * 0.001f;
    
    qDot1 -= BETADEF * s0;
    qDot2 -= BETADEF * s1;
    qDot3 -= BETADEF * s2;
    qDot4 -= BETADEF * s3;
  }
  
  delta = (dt * gx) * (dt * gx) + (dt * gy) * (dt * gy) + (dt * gz) * (dt * gz);
  att.q[0] = (1.0f - delta / 8.0f) * att.q[0] + qDot1 * dt;
  att.q[1] = (1.0f - delta / 8.0f) * att.q[1] + qDot2 * dt;
  att.q[2] = (1.0f - delta / 8.0f) * att.q[2] + qDot3 * dt;
  att.q[3] = (1.0f - delta / 8.0f) * att.q[3] + qDot4 * dt;
  
  recipNorm = invSqrt(att.q[0] * att.q[0] + att.q[1] * att.q[1] + att.q[2] * att.q[2] + att.q[3] * att.q[3]);
  att.q[0] *= recipNorm;
  att.q[1] *= recipNorm;
  att.q[2] *= recipNorm;
  att.q[3] *= recipNorm;
  
  ROLL = atan2f(2.0f * att.q[2] * att.q[3] + 2.0f * att.q[0] * att.q[1], -2.0f * att.q[1] * att.q[1] - 2.0f * att.q[2]* att.q[2] + 1.0f);
  PITCH = asinf(2.0f * att.q[0]* att.q[2]-2.0f * att.q[1] * att.q[3]);
  YAW = atan2f(2.0f * att.q[0] * att.q[3] + 2 * att.q[1] * att.q[2], 1.0f - 2.0f * att.q[2] * att.q[2] - 2.0f * att.q[3] * att.q[3]);
  
  pSenser_Data->Roll = ROLL;
  pSenser_Data->Pitch = PITCH;  
  pSenser_Data->Yaw  = YAW;
}
