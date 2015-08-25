#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP
#include "altitude.h"

volatile float exInt, eyInt, ezInt;  
volatile float integralFBx,integralFBy,integralFBz;
volatile float q0, q1, q2, q3; 
volatile float qa0, qa1, qa2, qa3;
volatile float integralFBhand,handdiff;
volatile double halftime ;
volatile uint32_t lastUpdate, now; 
volatile uint16_t sysytem_time_ms=0;
volatile float IMU_Pitch, IMU_Roll, IMU_Yaw;
volatile float  IMU_GYROx, IMU_GYROy, IMU_GYROz;
volatile unsigned char IMU_inited = 0;
volatile uint16_t imu_clce =0;
volatile int16_t accgyroval[6];
float acc_vector = 0;  
#define M_PI  (float)3.1415926535 
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

float LPF_2nd(float newData)
{
	float lpf_2nd_data;
	static float lpf_2nd_preout=0;
	static float lpf_2nd_lastout=0;
	
	lpf_2nd_data = newData * 0.1883633f + lpf_2nd_lastout * 1.023694f - lpf_2nd_preout * 0.2120577f;
	lpf_2nd_preout = lpf_2nd_lastout;
	lpf_2nd_lastout = lpf_2nd_data;
	
	return lpf_2nd_data;
}
void IMU_init(void)
{	 
        init_mpu6050();
        delay_ms(50);
        init_mpu6050();
        delay_ms(50);
        hmc5883Setup();
        delay_ms(50);
        hmc5883Setup();
        delay_ms(50);
	// initialize quaternion
  	q0 = 1.0f;  
  	q1 = 0.0f;
  	q2 = 0.0f;
  	q3 = 0.0f;
	qa0 = 1.0f;  
  	qa1 = 0.0f;
  	qa2 = 0.0f;
  	qa3 = 0.0f;
  	exInt = 0.0;
  	eyInt = 0.0;
  	ezInt = 0.0;
        integralFBx = 0.0;
        integralFBy = 0.0; 
        integralFBz = 0.0;
        lastUpdate = micros();
  	now = micros();
}

int16_t	ACC_X_BUF[10],ACC_Y_BUF[10],ACC_Z_BUF[10],GRYO_X_BUF[20],GRYO_Y_BUF[20],GRYO_Z_BUF[20];	
int32_t ACC_AVG_X=0;
int32_t ACC_AVG_Y=0;
int32_t ACC_AVG_Z=0;

void  _fliter(float x1,float x2,float x3)
{	
	static uint8_t filter_cnt=0;
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

	ACC_X_BUF[filter_cnt] = x1;
	ACC_Y_BUF[filter_cnt] = x2;
	ACC_Z_BUF[filter_cnt] = x3;
	for(i=0;i<4;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	
	}
	ACC_AVG_X = temp1 / 4;
	ACC_AVG_Y = temp2 / 4;
	ACC_AVG_Z = temp3 / 4;
	
		
	filter_cnt++;
	if(filter_cnt==4)	filter_cnt=0;
}

#define new_weight 0.4f
#define old_weight 0.6f


#define new_weight1 0.5f
#define old_weight1 0.5f




void IMU_getValues(float * values) {  
	
	static  float lastacc[3]= {0,0,0};
	static  float lastgyro[3]= {0,0,0};
	int i;
	int16_t X1=0,X2=0,X3=0;
	
      mpu6050GetData1(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5] );
    for(i = 0; i<6; i++)
    {
      if(i < 3)
      {
        values[i] = (float) accgyroval[i] * new_weight +lastacc[i] * old_weight ;
	lastacc[i]=values[i];	    
      }  
      else
      { 
	      //values[i] = (float) accgyroval[i]/16.4 ;
	      values[i] = (float) accgyroval[i]/16.4*new_weight1 +lastgyro[i-3]*old_weight1 ;
              lastgyro[i-3]=values[i];
		
		
      }
      
    }
   
   
    //values[0]=ACC_AVG_X;
    //values[1]=ACC_AVG_Y;
    //values[2]=ACC_AVG_Z;
  HMC58X3_mgetValues(&values[6]);	
	IMU_GYROx = values[3];
	IMU_GYROy = values[4];
	IMU_GYROz = values[5];
	
}

void IMU_getValues1(float * values1)
	{  
	int16_t accgyroval[6];

	int i;
	
      mpu6050GetData1(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5] );
    for(i = 0; i<6; i++) {
      if(i < 3) {
       
      }
      else  {
       values1[i] = ((float) accgyroval[i]) / 16.4f; 
		
		
      }
      }
		
		
		
			
  HMC58X3_mgetValues(&values1[6]);	
	IMU_GYROx = values1[3];
	IMU_GYROy = values1[4];
	IMU_GYROz = values1[5];
	
}
float halfT;
#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.03f   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  volatile float norm;
  volatile float hx, hy, hz, bx, bz;
  volatile float vx, vy, vz, wx, wy, wz;
  volatile float ex, ey, ez;
  float temp0,temp1,temp2,temp3;
  float temp;
  // 
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
  now = micros(); 
  if(now < lastUpdate){
	   halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);//i got changed
		 lastUpdate = now;
		//return ;
    }
  else{
		halfT =  ((float)(now - lastUpdate) / 2000000.0f);//this place have a problem
  	}
  halftime = halfT;
  lastUpdate = now;	

 

  norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  // compute reference direction of flux
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);  
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
  exInt = exInt + ex * Ki * halfT;
  eyInt = eyInt + ey * Ki * halfT;	
  ezInt = ezInt + ez * Ki * halfT;

  // adjusted gyroscope measurements
  gx = gx + (Kp*ex + exInt);
  gy = gy + (Kp*ey + eyInt);
  gz = gz + (Kp*ez + ezInt);

  }

  // integrate quaternion rate and normalise
  temp0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  temp1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  temp2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  temp3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // normalise quaternion
  norm = invSqrt(temp0*temp0 + temp1*temp1 + temp2*temp2 + temp3*temp3);
  q0 = temp0 * norm;
  q1 = temp1 * norm;
  q2 = temp2 * norm;
  q3 = temp3 * norm;
}




#define twoKpDef  (1.0f ) // 2 * proportional gain
#define twoKiDef  (0.2f) // 2 * integral gain

void FreeIMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
  float norm;
//  float hx, hy, hz, bx, bz;
  float vx, vy, vz;
  float ex, ey, ez;
  float temp0,temp1,temp2,temp3;

  float q0q0 = qa0*qa0;
  float q0q1 = qa0*qa1;
  float q0q2 = qa0*qa2;
  float q0q3 = qa0*qa3;
  float q1q1 = qa1*qa1;
  float q1q2 = qa1*qa2;
  float q1q3 = qa1*qa3;
  float q2q2 = qa2*qa2;   
  float q2q3 = qa2*qa3;
  float q3q3 = qa3*qa3;          

  norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;   
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3; 
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

if(ex != 0.0f && ey != 0.0f && ez != 0.0f){

  integralFBx +=  ex * twoKiDef * halftime;
  integralFBy +=  ey * twoKiDef * halftime;	
  integralFBz +=  ez * twoKiDef * halftime;

  gx = gx + twoKpDef*ex + integralFBx;
  gy = gy + twoKpDef*ey + integralFBy;
  gz = gz + twoKpDef*ez + integralFBz;

  }
  // integrate quaternion rate and normalise
  temp0 = qa0 + (double)(-qa1*gx - qa2*gy - qa3*gz)*halftime;
  temp1 = qa1 + (double)(qa0*gx + qa2*gz - qa3*gy)*halftime;
  temp2 = qa2 + (double)(qa0*gy - qa1*gz + qa3*gx)*halftime;
  temp3 = qa3 + (double)(qa0*gz + qa1*gy - qa2*gx)*halftime;  
  
  // normalise quaternion
  norm = invSqrt(temp0*temp0 + temp1*temp1 + temp2*temp2 + temp3*temp3);
  qa0 = temp0 * norm;
  qa1 = temp1 * norm;
  qa2 = temp2 * norm;
  qa3 = temp3 * norm;
}
float mygetqval[9];	
float mygetqval1[9];
void IMU_getQ(float * q) {

   IMU_getValues(mygetqval);	 
 
   IMU_AHRSupdate(mygetqval[3] * M_PI/180.0f, mygetqval[4] * M_PI/180.0f, mygetqval[5] * M_PI/180.0f,
   mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);

   FreeIMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
   mygetqval[0],mygetqval[1], mygetqval[2]);

  q[0] = qa0; 
  q[1] = qa1;
  q[2] = qa2;
  q[3] = qa3;
}
void IMU_getQ1(float * q) {

   IMU_getValues1(mygetqval1);	 
  
   IMU_AHRSupdate(mygetqval1[3] * M_PI/180, mygetqval1[4] * M_PI/180, mygetqval1[5] * M_PI/180,
   mygetqval1[0], mygetqval1[1], mygetqval1[2], mygetqval[6], mygetqval[7], mygetqval[8]);

   FreeIMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
   mygetqval1[0], mygetqval1[1], mygetqval1[2]);

  q[0] = qa0; //¡¤¦Ì??¦Ì¡À?¡ã?¦Ì	FreeIMU_AHRSupdate ????3?¨¤¡ä¦Ì????a¨ºy ¡À?¨®?¦Ì?
  q[1] = qa1;
  q[2] = qa2;
  q[3] = qa3;
}
float safe_asin(float v)
{
 // if (isnan(v)) 
  //{
  //  return 0.0f;
  //}
  if (v >= 1.0f) 
  {
    return M_PI/2;
  }
  if (v <= -1.0f)
  {
    return -M_PI/2;
  }
    return asin(v);
}
void IMU_getYawPitchRoll(float * angles) {
  static float q[4]; 
	
  IMU_getQ(q); 

  IMU_Roll = angles[2] = (atan2(2.0f*(q[0]*q[1] + q[2]*q[3]),
                       1 - 2.0f*(q[1]*q[1] + q[2]*q[2])))* 180/M_PI;//;
    // we let safe_asin() handle the singularities near 90/-90 in pitch
  IMU_Pitch = angles[1] = -safe_asin(2.0f*(q[0]*q[2] - q[3]*q[1]))* 180/M_PI;

  IMU_Yaw = angles[0] = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/M_PI; // yaw+= IMU_GYROz*0.0044;//

//	if(IMU_Yaw <0)IMU_Yaw +=360.0f; 
			
	

}
void IMU_getYawPitchRoll1(float * angles) {
  static float q[4];
	
  IMU_getQ1(q); 

  IMU_Roll = angles[2] = (atan2(2.0f*(q[0]*q[1] + q[2]*q[3]),
                       1 - 2.0f*(q[1]*q[1] + q[2]*q[2])))* 180/M_PI;//;
    // we let safe_asin() handle the singularities near 90/-90 in pitch
  IMU_Pitch = angles[1] = -safe_asin(2.0f*(q[0]*q[2] - q[3]*q[1]))* 180/M_PI;

  IMU_Yaw = angles[0] = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/M_PI; // yaw+= IMU_GYROz*0.0044;//

//	if(IMU_Yaw <0)IMU_Yaw +=360.0f; 
	

}