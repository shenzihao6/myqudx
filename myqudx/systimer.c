#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP

#include "systimer.h"
float acc_gz_values=0;
volatile uint8_t  Fly_Mode = default_quad,yaw_lock = 0; 
volatile int16_t  Yaw_DIRECT = 1;
int16_t THROTTLE,PID_ROLL,PID_PITCH,PID_YAW;
int16_t PWM_Offset_Roll,PWM_Offset_Pitch,PWM_Offset_Yaw;
volatile float PID_dt = 0;
volatile float Quad_Pitch,Quad_Roll,Quad_Raw,Quad_THR;
float  Target_Roll,Target_Pitch,Target_Yaw,Tartget_hight = 1.5;
volatile uint8_t IsAutoHigh = 0;  
volatile uint8_t Quadrotor_Mode = Quad_Manual; 	
extern float ypr[3];




struct Quad_PID  PID_NULL[10];
struct Quad_PID 
       no_pid_temp,
       Stabilize_Roll,  
       RollRate,		 
       Stabilize_Pitch, 
       PitchRate,		 
       Stabilize_Yaw,	 
       YawRate,          
       AutoHigh_THR,	  
       Climb,           
       Position_Hold,	  
       Position_Speed;	  

float GPS_PITCH=0,GPS_ROLL=0;
int16_t Auto_High_PID(float TargetHigh,uint8_t isRate);
void Roll_Pitch_Yaw_RatePID(float Rate_roll,float Rate_pitch,float Rate_yaw);
void Roll_Pitch_Yaw_AnglePID(float Angle_roll,float Angle_pitch,float Angle_yaw);
float Quadrotor_PID(float err,float Differ_in,struct Quad_PID *PID);
void PID_Reset_Integ(struct Quad_PID *PID);
float Get_Yaw_Error(float set,float currt);
int16_t Pitch_Roll_PID(struct Quad_PID *PID,int16_t gyro);
int16_t Thr_High_PID(void);
int16_t Yaw_PID(float gyro);
void Position_Hold_Reset(void);
void GPS_Position_Hold(void);



unsigned char Read_Mode(void)
{	 
	
	return Quad_Level_Lock;	
}


  void Quadrotor_Motor_Update(void) 
  {
  	static int16_t calcount=0;
 	 float ftemp;
  	static unsigned char mode;
  	static uint32_t last_time = 0,now_time;

  if(last_time == 0)
  {
  	last_time = micros();
	return ;
  }
  now_time = micros(); 
  if(now_time < last_time)
  {
  	last_time = now_time ;
  	return ;
  }
  PID_dt = (float)(now_time - last_time);
  PID_dt /= 1000000.0f;   //to see the time is right or not
  
  if(RC_Data.THROTTLE >1050)
  {
	if(yaw_lock != 1)
	{
	  yaw_lock = 1;
	  Target_Yaw = ypr[0];
	}
  }
  else
  {
	yaw_lock = 0;
  }
      
	Quad_THR = (int16_t)(RC_Data.THROTTLE); 
	Target_Roll = (int16_t)(RC_Data.ROLL - PWM_Input_Offset);
	Target_Pitch = (int16_t)(RC_Data.PITCH - PWM_Input_Offset);

			
	if((RC_Data.YAW > (int16_t)(PWM_Input_Offset+100))||(RC_Data.YAW < (int16_t)(PWM_Input_Offset-100)))
	{
		ftemp = (float)(RC_Data.YAW - PWM_Input_Offset); //[-500,+500]
		Target_Yaw += (ftemp / 500.0f)*0.2f; //[-0.2,+0.2] 
		if(Target_Yaw >180.0f)
		{
			Target_Yaw = Target_Yaw-360.0f;	//譨[-180.0,+180.0]
		}
		else if	(Target_Yaw <-180.0f)
		{
	 		Target_Yaw = 360.0f + Target_Yaw;
		}		
        }

		
	Roll_Pitch_Yaw_AnglePID( Target_Roll/10.0f , Target_Pitch/10.0f , Target_Yaw);
		
	if(RC_Data.THROTTLE > (int16_t)(MINTHROTTLE+(MAXTHROTTLE-MINTHROTTLE)/10))
	{
		 	
		 	
	}

	PID_PITCH = PitchRate.PID_out;
	PID_ROLL = RollRate.PID_out;
	PID_YAW = YawRate.PID_out;
	THROTTLE = Quad_THR;
		
       PWM_Control();
		

}

void Roll_Pitch_Yaw_AnglePID(float Angle_roll,float Angle_pitch,float Angle_yaw){
float RateTarget ,yaw_error;
	static U8 add=0;
	add++;
	//ROLL
	if(add==2)
         {	
		pidSetTarget_Measure(&Stabilize_Roll, Angle_roll*100.0f,ypr[2]*100.0f);	 //目标角度
		Stabilize_Roll.merror = Math_fConstrain(Stabilize_Roll.merror,-4500.0f,+4500.0f);
		RateTarget = pidUpdate(&Stabilize_Roll ,ypr[2]*100.0f , PID_dt);
		pidSetTarget(&RollRate, RateTarget);
		//Math_fConstrain(RollRate.RateTarget,-150.0f,+150.0f)//角度限幅据说可以减少震荡
		
	}	
	pidUpdate(&RollRate ,IMU_GYROx*100.0f , PID_dt);
	RollRate.PID_out *= 0.1f;
	RollRate.PID_out = Math_fConstrain(RollRate.PID_out,-150.0f,+150.0f);  //限制控制PWM信号的幅度,限幅的大小可以适当的改变
	//PITCH
	if(add==2)
         {	
		pidSetTarget_Measure(&Stabilize_Pitch, Angle_pitch*100.0f,-ypr[1]*100.0f);
		Stabilize_Pitch.merror = Math_fConstrain(Stabilize_Pitch.merror,-4500.0f,+4500.0f);
		RateTarget = pidUpdate(&Stabilize_Pitch ,-ypr[1]*100.0f , PID_dt);
		pidSetTarget(&PitchRate, RateTarget);
		//Math_fConstrain(PitchRate.RateTarget,-150.0f,+150.0f)//角度限幅据说可以减少震荡
	 }	
	
	pidUpdate(&PitchRate ,IMU_GYROy*100.0f , PID_dt);
	PitchRate.PID_out *= 0.1f;
	PitchRate.PID_out = Math_fConstrain(PitchRate.PID_out,-150.0f,+150.0f);  //限制控制PWM信号的幅度,限幅的大小可以适当的改变
	//YAW
	if(add==2)
	{	
		pidSetTarget(&Stabilize_Yaw, Angle_yaw*100.0f);
		pidSetMeasured(&Stabilize_Yaw, ypr[0]*100.0f);	 
		yaw_error =Get_Yaw_Error(Angle_yaw , ypr[0]);	//目标角度
		yaw_error *= 100.0f;
		yaw_error = Math_fConstrain(yaw_error,-4500.0f,+4500.0f);
		RateTarget = pidUpdate_err(&Stabilize_Yaw ,yaw_error, PID_dt);
		pidSetTarget(&YawRate, RateTarget);
		//Math_fConstrain(YawRate.RateTarget,-150.0f,+150.0f)//角度限幅据说可以减少震荡
		add=0;
	}
	pidUpdate(&YawRate ,-IMU_GYROz*100.0f , PID_dt);
	YawRate.PID_out *= 0.1f;
	YawRate.PID_out = Math_fConstrain(YawRate.PID_out,-150.0f,+150.0f);  //限制控制PWM信号的幅度,限幅的大小可以适当的改变

}

void Initial_Math(void)
{
	
	RollRate.Kp=RollRate.Kp/100.0f;//all reduce 100;
	RollRate.Ki=RollRate.Ki/1000.0f;
	RollRate.Kd=RollRate.Kd/100.0f;
	PitchRate.Kp=PitchRate.Kp/100.0f;
	PitchRate.Ki=PitchRate.Ki/1000.0f;
	PitchRate.Kd=PitchRate.Kd/100.0f;
	YawRate.Kp=YawRate.Kp/100.0f;
	YawRate.Ki=YawRate.Ki/1000.0f;
	YawRate.Kd=YawRate.Kd/100.0f;
	Stabilize_Roll.Kp=Stabilize_Roll.Kp/100.0f;
        Stabilize_Roll.Ki=0.0f;
	Stabilize_Roll.Kd=0.0f;
	Stabilize_Pitch.Kp=Stabilize_Pitch.Kp/100.0f;
	Stabilize_Pitch.Ki=0.0f;
	Stabilize_Pitch.Kd=0.0f;
	Stabilize_Yaw.Kp=Stabilize_Yaw.Kp/100.0f;
	Stabilize_Yaw.Ki=0.0f;
	Stabilize_Yaw.Kd=0.0f;
	pidSetLowPassEnable(&RollRate);	
	pidSetLowPassEnable(&PitchRate);
	pidSetLowPassEnable(&YawRate);
	pidSetLowPassEnable(&Climb);
	pidSetLowPassEnable(&AutoHigh_THR);
	pidSetIntegralLimit(&Climb , 100.0f);
	pidSetIntegralLimit(&AutoHigh_THR , 10.0f);
	pidSetIntegralLimit(&RollRate,100.0f);
	pidSetIntegralLimit(&PitchRate,100.0f);
	pidSetIntegralLimit(&YawRate,100.0f);
	
	
	/*
	RollRate.Kp=0.15f;//all reduce 100;
	RollRate.Ki=0.002f;
	RollRate.Kd=3;
	PitchRate.Kp=0.15f;
	PitchRate.Ki=0.002f;
	PitchRate.Kd=3;
	YawRate.Kp=1.8f;
	YawRate.Ki=0.004f;
	YawRate.Kd=0.0f;
	Stabilize_Roll.Kp=2.08f;
        Stabilize_Roll.Ki=0.0f;
	Stabilize_Roll.Kd=0.0f;
	Stabilize_Pitch.Kp=2.08f;
	Stabilize_Pitch.Ki=0.0f;
	Stabilize_Pitch.Kd=0.0f;
	Stabilize_Yaw.Kp=1.3f;
	Stabilize_Yaw.Ki=0.0f;
	Stabilize_Yaw.Kd=0.0f;
	*/
	pid_flag=1;
}

int16_t hight_BUF[10];	
int16_t hight_AVG=0;
void  fliter_hight(int16_t x1)
{	
	static uint8_t filter_cnt=0;
	int32_t temp1=0;
	uint8_t i;

	hight_BUF[filter_cnt] = x1;
	
	for(i=0;i<5;i++)
	{
		temp1 += hight_BUF[i];
	
	
	}
	hight_AVG = temp1 / 5;
	
	
		
	filter_cnt++;
	if(filter_cnt==5)	filter_cnt=0;
}


void limithight(void)
{
   	static int16_t Alt_CSB_last=0;
		if(Alt_CSB>1200||Alt_CSB<-1200)
		{	
		    Alt_CSB=Alt_CSB_last;
		}	
		else
		{	
		    Alt_CSB_last=Alt_CSB;
		}		
}

float pid_out=0;
void US100_CONTROL(float Alt_CSB1,float US100_Alt_Target_1,float dt)//mm
{
	float kp=43;//kp=74//kp=34
	float ki=0.013;
	float kd=11;//kd=54//kd=31
	float tr_tmep1=0;
	static float thr_tmep=0;
	float PID_US100_pout,PID_US100_dout,PID_US100_OUT;
	float alt_us100_1=0;
	float US100_Alt_V=0;
	static float vel_us100=0;
	static float  ALt_CSB_Last=0;

  alt_us100_1 =(Alt_CSB1 - US100_Alt_Target_1)/1000.f;   
  
  if(alt_us100_1>0.6f)   
  {ARMED=0;}    
    US100_Alt_V=((float)(Alt_CSB1-ALt_CSB_Last))/dt*0.001f;
	
		PID_US100_pout=-(kp)*alt_us100_1;
	        ALt_CSB_Last=Alt_CSB1;
		PID_US100_dout=-(kd)*US100_Alt_V;
	 //PID_US100_dout=DEAD_BAND(PID_US100_dout,0,0.02);
	
		thr_tmep+=(PID_US100_pout)*dt*ki;
		tr_tmep1=thr_tmep;
		


	  PID_US100_OUT=tr_tmep1+PID_US100_pout+PID_US100_dout;
		pid_out=PID_US100_OUT;
	  if(pid_out>130)   //油门输出限制幅度
    pid_out=130; 
  if(pid_out<-500) 
    pid_out=-500; 
  //THR_Lock=thr_tmep; //锁定油门大少
 

}
#define newdata 0.13

int16_t iic_fliter(int16_t ndata)
{
	int16_t data=0;
	static int16_t odata=0;
	data=ndata*newdata+odata*(1-newdata);
	odata=data;
	return data;
}
int16_t datafliter=0;   	
void prate_hight(void)
{       
	
	uint32_t nowtime=micros();
	static uint32_t lasttime=0;
	float hight_dt=(nowtime-lasttime)/1000000.0f;
	U8 senddata=0x55;
	 R_UART1_Send(&senddata,1);
	data_cnt=0;
	limithight();
	datafliter=iic_fliter(Alt_CSB);
	//fliter_hight(Alt_CSB);
	US100_CONTROL(datafliter,500.0f,hight_dt);
	lasttime=nowtime;
}

