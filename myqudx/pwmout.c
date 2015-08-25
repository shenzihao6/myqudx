#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP

#include "pwmout.h"

volatile int16_t Moto_PWM_1=0,Moto_PWM_2=0,Moto_PWM_3=0,Moto_PWM_4=0;
#define Limit_Max     (int16_t)20
#define PIDMIX(X,Y,Z) THROTTLE	+ PID_ROLL*X + PID_PITCH*Y +PID_YAW*Z

#define Min_PWM_Out  1000    //us
#define Max_PWM_Out  2000    //us
int16_t Last_PID_ROLL,Last_PID_PITCH,Last_PID_YAW;

void set_pwm(uint16_t m1,uint16_t m2,uint16_t m3, uint16_t m4)//should check
{
	 TDR01=m1*16;
	 TDR02=m2*16;
	 TDR03=m3*16;
	 TDR04=m4*16;
	
}
void PWM_PID_Smooth(void)
{
   PID_ROLL = Last_PID_ROLL + Math_Constrain((PID_ROLL - Last_PID_ROLL),-Limit_Max,+Limit_Max);
   PID_PITCH = Last_PID_PITCH + Math_Constrain((PID_PITCH - Last_PID_PITCH),-Limit_Max,+Limit_Max);
   PID_YAW = Last_PID_YAW + Math_Constrain((PID_YAW - Last_PID_YAW),-Limit_Max,+Limit_Max);
   
  
   Last_PID_ROLL = PID_ROLL;
   Last_PID_PITCH = PID_PITCH;
   Last_PID_YAW	= PID_YAW;
}
void PWM_Control(void)
{
	PWM_PID_Smooth();
	
	if(THROTTLE>1250&&ARMED==1)
	{
		Moto_PWM_1 =Math_Constrain(PIDMIX(-1,-1,+1),Min_PWM_Out,Max_PWM_Out)+pid_out; //FRONT_R ?°Ó?µ?»?
		Moto_PWM_2 =Math_Constrain(PIDMIX(+1,-1,-1),Min_PWM_Out,Max_PWM_Out)+pid_out; //FRONT_L ?°×óµ?»?	
		Moto_PWM_3 =Math_Constrain(PIDMIX(+1,+1,+1),Min_PWM_Out,Max_PWM_Out)+pid_out; //REAR_L  ?ó×óµ?»?
		Moto_PWM_4 =Math_Constrain(PIDMIX(-1,+1,-1),Min_PWM_Out,Max_PWM_Out)+pid_out; //REAR_R  ?óÓ?µ?»?
	}
	else
	{
		Moto_PWM_1 = 1000; 
		Moto_PWM_2 = 1000;
		Moto_PWM_3 = 1000;
		Moto_PWM_4 = 1000; 
	}
	set_pwm(Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4);//should check
}