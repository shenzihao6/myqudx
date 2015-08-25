#ifndef __SYSTIMER_H
#define __SYSTIMER_H
//#include "r_cg_userdefine.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "pid.h"
#include "fmath.h"
#include "altitude.h"
#include "pwmin.h"
#include "pwmout.h"
#include "r_cg_timer.h"
#define  CVERSION  1.4

#define Quad_Manual   0x01
#define Quad_Level_Lock  0x02
#define Quad_Hold_Position   0x03
#define Quad_ESC_Cal  0x04

extern volatile uint8_t  Fly_Mode;
extern volatile int16_t  Yaw_DIRECT ;
extern volatile float PID_dt;
extern float  Target_Roll;
extern uint32_t last_time;
extern int16_t THROTTLE,PID_ROLL,PID_PITCH,PID_YAW;
extern int16_t PWM_Offset_Roll,PWM_Offset_Pitch,PWM_Offset_Yaw;
extern volatile uint8_t Quadrotor_Mode;	//¡¤¨¦DD?¡ê¨º?
extern int16_t tpitch,troll,tyaw,pidpitch,pidroll,pidyaw;
extern  float acc_gz_values;
extern  float  ClimbTarget ;
extern float pid_out;
extern float vel;
extern int16_t hight_AVG;
extern int16_t datafliter;
extern float pid_out;
extern struct Quad_PID 
	     Stabilize_Roll,  
	     Stabilize_Pitch,  
	     RollRate,		  
	     PitchRate,		  
	     Stabilize_Yaw,	
	     YawRate,          
	     AutoHigh_THR,	 
	     Climb,            
	     Position_Hold,	  
	     Position_Speed;	  

void Quadrotor_Motor_Update(void);
void Initial_Math(void);
void PWM_Save_Offset(void);
void PWM_Output_ESC_Calibration(void);
void prate_hight(void);//us100
void prate_hight_ms5611(void);//ms5611




#define DATA_TRANSFER_USE_NRF
#define RC_CHANL_NUM	7
#define RC_MIDDLE 		1500
#define RC_MIN				1200
#define	RC_MAX				1800
#define RC_DEADBAND		5


#define QUADP	 0x01  
#define QUADX	 0x02  
#define Y4		 0x03
#define Y6		 0x04
#define HEX6     0x05
#define HEX6X	 0x06
#define OCTOX8	 0x07

#define default_quad  QUADX  

#define MINTHROTTLE 1020

#define MAXTHROTTLE 1850

#define MAX_RC_Ch	6
#define PWM_Input_Offset  1500	 //Ö?ÁcÖµ 1500 us
#define PWM_Input_MID   1500
#define PWM_Input_MAX   2200
#define PWM_Input_MIN   800
#define PWM_Input_Low_position  1000  //×îµÍÖµ 1000us

//#define YAW_DIRECTION -1 //
#define YAW_DIRECTION 1
#define DEAD_BAND(value,mid,ban) (((value)<(mid)-(ban))?((value)+(ban)):(((value)>(mid)+(ban))?((value)-(ban)):(mid)))

#endif