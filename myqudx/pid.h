#ifndef __PID_H
#define __PID_H
#include "r_cg_userdefine.h"
#define PID_TRUE  0xff
#define PID_FALSE 0x00



struct Quad_PID{
	float target;  
	float current;  
	float merror;
	float last_error;
  float Integrator;	
	float deriv;
	float last_deriv;
	float iLimit;
	float Kp;	   
	float Ki;	  
	float Kd;	  
	unsigned char Lowpass_EN;
	float outP;         //< proportional output (debugging)
  	float outI;         //< integral output (debugging)
  	float outD;         //< derivative output (debugging)
	float PID_out;   
};

#define DEFAULT_PID_INTEGRATION_LIMIT  15.0

void pidInit(struct Quad_PID* pid, const float kp,
             const float ki, const float kd);
float pidUpdate(struct Quad_PID* pid, float measured,float dt);
float pidUpdate_err(struct Quad_PID* pid,float err, float dt);
void pidSetIntegralLimit(struct Quad_PID* pid, float limit);
void pidSetError(struct Quad_PID* pid, float err);
void pidReset(struct Quad_PID* pid);
void pidSetTarget(struct Quad_PID* pid, float target);
void pidSetKp(struct Quad_PID* pid, float kp);
void pidSetKi(struct Quad_PID* pid, float ki);
void pidSetKd(struct Quad_PID* pid, float kd);
void pidSetMeasured(struct Quad_PID* pid, float measured);
void pidSetTarget_Measure(struct Quad_PID* pid, float target, float measured);
void pidSetLowPassEnable(struct Quad_PID* pid);

#endif