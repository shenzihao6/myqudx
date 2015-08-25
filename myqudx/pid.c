#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP
#include "pid.h"


// low pass filter:
// f_cut = 1/(2*PI*cutoff_freq)
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
const float  lowpass_filter = 7.9577e-3;



void pidInit(struct Quad_PID* pid, float kp, float ki, float kd)//need init, but i didn"t do it
{
  pid->merror = 0;
  pid->last_error = 0;
  pid->Integrator = 0;
  pid->deriv = 0;
  pid->target = 0;
  pid->last_deriv = 0;
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  
  pid->Lowpass_EN = PID_FALSE;   
  pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;	
}

float pidUpdate(struct Quad_PID* pid, float measured,float dt)
{
  float output;
	static float last_Integrator=0; 
  pid->current = measured;

  pid->merror = pid->target - measured;

  //I  
  if(pid->Ki != 0)
	{
		pid->Integrator =last_Integrator+ (pid->Ki * pid->merror) * dt;
		last_Integrator=pid->Integrator;
		
		if (pid->Integrator > pid->iLimit)
		{
			pid->Integrator = pid->iLimit;
		}
		else if (pid->Integrator < -pid->iLimit)
		{
			pid->Integrator = -(pid->iLimit);
		}
	
  }
	
  //D 
  pid->deriv = (pid->merror - pid->last_error) / dt;
  if(pid->Lowpass_EN != PID_FALSE)
	{	
   pid->deriv = pid->last_deriv + (dt / (lowpass_filter + dt)) * (pid->deriv - pid->last_deriv);	
  }

  pid->outP = pid->Kp * pid->merror;
  pid->outI = pid->Integrator;
  pid->outD = pid->Kd * pid->deriv;

  pid->PID_out = output = 	pid->outP +pid->outI +pid->outD;

  pid->last_error = pid->merror;

  return output;
}

float pidUpdate_err(struct Quad_PID* pid,float err, float dt)
{
  float output;
	static float last_Integrator=0; 
  pid->merror = err;
  //I
  pid->Integrator =last_Integrator+(pid->Ki * pid->merror) * dt;
	 last_Integrator=pid->Integrator;
  if (pid->Integrator > pid->iLimit)
  {
    pid->Integrator = pid->iLimit;
  }
  else if (pid->Integrator < -pid->iLimit)
  {
    pid->Integrator = -(pid->iLimit);
  }

  //D
  pid->deriv = (pid->merror - pid->last_error) / dt;
  if(pid->Lowpass_EN != PID_FALSE)
	{	
   pid->deriv = pid->last_deriv + (dt / (lowpass_filter + dt)) * (pid->deriv - pid->last_deriv);	
  }
  //P
  pid->outP = pid->Kp * pid->merror;
  pid->outI = pid->Integrator;
  pid->outD = pid->Kd * pid->deriv;

  pid->PID_out = output = pid->outP +pid->outI +pid->outD;

  pid->last_error = pid->merror;

  return output;
}

void pidSetError(struct Quad_PID* pid, float err)
{
  pid->merror = err;
}

void pidSetIntegralLimit(struct Quad_PID* pid, float limit)
{
  pid->iLimit = limit;
}

void pidReset(struct Quad_PID* pid)
{
  pid->merror = 0;
  pid->last_error = 0;
  pid->Integrator = 0;
  pid->deriv = 0;
}


void pidSetTarget(struct Quad_PID* pid, float target)
{
  pid->target = target;
}

void pidSetTarget_Measure(struct Quad_PID* pid, float target, float measured)
{
  pid->target = target;
  pid->current = measured;
  pid->merror = pid->target - measured;
}

void pidSetKp(struct Quad_PID* pid, float kp)
{
  pid->Kp = kp;
}

void pidSetKi(struct Quad_PID* pid, float ki)
{
  pid->Ki = ki;
}

void pidSetKd(struct Quad_PID* pid, float kd)
{
  pid->Kd = kd;
}

void pidSetMeasured(struct Quad_PID* pid, float measured)
{
  pid->current = measured;
}

void pidSetLowPassEnable(struct Quad_PID* pid)
{
	pid->Lowpass_EN = PID_TRUE;	
}