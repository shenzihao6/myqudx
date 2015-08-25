#ifndef __PWMOUT_H
#define __PWMOUT_H
#include "systimer.h"
#include "r_cg_userdefine.h"
#include "r_cg_timer.h"
#include "pwmin.h"

extern volatile int16_t Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4;

void set_pwm(uint16_t m1,uint16_t m2,uint16_t m3, uint16_t m4);
void PWM_Control(void);
#endif