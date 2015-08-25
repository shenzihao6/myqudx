#ifndef __ALTITUDE_H
#define __ALTITUDE_H
#include "mpu6050.h"
#include "r_cg_userdefine.h"
#include "hmc5883l.h"
#include "math.h"
#include "r_cg_timer.h"
#include "pwmin.h"

typedef struct {
	float b0;
        float a1;
	float a2;
        float preout;
        float lastout;
}LPF2ndData_t;

extern volatile float IMU_Pitch, IMU_Roll, IMU_Yaw;
extern volatile float  IMU_GYROx, IMU_GYROy, IMU_GYROz;

void IMU_init(void);
void IMU_getYawPitchRoll(float * angles);
void  _fliter(float x1,float x2,float x3);
extern volatile int16_t accgyroval[6];
#endif