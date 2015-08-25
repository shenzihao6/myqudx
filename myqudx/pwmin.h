#ifndef __PWMIN_H
#define __PWMIN_H
#include "r_cg_userdefine.h"
 struct Rc_Data
{
	U16 ROLL;
	U16 PITCH;
	U16 YAW;
	U16 THROTTLE;
	U16 AUX1;
	U16 AUX2;
	U16 AUX3;
};


extern volatile struct  Rc_Data RC_Data;
extern U8 ARMED;
void RC_Data_Anl(void);
#endif