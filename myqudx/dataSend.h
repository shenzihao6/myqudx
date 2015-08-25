#ifndef __DATASEND_H
#define __DATASEND_H

//#include "r_cg_serial.h"
#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"
#include "r_cg_userdefine.h"
#include "pwmin.h"
#include "pwmout.h"
void Data_Send_Senser(void);
void Send_Data(void);
void Data_Send_Status(void);
void Data_Send_RCData(void);
void Data_Send_MotoPWM(void);
void Data_Send_PID1(void);
void Data_Send_PID2(void);
extern U8 pid_flag;
extern U8 SendPidDataFlag;
#endif