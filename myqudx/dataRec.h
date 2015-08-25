#ifndef __DATAREC_H
#define __DATAREC_H

#include "dataSend.h"
#include "hmc5883l.h"
extern U8 acc_gyro_flag;
void Data_Receive_Anl(U8 *data_buf,U8 num);
#endif