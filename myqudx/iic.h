#ifndef __IIC_H
#define __IIC_H
#include "r_cg_userdefine.h"
 U8 iic_start(void);
 U8 iic_rstart(void);
 U8 iic_stop(void);
 U8 i2c_senddat(U8 dat);
 U8 i2c_recedat(U16 mode);
#endif