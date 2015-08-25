#ifndef __MPU6050_H
#define __MPU6050_H
#include "dataRec.h"
#include "iic.h"
#include "eeprom.h"
#include "hmc5883l.h"
struct mpu6050st
{
	S16 accel_xout;
	S16 accel_yout;
	S16 accel_zout;
	S16 temp_out;
	S16 gyro_xout;
	S16 gyro_yout;
	S16 gyro_zout;
};

#define W_MPU6050_ADDRESS 		0xD0
#define R_MPU6050_ADDRESS 		0xD1
#define MPU6050_REG_WHOAMI		0x75

#define MPU6050_REG_SENSOR_DAT 0x3B

#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_GYRO_XOUT_H      0x43
/*power control*/
#define MPU6050_REG_PM1			0x6B
#define MPU6050_REG_PM2			0x6C
/*user config*/
#define MPU6050_REG_USERC		0x6A
#define MPU6050_REG_SMPRT		0x19/*Sample Rate Divider*/
#define MPU6050_REG_CONFIG		0x1A/*Configuration*/ //¨ºy¡Á???2¡§?¡Â¨°2¨º??¨²?a¨¦¨¨??
#define MPU6050_REG_GYROC		0x1B 
#define MPU6050_REG_ACCELC		0x1C 
#define MPU6050_REG_INTP_CFG	0x37
#define MPU6050_REG_INTP_EN		0x38

#define USER_CTRL               0x6A
#define I2C_MST_CTRL            0x24
#define I2C_SLV0_ADDR           0x25
#define I2C_SLV0_REG            0x26
#define I2C_SLV0_CTRL           0x27

#define SAMPLE_RATE				3
#define Z_accel_xout	(65)
#define Z_accel_yout	(-12)
#define Z_accel_zout	(-2095)

#define Z_gyro_xout	(20)
#define Z_gyro_yout	(-22)
#define Z_gyro_zout	(-20)

#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.


#define MAG_ADDRESS        0x1E
#define MAG_DATA_REGISTER  0x03
#define EXT_SENS_DATA      0x49

#define MPU6050_REG_RAW_DATA	0x3B
#define MPU6050_REG_ACC		0x3B
#define MPU6050_REG_TEM		0x41
#define MPU6050_REG_GYR		0x43

U8 test_mpu6050(void);
U8 init_mpu6050(void);
//U8 get_mpu6050_zero(void);
//U8 get_mpu6050_dat_h(struct mpu6050st *datbuf);


//#define GYRO_SCALE_FACTOR     0.00121414f       //  MPU3050 14.375 LSBs per dps
#define GYRO_SCALE_FACTOR       0.00106422f      //  MPU6050 16.4 LSB/DPS (1/16.4*PI/180)


extern int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;
extern int16_t Acc_X_Offset,Acc_Y_Offset,Gyro_X_Offset,Gyro_Y_Offset,Gyro_Z_Offset;

U8 test_mpu6050(void);
U8 init_mpu6050(void);
void mpu6050GetData(int16_t* ax,int16_t* ay,int16_t* az,int16_t* gx,int16_t* gy,int16_t* gz);
void mpu6050GetData1(int16_t* ax,int16_t* ay,int16_t* az,int16_t* gx,int16_t* gy,int16_t* gz);
 void mpu6050offset(void);

#endif