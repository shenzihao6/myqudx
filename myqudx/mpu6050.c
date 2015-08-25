
#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP

#include "mpu6050.h"

int16_t Acc_X_Offset=0,Acc_Y_Offset=0,Gyro_X_Offset=0,Gyro_Y_Offset=0,Gyro_Z_Offset=0;
int16_t MPU6050_FIFO[6][10];
int16_t ledflag=0;
#define GYRO_GAIN_RAD		((2000*3.1415926)/(32768*180)) //0.00106526



extern void R_IICA0_Create(void);
/*extern void led_flash(void);*/



const U8 mpu6050initdat[][2]=
{
{MPU6050_REG_PM1,   0x03},//¨¬¨¢1?¨º¡À?¨®¨¦¨¨????D?¡Á¡Â¨®?
				 /*   |||||***------CLKSEL = 001:PLL with X axis gyroscope reference
				      ||||*---------TEMP_DIS 0  :enable the temperature sensor
				      |||*----------always 0
				      ||*-----------CYCLE =  0  :disable cycle
				      |*------------SLEEP =  0  :out sleep mode
				      *-------------DEVICE_RESET = 0: 
				*/
{MPU6050_REG_CONFIG,0b00000010},//¨ºy¡Á???2¡§?¡Â¦Ì?¨¦¨¨???a¨¤??1o¨¹?¡êoy,o???¨º?98hz¦Ì?¡ê?¨®|???¨¦¨°??¨´??
				/*    |||||***------DLPF_CFG = 0b010 :Accelerometer Bandwidth= 94HZ Gyroscope = 98HZ sample=1Khz
				      ||***---------EXT_SYNC_SET  = 0b00:Input disabled
				      **------------always 0
				*/
{MPU6050_REG_SMPRT ,SAMPLE_RATE},//2¨¦?¨´?¦Ì?¨º?¨ª?a?¨´¡ä???¡ã¨¦¡ê??1?¨¹??
				/*    ********------SMPLRT_DIV = 2:Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)=1K/4=250HZ
				*/
{MPU6050_REG_GYROC ,0b00011000},//+-2000
				/*    |||||***------always 0
				      |||**---------FS_SEL  = 11:Full Scale Range = ?2000 ?s
				      ||*-----------ZG_ST   = 0 :Z axis gyroscope don't self test
				      |*------------YG_ST   = 0 :Y axis gyroscope don't self test
				      *-------------XG_ST   = 0 :X axis gyroscope don't self test
				*/
{MPU6050_REG_ACCELC,0x00000000},//+-4g
				/*    |||||***------always 0
				      |||**---------FS_SEL  = 01 : Full Scale Range = +/-4g
				      ||*-----------ZA_ST   = 0 :Z axis accelerometer don't self test
				      |*------------YA_ST   = 0 :Y axis accelerometer don't self test
				      *-------------XA_ST   = 0 :X axis accelerometer don't self test
				*/
{MPU6050_REG_INTP_CFG,0b00110010},/*set as pulse and read clear mode*/ //HERE 1
				/*      |||||||*------ 0 
				        ||||||*-------I2C_BYPASS_EN  =0   0 disable 1 enable
				        |||||*--------FSYNC_INT_EN   =0   0 disable 1 enable
				        ||||*---------FSYNC_INT_LEVEL=0   0 disable 1 enable
				        |||*----------INT_RD_CLEAR   =0   0 disable  1 enable
				        ||*-----------LATCH_INT_EN   =0   0 pulse interrupt 1 level
				        |*------------INT_OPEN       =0   0 push pull 1 open drain
				        *-------------INT_LEVEL      =0   0 active h 1 active l
				*/

{MPU6050_REG_INTP_EN, 0b00000001},/*enable data ready interrput*/
				/*      |||||||*------DATA_RDY_EN  =1    0 disable 1 enable
				        ||||||*-------0
				        |||||*--------0
				        ||||*---------I2C_MST_INT_EN=0   0 disable 1 enable
				        |||*----------FIFO_OFLOW_EN =0   0 disable  1 enable
				        ||*-----------0
				        |*------------MOT_EN        =0   0 disable  1 enable
				        *-------------0
				*/
{0xff,0xff}
};

struct mpu6050st mpudat;



static U8 sel_mpu6050_reg(U8 reg)//I2c_ReadOneByte(dev,reg);
{
	if(OK!=iic_start())return ERROR;
	if(OK!=i2c_senddat(W_MPU6050_ADDRESS))
	{
		iic_stop();
		return ERROR;
	};
	if(OK!=i2c_senddat(reg))
	{
		iic_stop();
		return ERROR;
	};
	return OK;
}


static U8 write_mpu6050(U8 reg,U8 *datbuf,U16 datl)
{
	if(OK!=sel_mpu6050_reg(reg))return ERROR;
	for(;datl!=0;datl--)
	{
		if(OK!=i2c_senddat(*datbuf))
		{
			iic_stop();
			return ERROR;
		};
		datbuf++;
	}
	return iic_stop();
}

static U8 read_mpu6050(U8 reg,U8 *datbuf,U16 datl)
{
	if(OK!=sel_mpu6050_reg(reg))return ERROR;
	if(OK!=iic_rstart())return ERROR;
	if(OK!=i2c_senddat(R_MPU6050_ADDRESS))
	{
		iic_stop();
		return ERROR;
	};

	for(;datl!=0;datl--)
	{
		*datbuf=i2c_recedat(datl);
		datbuf++;
	}
	return iic_stop();
}

U8 test_mpu6050(void)
{
	U8 buff[2]={0,0};
	read_mpu6050(MPU6050_REG_WHOAMI,buff,1);//readbytes
	buff[0]&=0x7e;
	if(buff[0]==0x68)return OK;
	return ERROR;
}

U8 init_mpu6050(void)
{
	U8 i;
	const U8 *p;
	R_IICA0_Create();
	if(OK!=test_mpu6050())
	return ERROR;
	p=&(mpu6050initdat[0][0]);
	while(*p!=0xff)
	{
		if(OK!=write_mpu6050(*p,(p+1),1))return ERROR;
		p+=2;
	}
	return OK;
}


int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;
uint8_t buf[14];

void mpu6050_fifo_fliter(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)//huadong chuangkou array
{
    unsigned char i ;
    int32_t sum=0;
    for(i=1;i<10;i++)
    {	
		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
    }
		MPU6050_FIFO[0][9]=ax;
		MPU6050_FIFO[1][9]=ay;
		MPU6050_FIFO[2][9]=az;
		MPU6050_FIFO[3][9]=gx;
		MPU6050_FIFO[4][9]=gy;
		MPU6050_FIFO[5][9]=gz;


	sum=0;
	for(i=0;i<10;i++)
	{	
  	 	sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
   		sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
	   	sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
  		 sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
  		 sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
   		sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
}

void mpu6050GetData1(int16_t* ax,int16_t* ay,int16_t* az,int16_t* gx,int16_t* gy,int16_t* gz)
{
	read_mpu6050(MPU_RA_ACCEL_XOUT_H,buf,14);
	 *ax=MPU6050_Lastax=(((int16_t)buf[0])<<8|buf[1])-Acc_X_Offset;
	 *ay=MPU6050_Lastay=(((int16_t)buf[2])<<8|buf[3])-Acc_Y_Offset;
	 *az=MPU6050_Lastaz=(((int16_t)buf[4])<<8|buf[5]);
	 
	 *gx=MPU6050_Lastgx=(((int16_t)buf[8])<<8|buf[9])-Gyro_X_Offset;
	 *gy=MPU6050_Lastgy=(((int16_t)buf[10])<<8|buf[11])-Gyro_Y_Offset;
	 *gz=MPU6050_Lastgz=(((int16_t)buf[12])<<8|buf[13])-Gyro_Z_Offset;
	/*
	mpu6050_fifo_fliter(MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz);
	*ax  =MPU6050_FIFO[0][10]-Acc_X_Offset;
	*ay  =MPU6050_FIFO[1][10]-Acc_Y_Offset;
	*az = MPU6050_FIFO[2][10];	
	*gx  =MPU6050_FIFO[3][10]-Gyro_X_Offset;
	*gy = MPU6050_FIFO[4][10]-Gyro_Y_Offset;
	*gz = MPU6050_FIFO[5][10]-Gyro_Z_Offset;
	*/
	 _fliter(*ax,*ay,*az);
	if(acc_gyro_flag==1)
	{
	  mpu6050offset();
	  ledflag=100;
	}
	if(ledflag>0)
	{
		ledflag--;
		P7.0=0;
	}
	else
	{
		P7.0=1;
		ledflag=0;
	}	
}
void mpu6050GetData(int16_t* ax,int16_t* ay,int16_t* az,int16_t* gx,int16_t* gy,int16_t* gz)
{
          read_mpu6050(MPU_RA_ACCEL_XOUT_H,buf,14);
	 *ax=MPU6050_Lastax=(((int16_t)buf[0])<<8|buf[1]);
	 *ay=MPU6050_Lastay=(((int16_t)buf[2])<<8|buf[3]);
	 *az=MPU6050_Lastaz=(((int16_t)buf[4])<<8|buf[5]);
	 
	 *gx=MPU6050_Lastgx=(((int16_t)buf[8])<<8|buf[9]);
	 *gy=MPU6050_Lastgy=(((int16_t)buf[10])<<8|buf[11]);
	 *gz=MPU6050_Lastgz=(((int16_t)buf[12])<<8|buf[13]);
	mpu6050_fifo_fliter(MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz);
	/*
	*ax  =MPU6050_FIFO[0][10];
	*ay  =MPU6050_FIFO[1][10];
	*az = MPU6050_FIFO[2][10];	
	*gx  =MPU6050_FIFO[3][10];
	*gy = MPU6050_FIFO[4][10];
	*gz = MPU6050_FIFO[5][10];
	*/
}

void mpu6050offset(void)//original data zero fix ,or fliter data zero fix
{
		unsigned char i;
	int16_t temp[6];
	int32_t	tempgx=0,tempgy=0,tempgz=0;
	int32_t	tempax=0,tempay=0,tempaz=0;
	Acc_X_Offset=0;
	Acc_Y_Offset=0;

	Gyro_X_Offset=0;
	Gyro_Y_Offset=0;
	Gyro_Z_Offset=0;
	for(i=0;i<50;i++)
	{
  		delay_us(100);
  		mpu6050GetData(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
	}
 	for(i=0;i<100;i++)
	{
		delay_us(200);
		mpu6050GetData(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
		tempax+= temp[0];
		tempay+= temp[1];
		tempaz+= temp[2];
		tempgx+= temp[3];
		tempgy+= temp[4];
		tempgz+= temp[5];
	}

	Acc_X_Offset=tempax/100;
	Acc_Y_Offset=tempay/100;
	//acc_gz_values=acc_vector;
	Gyro_X_Offset=tempgx/100;//MPU6050_FIFO[3][10];
	Gyro_Y_Offset=tempgy/100;//MPU6050_FIFO[4][10];
	Gyro_Z_Offset=tempgz/100;//MPU6050_FIFO[5][10];
	acc_gyro_flag=0;
	
	eeprom_write(Acc_X_Offset,Acc_Y_Offset,Gyro_X_Offset,Gyro_Y_Offset,Gyro_Z_Offset,Hmc5883l_X_Offset,Hmc5883l_Y_Offset,Hmc5883l_Z_Offset,
    (int16_t)RollRate.Kp,(int16_t)PitchRate.Kp,(int16_t)YawRate.Kp,(int16_t)RollRate.Ki,(int16_t)PitchRate.Ki,(int16_t)YawRate.Ki,(int16_t)RollRate.Kd,(int16_t)PitchRate.Kd,(int16_t)YawRate.Kd,(int16_t)Stabilize_Roll.Kp,(int16_t)Stabilize_Pitch.Kp,(int16_t)Stabilize_Yaw.Kp);
}

