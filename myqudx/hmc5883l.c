#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP

#include "hmc5883l.h"
float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;
int16_t Hmc5883l_X_Offset=0,Hmc5883l_Y_Offset=0,Hmc5883l_Z_Offset=0;
float  HMC5883_Scale_X = 1.0f,HMC5883_Scale_Y = 1.0f,HMC5883_Scale_Z = 1.0f;

static U8 sel_hmc5883l_reg(U8 reg)
{
	if(OK!=iic_start())return ERROR;
	if(OK!=i2c_senddat(HMC58X3_ADDR))
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

static U8 single_write_hmc5883l(U8 reg,U8 dat)
{
	if(OK!=sel_hmc5883l_reg(reg))return ERROR;
	
		if(OK!=i2c_senddat(dat))
		{
			iic_stop();
			return ERROR;
		};
	
	return iic_stop();
}
static U8 HMC58X3_writeReg(U8 reg,U8 dat)
{
	if(OK!=single_write_hmc5883l(reg,dat))
	{
		return ERROR;
	};
	return OK;
}


void HMC58X3_setMode(U8 mode) 
{
  if (mode > 2)
  {
    return;
  }
  HMC58X3_writeReg(HMC58X3_R_MODE, mode);
  
}
void HMC58X3_setDOR(unsigned char DOR) {
  if (DOR>6)
  {
   return;
  } 
  HMC58X3_writeReg(HMC58X3_R_CONFA,DOR<<2);
}

static U8 read_hmc5883l(U8 reg,U8 *datbuf,U16 datl)
{
	if(OK!=sel_hmc5883l_reg(reg))return ERROR;
	if(OK!=iic_rstart())return ERROR;
	if(OK!=i2c_senddat(HMC58X3_ADDR+1))//not really right
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
int16_t  HMC5883_FIFO[3][11]; 
int16_t  HMC5883_maxx=0,HMC5883_maxy=0,HMC5883_maxz=0,HMC5883_minx=-0,HMC5883_miny=-0,HMC5883_minz=-0;
unsigned char HMC5883_calib=0; 
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	static U8 add=0;
	int32_t sum=0;

	for(i=1;i<10;i++)
	{
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}

	HMC5883_FIFO[0][9]=x;
	HMC5883_FIFO[1][9]=y;
	HMC5883_FIFO[2][9]=z;

	sum=0;
	for(i=0;i<10;i++)
	{	
   		sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;	

	sum=0;
	for(i=0;i<10;i++)
	{
   		sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
   		sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10]=sum/10;

	if(HMC5883_calib)
	{
		
		led1FastFlash();
		if(HMC5883_minx>HMC5883_FIFO[0][10])
		{
		 	HMC5883_minx=(int16_t)HMC5883_FIFO[0][10];
		} 
		if(HMC5883_miny>HMC5883_FIFO[1][10])
		{
			 HMC5883_miny=(int16_t)HMC5883_FIFO[1][10];
		} 
		if(HMC5883_minz>HMC5883_FIFO[2][10])
		{
			 HMC5883_minz=(int16_t)HMC5883_FIFO[2][10];
		}

		if(HMC5883_maxx<HMC5883_FIFO[0][10])
		{
                 	HMC5883_maxx=(int16_t)HMC5883_FIFO[0][10];
		} 
		if(HMC5883_maxy<HMC5883_FIFO[1][10])
		{
			 HMC5883_maxy=(int16_t)HMC5883_FIFO[1][10];
		} 
		if(HMC5883_maxz<HMC5883_FIFO[2][10])
		{
			 HMC5883_maxz=(int16_t)HMC5883_FIFO[2][10];
		} 
	//	add++;
	//	if(add<=128)
	//		led1on;
	//	else
	//		led1off;
	//	if(add==255)
	//		add=0;
//	}	LED_Set_Blink(Blue,30,50,1); 
	  }

} //HMC58X3_newValues
U8 HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
{
   unsigned char vbuff[6];
   unsigned char ok;
   vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
   ok=read_hmc5883l(HMC58X3_R_XM,vbuff,6);
   HMC58X3_newValues(((int16_t)vbuff[0] << 8) | vbuff[1],((int16_t)vbuff[4] << 8) | vbuff[5],((int16_t)vbuff[2] << 8) | vbuff[3]);
   *x = HMC5883_FIFO[0][10];
   *y = HMC5883_FIFO[1][10];
   *z = HMC5883_FIFO[2][10];
   return ok;
}

void HMC58X3_mgetValues(float *arry) 
{
  int16_t xr,yr,zr;
  HMC58X3_getRaw(&xr, &yr, &zr);
 
	
  arry[0]= HMC5883_lastx=(float)(((float)(xr - Hmc5883l_X_Offset)) * HMC5883_Scale_X);
  arry[1]= HMC5883_lasty=(float)(((float)(yr - Hmc5883l_Y_Offset)) * HMC5883_Scale_Y);
  arry[2]= HMC5883_lastz=(float)(((float)(zr - Hmc5883l_Z_Offset)) * HMC5883_Scale_Z);
}

void HMC58X3_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++)
  {
  	HMC58X3_getRaw(&temp[0],&temp[1],&temp[2]);
 // delay_us(200); 
  }
}
U8 hmc5883_Init(U8 setmode)
{ 
	U8 ok1=0,ok2=0,ok3=0;
  if(setmode)
  {
     HMC58X3_setMode(0);  
  }
     
  ok1=HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
  ok2=HMC58X3_writeReg(HMC58X3_R_CONFB, 0xA0);
  ok3=HMC58X3_writeReg(HMC58X3_R_MODE, 0x00);
  if(ok1&&ok2&&ok3)
  {
     return OK;	  
  }
  else 
     return ERROR;
}
void hmc5883Setup(void)
{    
	
    	hmc5883_Init(0); // Don't set mode yet, we'll do that later on.
	  delay_ms(50);
        HMC58X3_setMode(0);
	 delay_ms(50);
        HMC58X3_setDOR(6);  //75hz ¸
	delay_ms(50);
       HMC58X3_FIFO_init();
	delay_ms(50);
      
}
void HMC5883L_Start_Calib(void)
{
	HMC5883_calib=1;
	HMC5883_maxx=0;	
	HMC5883_maxy=0;
	HMC5883_maxz=0;
	HMC5883_minx=-0;
	HMC5883_miny=-0;
	HMC5883_minz=-0;
}
void HMC5883L_Save_Calib(void)
{
	
	 Hmc5883l_X_Offset = (HMC5883_maxx+HMC5883_minx)/2;
	 Hmc5883l_Y_Offset = (HMC5883_maxy+HMC5883_miny)/2;
	 Hmc5883l_Z_Offset = (HMC5883_maxz+HMC5883_minz)/2;
  
	  HMC5883_Scale_X=1.0f;
	  HMC5883_Scale_Y=1.0f;
	  HMC5883_Scale_Z=1.0f;

	eeprom_write(Acc_X_Offset,Acc_Y_Offset,Gyro_X_Offset,Gyro_Y_Offset,Gyro_Z_Offset,Hmc5883l_X_Offset,Hmc5883l_Y_Offset,Hmc5883l_Z_Offset,
    RollRate.Kp,PitchRate.Kp,YawRate.Kp,RollRate.Ki,PitchRate.Ki,YawRate.Ki,RollRate.Kd,PitchRate.Kd,YawRate.Kd,Stabilize_Roll.Kp,Stabilize_Pitch.Kp,Stabilize_Yaw.Kp);
		

	HMC5883_calib=0; 

}	//HMC5883L_Save_Calib()
