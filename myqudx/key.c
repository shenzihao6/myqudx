#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP

#include "key.h"
int16_t accflag=0;
int16_t hmc5883lflag=0;
int16_t mode1flag=0;
int16_t mode2flag=0;
int16_t mode3flag=0;
void keyacc_gyro_fix(void)
{
	if(key1==0)
	{
		delay_ms(10);
		if(key1==0)
		{
			accflag++;
			if(accflag==500)
			{
				acc_gyro_flag=1;
				accflag=0;
			}	
			
		}
		else
		{
			key1=1;
		}	
	}
	else
	{
		key1=1;
	}	
}

void keyhmc5883l(void)
{
	if(key2==0)
	{
		delay_ms(10);
		if(key2==0)
		{
			hmc5883lflag++;
			if(hmc5883lflag==500)
			{
				HMC5883L_Start_Calib();
			}
			else if(hmc5883lflag==1000)
                        {
				HMC5883L_Save_Calib();
				hmc5883lflag=0;
			}  		
		}
		else
		{
			key2=1;
		}	
	}
	else
	{
		key2=1;
	}	
}
uint16_t fun_cnt1=0;
uint16_t fun_cnt2=0;
uint16_t fun_cnt3=0;
void mode1(void)
{
	if(key3==0)
	{
		delay_ms(10);
		if(key3==0)
		{
			mode1flag++;
			if(mode1flag==500)
			{
				mode1flag=0;
				fun_cnt1=1;
				ARMED=1;
				P7.2=0;
				   
			}
			
		}
		else
		{
			key3=1;
		}	
	}
	else
	{
		key3=1;
	}	
}
void mode2(void)
{
	if(key4==0)
	{
		delay_ms(10);
		if(key4==0)
		{
			mode2flag++;
			if(mode2flag==500)
			{
				mode2flag=0;	
			}
			
		}
		else
		{
			key4=1;
		}	
	}
	else
	{
		key4=1;
	}	
}
void mode3(void)
{
	if(key5==0)
	{
		delay_ms(10);
		if(key5==0)
		{
			mode3flag++;
			if(mode3flag==500)
			{
				mode3flag=0;
			}
			
		}
		else
		{
			key5=1;
		}	
	}
	else
	{
		key5=1;
	}	
}

uint16_t thradd;

void flymode1(void)
{
	
	if(fun_cnt1&&ARMED)
	{
		P7.2=0;
		thradd++;
		if(thradd==1000)
		{
			RC_Data.THROTTLE=1300;
		}
		else if(thradd==1500)
		{
			RC_Data.THROTTLE=1000;
			ARMED=0£»
			fun_cnt1=0;
		}
	}
	else
	{
		P7.2=1;
	}
	
}
void flymode2(void)
{
}
void flymode3(void)
{
}
void hongwaisacn(void)
{
	if(10001)
	{
		
	}
	if(10000||11000)
	{
		
	}
	
	if(10111)
	{
		
	}
	if(10011)
	{
		
	}
	if(11001)
	{
		
	}
	if(11101)
	{
		
	}
	if(11110)
	{
	  //flag
	}
	if(01111)
	{
	  //~flag	
	}
	
}