#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP

#include "pwmin.h"
U8 ARMED=0;
#define RC_MIN				1200
#define	RC_MAX				1800
#define RC_FUN_CNT	80

void RC_Data_Anl(void)
{
	static U8 fun_cnt = 0;

	
	if(RC_Data.THROTTLE < RC_MIN)
	{
		if(RC_Data.YAW < RC_MIN)
		{
			fun_cnt++;
			if(fun_cnt==RC_FUN_CNT)
			{	
				ARMED = 1;
				P7.2=0;
			}
		}
		else if(RC_Data.YAW > RC_MAX)
		{
			fun_cnt++;
			if(fun_cnt==RC_FUN_CNT)
			{	
				ARMED = 0;
				P7.2=1;
			}	
		}
		else
			fun_cnt = 0;
	}
	
	
}	
