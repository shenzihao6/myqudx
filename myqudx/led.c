#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP
#include "led.h"
U16 ledflag1=0,ledflag2=0;
void led1LowFlash(void)
{
	ledflag1++;
	if(ledflag1==600)
	{
		P7.1=1;
		P7.3=0;
	}
	else if(ledflag1==1200)
	{
		P7.1=0;
	        P7.3=1;
		ledflag1=0;
	}
	
}
void led1FastFlash(void)
{
	ledflag2++£»
	if(ledflag2==50)
	{
		P7.1=1;
	        P7.3=0;;
		
	}
	else if(ledflag2==100)
	{
		P7.1=0;
	        P7.3=1;
		ledflag2=0;
	}
}


