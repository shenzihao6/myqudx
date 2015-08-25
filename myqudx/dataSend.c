#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP
#include "dataSend.h"

extern  float ypr[3];
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
U8 pid_flag=0;
U8 SendPidDataFlag=0;
U8 Data_Check,Send_Status,Send_Senser,Send_RCData,Send_GpsData,Send_Offset,Send_PID1,Send_PID2,Send_PID3,Send_MotoPwm;
static U8 data_to_send[120];
void Send_Data(void)
{
	if(Send_Status==2)
	{
		Data_Send_Status();
	}	
	else if(Send_Status==4)
	{
		Data_Send_RCData();
		
	}
	else if(Send_Status==6)
	{
		Data_Send_MotoPWM();
	}    
	else if(Send_Status==8)  
	{
		Data_Send_Senser();
		
	}
	else if(Send_Status==10)
	{
		if(SendPidDataFlag==1)
		{
			Data_Send_PID1();
		}	
	}
	else if(Send_Status==12)
	{      if(SendPidDataFlag==1)
	       {
		Data_Send_PID2();
		SendPidDataFlag=0;
	       }
		Send_Status=0;
	}
	       
	
	Send_Status++;
}
void Data_Send_Status(void)
{
	U8 _cnt=0;
	 int16_t _temp;
	 U8 sum = 0;
	 U8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(ypr[2]*100);//roll
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(ypr[1]*100);//pitch
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	//_temp = (int)(Q_ANGLE.YAW*100);
	_temp = (int)(ypr[0]*100);//yaw
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
//	vs32 _temp2 = BaroAlt;
//	data_to_send[_cnt++]=BYTE3(_temp2);
//	data_to_send[_cnt++]=BYTE2(_temp2);
//	data_to_send[_cnt++]=BYTE1(_temp2);
//	data_to_send[_cnt++]=BYTE0(_temp2);
		
//	if(ARMED==0)				data_to_send[_cnt++]=0xA0;	//???¡§
//	else if(ARMED==1)		data_to_send[_cnt++]=0xA1;
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	R_UART0_Send(data_to_send,_cnt);

}
void Data_Send_RCData(void)
{
	U8 _cnt=0;
	U8 sum = 0;
	U8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(RC_Data.THROTTLE);
	data_to_send[_cnt++]=BYTE0(RC_Data.THROTTLE);
	data_to_send[_cnt++]=BYTE1(RC_Data.YAW);
	data_to_send[_cnt++]=BYTE0(RC_Data.YAW);
	data_to_send[_cnt++]=BYTE1(RC_Data.ROLL);
	data_to_send[_cnt++]=BYTE0(RC_Data.ROLL);
	data_to_send[_cnt++]=BYTE1(RC_Data.PITCH);
	data_to_send[_cnt++]=BYTE0(RC_Data.PITCH);
	data_to_send[_cnt++]=BYTE1(RC_Data.AUX1);
	data_to_send[_cnt++]=BYTE0(RC_Data.AUX1);
	data_to_send[_cnt++]=BYTE1(RC_Data.AUX2);
	data_to_send[_cnt++]=BYTE0(RC_Data.AUX2);
	data_to_send[_cnt++]=BYTE1(RC_Data.AUX3);
	data_to_send[_cnt++]=BYTE0(RC_Data.AUX3);
	//data_to_send[_cnt++]=BYTE1(RC_Data.AUX4);
	//data_to_send[_cnt++]=BYTE0(RC_Data.AUX4);
//	data_to_send[_cnt++]=BYTE1(RC_Data.AUX5);
//	data_to_send[_cnt++]=BYTE0(RC_Data.AUX5);
//	data_to_send[_cnt++]=BYTE1(RC_Data.AUX6);
//	data_to_send[_cnt++]=BYTE0(RC_Data.AUX6);
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	

	R_UART0_Send(data_to_send,_cnt);

}
void Data_Send_MotoPWM(void)
{
	U8 _cnt=0;
	U8 sum = 0;
	U8 i=0;
	volatile U16 temp1=	Moto_PWM_3-1000;
	volatile U16 temp2=	Moto_PWM_4-1000;
	volatile U16 temp3=	Moto_PWM_1-1000;
	volatile U16 temp4=	Moto_PWM_2-1000;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1( temp1);
	data_to_send[_cnt++]=BYTE0( temp1);
	data_to_send[_cnt++]=BYTE1( temp2);
	data_to_send[_cnt++]=BYTE0( temp2);
	data_to_send[_cnt++]=BYTE1( temp3);
	data_to_send[_cnt++]=BYTE0( temp3);
	data_to_send[_cnt++]=BYTE1( temp4);
	data_to_send[_cnt++]=BYTE0( temp4);
//	data_to_send[_cnt++]=BYTE1(moto1);
//	data_to_send[_cnt++]=BYTE0(moto1);
//	data_to_send[_cnt++]=BYTE1(moto1);
//	data_to_send[_cnt++]=BYTE0(moto1);
//	data_to_send[_cnt++]=BYTE1(moto1);
//	data_to_send[_cnt++]=BYTE0(moto1);
//	data_to_send[_cnt++]=BYTE1(moto1);
//	data_to_send[_cnt++]=BYTE0(moto1);
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	

	R_UART0_Send(data_to_send,_cnt);
}
 int16_t ax1, ay1, az1;	
 int16_t mx, my, mz;	
 int16_t gx1, gy1, gz1;
 int16_t hx, hy, hz;
 extern float mygetqval[9];
 void Data_Send_Senser(void)
{
	U8 _cnt=0;
	U8 sum = 0;
	U8 i=0;
	ax1=(volatile int16_t)mygetqval[0];
	ay1=(volatile int16_t)mygetqval[1];
	az1=(volatile int16_t)mygetqval[2];
	gx1=(volatile int16_t)mygetqval[3];
	gy1=(volatile int16_t)mygetqval[4];
	gz1=(volatile int16_t)mygetqval[5];
	mx=(volatile int16_t)mygetqval[6];
	my=(volatile int16_t)datafliter;
	mz=(volatile int16_t)Alt_CSB;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(ax1);
	data_to_send[_cnt++]=BYTE0(ax1);
	data_to_send[_cnt++]=BYTE1(ay1);
	data_to_send[_cnt++]=BYTE0(ay1);
	data_to_send[_cnt++]=BYTE1(az1);
	data_to_send[_cnt++]=BYTE0(az1);
	data_to_send[_cnt++]=BYTE1(gx1);
	data_to_send[_cnt++]=BYTE0(gx1);
	data_to_send[_cnt++]=BYTE1(gy1);
	data_to_send[_cnt++]=BYTE0(gy1);
	data_to_send[_cnt++]=BYTE1(gz1);
	data_to_send[_cnt++]=BYTE0(gz1);
	data_to_send[_cnt++]=BYTE1(accgyroval[0]);
	data_to_send[_cnt++]=BYTE0(accgyroval[0]);
	data_to_send[_cnt++]=BYTE1(my);
	data_to_send[_cnt++]=BYTE0(my);
	data_to_send[_cnt++]=BYTE1(mz);
	data_to_send[_cnt++]=BYTE0(mz);
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	R_UART0_Send(data_to_send,_cnt);
}
void Data_Send_PID1(void)
{

	U8 _cnt=0;
	volatile int16_t _temp;
	U8 sum = 0;
	U8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;
	data_to_send[_cnt++]=0;
	
	if(pid_flag==1)
	{
	_temp = RollRate.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollRate.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollRate.Kd * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PitchRate.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PitchRate.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PitchRate.Kd * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = YawRate.Kp  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = YawRate.Ki  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = YawRate.Kd  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	}
	else
	{
	_temp = RollRate.Kp ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollRate.Ki ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollRate.Kd ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PitchRate.Kp ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PitchRate.Ki ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = PitchRate.Kd ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = YawRate.Kp ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = YawRate.Ki ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = YawRate.Kd ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	}
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	R_UART0_Send(data_to_send,_cnt);

}
void Data_Send_PID2(void)
{
	U8 _cnt=0;
	volatile int16_t _temp;
	U8 sum = 0;
	U8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x11;
	data_to_send[_cnt++]=0;
	
	if(pid_flag==1)
	{
	_temp = Stabilize_Roll.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Roll.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =Stabilize_Roll.Kd * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Pitch.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Pitch.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Pitch.Kd * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Yaw.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Yaw.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =Stabilize_Yaw.Kd * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
        }
        else
	{
	_temp = Stabilize_Roll.Kp ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Roll.Ki ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =Stabilize_Roll.Kd ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Pitch.Kp ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Pitch.Ki ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Pitch.Kd ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Yaw.Kp ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Yaw.Ki ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =Stabilize_Yaw.Kd ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	}
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	

	R_UART0_Send(data_to_send,_cnt);

}