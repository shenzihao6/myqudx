#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP

#include "dataRec.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


U8 acc_gyro_flag=0;
static U8 data_to_send[120];


void Data_Send_Check(U16 check)
{
	U8 sum = 0;
	U8 i=0;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xF0;
	data_to_send[3]=3;
	data_to_send[4]=0xBA;
	
	data_to_send[5]=BYTE1(check);
	data_to_send[6]=BYTE0(check);
	
	for( i=0;i<7;i++)
		sum += data_to_send[i];
		
	data_to_send[7]=sum;

	R_UART0_Send(data_to_send,8);
}

void Data_Receive_Anl(U8 *data_buf,U8 num)
{
	volatile  int16_t rc_value_temp;
	U8 sum = 0;
	U8 i=0;
	for( i=0;i<(num-1);i++)
	{
		sum += *(data_buf+i);
	}	
	if(!(sum==*(data_buf+num-1)))	
	{
		return;
	}	
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))
	{
		return;
	}
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
			acc_gyro_flag=1;
		if(*(data_buf+4)==0X02)
			acc_gyro_flag=1;
		if(*(data_buf+4)==0X03)
		{}
		if(*(data_buf+4)==0X04)
		{
			HMC5883L_Start_Calib();
		}
		if(*(data_buf+4)==0X05)
		{
			HMC5883L_Save_Calib();
		}//MS5611_CalOffset();
	}
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
	  	
			 SendPidDataFlag=1;//once is ok
		 // Data_Send_PID3();
		}
		if(*(data_buf+4)==0X02)
		{
			;
		}
			//Send_Offset = 1;
	}
	if(*(data_buf+2)==0X10)								//PID1
	{     
		
		/*
			RollRate.Kp = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			RollRate.Ki = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			RollRate.Kd = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			PitchRate.Kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			PitchRate.Ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			PitchRate.Kd = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			YawRate.Kp = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			YawRate.Ki = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
			YawRate.Kd = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
		*/	
		        RollRate.Kp = (float)((volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
			RollRate.Ki = (float)((volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
			RollRate.Kd = (float)((volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
			PitchRate.Kp = (float)((volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
			PitchRate.Ki = (float)((volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
			PitchRate.Kd = (float)((volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
			YawRate.Kp = (float)((volatile int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
			YawRate.Ki = (float)((volatile int16_t)(*(data_buf+18)<<8)|*(data_buf+19));
			YawRate.Kd = (float)((volatile int16_t)(*(data_buf+20)<<8)|*(data_buf+21));
			pid_flag=0;
			
			Data_Send_PID1();
		 eeprom_write(Acc_X_Offset,Acc_Y_Offset,Gyro_X_Offset,Gyro_Y_Offset,Gyro_Z_Offset,Hmc5883l_X_Offset,Hmc5883l_Y_Offset,Hmc5883l_Z_Offset,
    (int16_t)RollRate.Kp,(int16_t)PitchRate.Kp,(int16_t)YawRate.Kp,(int16_t)RollRate.Ki,(int16_t)PitchRate.Ki,(int16_t)YawRate.Ki,(int16_t)RollRate.Kd,(int16_t)PitchRate.Kd,(int16_t)YawRate.Kd,(int16_t)Stabilize_Roll.Kp,(int16_t)Stabilize_Pitch.Kp,(int16_t)Stabilize_Yaw.Kp);
		    
		   Data_Send_Check(sum);
		  
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
		/*
			Stabilize_Roll.Kp =  (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
		  Stabilize_Roll.Ki  = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			Stabilize_Roll.Kd  = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			Stabilize_Pitch.Kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			Stabilize_Pitch.Ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			Stabilize_Pitch.Kd=  (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
	    Stabilize_Yaw.Kp =   (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
		  Stabilize_Yaw.Ki=    (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
	    Stabilize_Yaw.Kd =   (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
//		Send_PID2 = 1;
		 */
		 Stabilize_Roll.Kp =  (float)((volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
		  Stabilize_Roll.Ki  = (float)((volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
			Stabilize_Roll.Kd  = (float)((volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
			Stabilize_Pitch.Kp = (float)((volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
			Stabilize_Pitch.Ki = (float)((volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
			Stabilize_Pitch.Kd=  (float)((volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
	    Stabilize_Yaw.Kp =   (float)((volatile int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
		  Stabilize_Yaw.Ki=    (float)((volatile int16_t)(*(data_buf+18)<<8)|*(data_buf+19));
	    Stabilize_Yaw.Kd =   (float)((volatile int16_t)(*(data_buf+20)<<8)|*(data_buf+21));
	              pid_flag=0;
		      Data_Send_PID2();
		      eeprom_write(Acc_X_Offset,Acc_Y_Offset,Gyro_X_Offset,Gyro_Y_Offset,Gyro_Z_Offset,Hmc5883l_X_Offset,Hmc5883l_Y_Offset,Hmc5883l_Z_Offset,
    (int16_t)RollRate.Kp,(int16_t)PitchRate.Kp,(int16_t)YawRate.Kp,(int16_t)RollRate.Ki,(int16_t)PitchRate.Ki,(int16_t)YawRate.Ki,(int16_t)RollRate.Kd,(int16_t)PitchRate.Kd,(int16_t)YawRate.Kd,(int16_t)Stabilize_Roll.Kp,(int16_t)Stabilize_Pitch.Kp,(int16_t)Stabilize_Yaw.Kp);
		      
		   Data_Send_Check(sum);
		   
	}
	if(*(data_buf+2)==0X12)								//PID3
	{
		/*
	        Climb.Kp = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
		Climb.Ki  = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
		Climb.Kd  = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
		
			AutoHigh_THR.Kp= (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
		  AutoHigh_THR.Ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			AutoHigh_THR.Kd= (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//	    Climb.Kd = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//		  Stabilize_Yaw.Ki= (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
//	    Stabilize_Yaw.Kd = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
//		Send_PID2 = 1;
		 
		      Data_Send_PID3();
		     Data_Save();
		*/
			Data_Send_Check(sum);
			
	}
	if(*(data_buf+2)==0X13)								//PID4
	{
//		PID_PID_5.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//		PID_PID_5.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//		PID_PID_5.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//		PID_PID_6.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//		PID_PID_6.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//		PID_PID_6.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//		PID_PID_7.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//		PID_PID_7.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
//		PID_PID_7.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
		if(*(data_buf+2)==0X14)								//PID5
	{
//			PID_PID_8.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_PID_8.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_PID_8.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_PID_9.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_PID_9.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_PID_9.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//			PID_PID_10.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//			PID_PID_10.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
//			PID_PID_10.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
		if(*(data_buf+2)==0X15)								//PID6
	{
//			PID_PID_8.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_PID_8.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_PID_8.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_PID_9.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_PID_9.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_PID_9.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//			PID_PID_10.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//			PID_PID_10.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
//			PID_PID_10.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X16)								//OFFSET
	{
//	Q_Angle_Offset_Rol = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
//	Q_Angle_Offset_Pit = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
//	Data_Save();
			Data_Send_Check(sum);
	}
	
	
	
	
	
///////////////////////////////////////////////////////////////////////////////////////////////
	if(*(data_buf+2)==0x03)								//?D??1|?¨¹¡Á?,=0x8a,?a¨°¡ê??¨ºy?Y
	{
//		U8 _cnt = 4;
//		#ifdef	RC_USE_DEADBAND
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.THROTTLE = RC_MIDDLE;
//			else
//				Rc_Data.THROTTLE = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.YAW = RC_MIDDLE;
//			else
//				Rc_Data.YAW = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.ROLL = RC_MIDDLE;
//			else
//				Rc_Data.ROLL = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.PITCH = RC_MIDDLE;
//			else
//				Rc_Data.PITCH = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX1 = RC_MIDDLE;
//			else
//				Rc_Data.AUX1 = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX2 = RC_MIDDLE;
//			else
//				Rc_Data.AUX2 = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX3 = RC_MIDDLE;
//			else
//				Rc_Data.AUX3 = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX4 = RC_MIDDLE;
//			else
//				Rc_Data.AUX4 = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX5 = RC_MIDDLE;
//			else
//				Rc_Data.AUX5 = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX6 = RC_MIDDLE;
//			else
//				Rc_Data.AUX6 = rc_value_temp;
//		#else
//			Rc_Data.THROTTLE 	= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.YAW				= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.ROLL			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.PITCH			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX1			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX2			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX3			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX4			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX5			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX6			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//		#endif
//		Rc_DataCal();
	}
}
