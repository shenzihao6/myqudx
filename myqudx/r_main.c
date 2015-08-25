/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only 
* intended for use with Renesas products. No other uses are authorized. This 
* software is owned by Renesas Electronics Corporation and is protected under 
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING 
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT 
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
* AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR 
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE 
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software 
* and to discontinue the availability of this software.  By using this software, 
* you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2013 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_main.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements main function.
* Creation Date: 2015/8/12
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_port.h"
#include "r_cg_intc.h"
#include "r_cg_serial.h"
#include "r_cg_timer.h"
#include "r_cg_it.h"
/* Start user code for include. Do not edit comment generated here */
#include "mpu6050.h"
#include "hmc5883l.h"
#include "iic.h"
#include "altitude.h"
#include "dataSend.h"
#include "eeprom.h"
#include "systimer.h"
#include "pwmout.h"
#include "led.h"
#include "key.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
 float ypr[3];
 U8 flag=0; 
 uint16_t flagthr=0;
/* End user code. Do not edit comment generated here */
void R_MAIN_UserInit(void);

/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void main(void)
{
    R_MAIN_UserInit();
    /* Start user code. Do not edit comment generated here */
   	P7.0=1;
   	P7.1=1;
   	P7.2=1;
        P7.3=1;       
	key1=1;
	key2=1;
	key3=1;
	key4=1;
	key5=1;
	
      R_IICA0_Create();
      R_TAU0_Channel0_Start();
      R_TAU0_Channel5_Start();
      set_pwm(1000,1000,1000,1000);
      //init the interrupt;
      //R_INTC1_Start();
     // R_INTC2_Start();
     // R_INTC3_Start();
     // R_INTC8_Start();
     // R_INTC9_Start();
     // R_INTC10_Start();  
     // R_INTC11_Start();
     // R_UART0_Start();  //start serial.then you can send the data...
     // R_UART1_Start();
     // Data_FAL_Init(); //init eeprom
     // delay_ms(50);
    
     eeprom_read(&Acc_X_Offset,&Acc_Y_Offset,&Gyro_X_Offset,&Gyro_Y_Offset,&Gyro_Z_Offset,&Hmc5883l_X_Offset,&Hmc5883l_Y_Offset,&Hmc5883l_Z_Offset,
     &RollRate.Kp,&PitchRate.Kp,&YawRate.Kp,&RollRate.Ki,&PitchRate.Ki,&YawRate.Ki,&RollRate.Kd,&PitchRate.Kd,&YawRate.Kd,&Stabilize_Roll.Kp,&Stabilize_Pitch.Kp,&Stabilize_Yaw.Kp);
    //id=init_mpu6050(
     IMU_init();
     delay_ms(50);
     Initial_Math();
      RC_Data.YAW=0;
      RC_Data.THROTTLE=0;
      RC_Data.ROLL=0;
      RC_Data.PITCH=0;
    while (1U)
    {  
       
       keyacc_gyro_fix();
       keyhmc5883l();
        mode1();
	flymode1();
       IMU_getYawPitchRoll(ypr);
       Quadrotor_Motor_Update();
       Send_Data(); 
       led1LowFlash();
       
      if(RC_Data.AUX2>1550)
      {
      	prate_hight();

      }
	 
        ;
    }
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MAIN_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
    EI();
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
