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
* File Name    : r_cg_intc_user.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements device driver for INTC module.
* Creation Date: 2015/8/12
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt INTP1 r_intc1_interrupt
#pragma interrupt INTP2 r_intc2_interrupt
#pragma interrupt INTP3 r_intc3_interrupt
#pragma interrupt INTP8 r_intc8_interrupt
#pragma interrupt INTP9 r_intc9_interrupt
#pragma interrupt INTP10 r_intc10_interrupt
#pragma interrupt INTP11 r_intc11_interrupt
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_intc.h"
/* Start user code for include. Do not edit comment generated here */
#include "pwmin.h"
#include "r_cg_timer.h"
#include "systimer.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
volatile struct Rc_Data RC_Data;
volatile U16 PWM_Input_CH1,PWM_Input_CH2,PWM_Input_CH3,PWM_Input_CH4,PWM_Input_CH5,PWM_Input_CH6£¬PWM_Input_CH7;
//volatile U16 RC_data[7]
volatile uint32_t CH1RisingTime, CH2RisingTime, CH3RisingTime, CH4RisingTime, CH5RisingTime, CH6RisingTime, CH7RisingTime£¬
CH1FallingTime,CH2FallingTime,CH3FallingTime,CH4FallingTime,CH5FallingTime,CH6FallingTime£¬CH7FallingTime;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_intc1_interrupt
* Description  : This function is INTP1 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_intc1_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    //p50
    
    uint32_t pwmin_temp;
	if( P5.0 ==1)
	{ 
		CH1RisingTime = micros();	
	}
	else
	{ 
		CH1FallingTime = micros(); 
		if(CH1FallingTime < CH1RisingTime)
		{
			return ; 
		}	
	
	       else
		{
			pwmin_temp = CH1FallingTime - CH1RisingTime;//T2-T1
		}
	if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
	{
		PWM_Input_CH1 = pwmin_temp;
		RC_Data.AUX1	= PWM_Input_CH1;
	}
        }
	
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_intc2_interrupt
* Description  : This function is INTP2 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_intc2_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    //p51
    
       uint32_t pwmin_temp;
	if( P5.1 ==1)
	{ 
		CH2RisingTime = micros();	
	}
	else
	{ 
		CH2FallingTime = micros(); 
		if(CH2FallingTime < CH2RisingTime)
		{
			return ; 
		}
	
		else
		{
			pwmin_temp = CH2FallingTime - CH2RisingTime;//T2-T1
		}
	if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
	{
		PWM_Input_CH2 = pwmin_temp;
		RC_Data.AUX2  = PWM_Input_CH2;
	}
	}
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_intc3_interrupt
* Description  : This function is INTP3 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_intc3_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    //p30
    
    uint32_t pwmin_temp;
	if( P3.0 ==1)
	{ 
		CH3RisingTime = micros();	
	}
	else
	{ 
		CH3FallingTime = micros(); 
		if(CH3FallingTime < CH3RisingTime)
		{
			return ; 
		}
	
		else
		{
			pwmin_temp = CH3FallingTime - CH3RisingTime;//T2-T1
		}
	if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
	{
		PWM_Input_CH3 = pwmin_temp;
		RC_Data.AUX3	= PWM_Input_CH3;
	}
	}
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_intc8_interrupt
* Description  : This function is INTP8 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_intc8_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    //p74
    uint32_t pwmin_temp;
	if( P7.4 ==1)
	{ 
		CH4RisingTime = micros();	
	}
	else
	{
		CH4FallingTime = micros(); 
		if(CH4FallingTime < CH4RisingTime)
		{
			return ; //³¬Ê± ·µ»Ø
		}	
	
		else
		{
			pwmin_temp = CH4FallingTime - CH4RisingTime;//T2-T1
		}
	if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
	{
		PWM_Input_CH4 = pwmin_temp;
		RC_Data.ROLL	= PWM_Input_CH4;
	}
	}
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_intc9_interrupt
* Description  : This function is INTP9 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_intc9_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    //p75
    
    uint32_t pwmin_temp;
	if( P7.5 ==1)
	{ 
		CH5RisingTime = micros();	
	}
	else
	{ 
		CH5FallingTime = micros(); 
		if(CH5FallingTime < CH5RisingTime)
		{
			return ; 
		}
	
		else
		{
			pwmin_temp = CH5FallingTime - CH5RisingTime;//T2-T1
		}
	if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
	{
		PWM_Input_CH5 = pwmin_temp;
		RC_Data.PITCH	= PWM_Input_CH5;
	}
	}
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_intc10_interrupt
* Description  : This function is INTP10 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_intc10_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    //p76
     uint32_t pwmin_temp;
	if( P7.6 ==1)
	{ 
		CH6RisingTime = micros();	
	}
	else
	{ 
		CH6FallingTime = micros(); 
		if(CH6FallingTime < CH6RisingTime)
		{
			return ; 
		}
	
		else
		{
			pwmin_temp = CH6FallingTime - CH6RisingTime;//T2-T1
		}
	if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
	{
		PWM_Input_CH6 = pwmin_temp;
		RC_Data.YAW	= PWM_Input_CH6;
	}
}
	RC_Data_Anl();
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_intc11_interrupt
* Description  : This function is INTP11 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_intc11_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    //p77
    uint32_t pwmin_temp;
	if( P7.7 ==1)
	{ 
		CH7RisingTime = micros();	
	}
	else
	{ 
		CH7FallingTime = micros(); 
		if(CH7FallingTime < CH7RisingTime)
		{
			return ;
		}
	
		else
		{
			pwmin_temp = CH7FallingTime - CH7RisingTime;//T2-T1
		}
	if((pwmin_temp>(uint32_t)PWM_Input_MIN) && (pwmin_temp<(uint32_t)PWM_Input_MAX))
	{
		PWM_Input_CH7 = pwmin_temp;
		RC_Data.THROTTLE	= PWM_Input_CH7;
	}
	}
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
