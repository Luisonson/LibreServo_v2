/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LS_PID.c
  * @brief          : The code for the PIDs
  * @author			: Luis Picó Chausson
  * @version		: 0.1
  * Created on: 30 mar. 2022
  ******************************************************************************
  * @attention
  *
  * LibreServo by Luis Picó is licensed under a Creative Commons 
  * Attribution-NonCommercial-ShareAlike 4.0 International License
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

struct LS_comando_motor {uint8_t comando; long pos_now; float velocidad; float aceleracion; float velocidad_int; float aceleracion_int; long pos_destino; float pos_destino_f; long pos_ini; long pos_final; long time_to_end; float step; long time_ramp1; long time_ramp2; float step_ramp1; float step_ramp2; int output_LS;};
extern volatile struct LS_comando_motor comando_motor;			//Definido en LS_funciones.c
long pos_bef_PID_LS,pos_bef_PID_M;
volatile float ITerm_LS = 0.0;
volatile float ITerm_M = 0.0;
volatile float K_P_LS_f, K_D_LS_f, K_I_LS_f;
volatile float K_P_M_f, K_D_M_f, K_I_M_f;

extern long buff_sensores[TAM_BUFF_SENSORES];	//Inicializado en main.c

extern volatile int corriente;					//Inicializado en LS_funciones.c
extern uint16_t corte_corriente;				//Inicializado en main.c
extern volatile uint16_t deadband; 				//Inicializado en LS_flash.c
extern volatile uint8_t direccion_motor;		//Inicializado en LS_flash.c

void PID_LS()
{
	float error, Derror, PDterm;
	long TIM_autoreload = TIM1->ARR;
	long output_LS, l_temp, old_output_LS;

	error = comando_motor.pos_destino - comando_motor.pos_now;
	if(error > deadband) error = error - deadband;
	else if(error < -deadband) error = error + deadband;
	else
	{
		error = 0;
		pos_bef_PID_LS = comando_motor.pos_now;
	}

	Derror = comando_motor.pos_now - pos_bef_PID_LS;
	pos_bef_PID_LS = comando_motor.pos_now;
	ITerm_LS = ITerm_LS + error * K_I_LS_f;
	PDterm = K_P_LS_f * error - K_D_LS_f * Derror;

	if (PDterm > TIM_autoreload) PDterm = TIM_autoreload;
	else if (PDterm < -TIM_autoreload) PDterm = -TIM_autoreload;

	if (ITerm_LS >= 0)
	{
		if (ITerm_LS > TIM_autoreload - PDterm) ITerm_LS = (TIM_autoreload - PDterm);
	}
	else
	{
		if (-ITerm_LS > TIM_autoreload + PDterm) ITerm_LS = -(TIM_autoreload + PDterm);
	}

	output_LS = PDterm + ITerm_LS;
	
	comando_motor.output_LS = output_LS;

	if (TIM1->CCR1 == 0) old_output_LS = TIM1->CCR2;
	else old_output_LS = TIM1->CCR1;
	
	if ( corriente != 0 && old_output_LS != 0) l_temp = ((long)corte_corriente * old_output_LS) / corriente;
	else l_temp = TIM_autoreload;			//Máximo valor posible
	
	if ( output_LS > l_temp ) output_LS = l_temp;
	else if ( output_LS < -l_temp ) output_LS = -l_temp;

	if(direccion_motor == 0)
	{
		if(output_LS >=0)
		{
			TIM1->CCR2 = 0;
			TIM1->CCR1 = output_LS;
		}
		else
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = -output_LS;
		}
	}
	else
	{
		if(output_LS >=0)
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = output_LS;
		}
		else
		{
			TIM1->CCR2 = 0;
			TIM1->CCR1 = -output_LS;
		}
	}

	if (error >= 0) l_temp = error + 0.5f;
	else l_temp = error - 0.5f;
	buff_sensores[19] = l_temp;
	if (Derror >= 0) l_temp = Derror + 0.5f;
	else l_temp = Derror - 0.5f;
	buff_sensores[20] = l_temp;
	if (PDterm >= 0) l_temp = PDterm + 0.5f;
	else l_temp = PDterm - 0.5f;
	buff_sensores[21] = l_temp;
	if (ITerm_LS >= 0) l_temp = ITerm_LS + 0.5f;
	else l_temp = ITerm_LS - 0.5f;
	buff_sensores[22] = l_temp;
}

void PID_M()
{
	float error, Derror, PDterm;
	long TIM_autoreload = TIM1->ARR;
	long output_M, l_temp, old_output_M;

	error = comando_motor.pos_destino - comando_motor.velocidad;
	if(error > deadband) error = error - deadband;
	else if(error < -deadband) error = error - deadband;
	else error = 0;

	Derror = comando_motor.velocidad - pos_bef_PID_M;
	pos_bef_PID_M = comando_motor.velocidad;
	ITerm_M = ITerm_M + error * K_I_M_f;
	PDterm = K_P_M_f * error - K_D_M_f * Derror;

	if (ITerm_M >= 0)
	{
		if (ITerm_M > PDterm - TIM_autoreload) ITerm_M = (PDterm - TIM_autoreload) - 1;
	}
	else
	{
		if (-PDterm > TIM_autoreload + ITerm_M) ITerm_M = (TIM_autoreload + PDterm) + 1;
	}
	if (ITerm_M > TIM_autoreload) ITerm_M = TIM_autoreload;
	else if (ITerm_M < -TIM_autoreload) ITerm_M = -TIM_autoreload;

	output_M = PDterm + ITerm_M;
	
	comando_motor.output_LS = output_M;

	if (TIM1->CCR1 == 0) old_output_M = TIM1->CCR2;
	else old_output_M = TIM1->CCR1;
	
	if ( corriente != 0 && old_output_M != 0) l_temp = ((long)corte_corriente * old_output_M) / corriente;
	else l_temp = TIM_autoreload;			//Máximo valor posible
	
	if ( output_M > l_temp ) output_M = l_temp;
	else if ( output_M < -l_temp ) output_M = -l_temp;
	
	if(direccion_motor == 0)
	{
		if(output_M >=0)
		{
			TIM1->CCR2 = 0;
			TIM1->CCR1 = output_M;
		}
		else
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = -output_M;
		}
	}
	else
	{
		if(output_M >=0)
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = output_M;
		}
		else
		{
			TIM1->CCR2 = 0;
			TIM1->CCR1 = -output_M;
		}
	}

	if (error >= 0) l_temp = error + 0.5f;
	else l_temp = error - 0.5f;
	buff_sensores[19] = l_temp;
	if (Derror >= 0) l_temp = Derror + 0.5f;
	else l_temp = Derror - 0.5f;
	buff_sensores[20] = l_temp;
	if (PDterm >= 0) l_temp = PDterm + 0.5f;
	else l_temp = PDterm - 0.5f;
	buff_sensores[21] = l_temp;
	if (ITerm_LS >= 0) l_temp = ITerm_LS + 0.5f;
	else l_temp = ITerm_LS - 0.5f;
	buff_sensores[22] = l_temp;
}

void not_PID()
{
	long l_temp, old_output_LS, output_LS;
	long TIM_autoreload = TIM1->ARR;

	if (TIM1->CCR1 == 0) old_output_LS = TIM1->CCR2;
	else old_output_LS = TIM1->CCR1;

	if ( corriente != 0 && old_output_LS != 0) l_temp = ((long)corte_corriente * old_output_LS) / corriente;
	else l_temp = TIM_autoreload;			//Máximo valor posible

	if ( comando_motor.output_LS > l_temp ) output_LS = l_temp;
	else if ( comando_motor.output_LS < -l_temp ) output_LS = -l_temp;
	else output_LS = comando_motor.output_LS;

	if(direccion_motor == 0)
	{
		if(output_LS >=0)
		{
			TIM1->CCR2 = 0;
			TIM1->CCR1 = output_LS;
		}
		else
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = -output_LS;
		}
	}
	else
	{
		if(output_LS >=0)
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = output_LS;
		}
		else
		{
			TIM1->CCR2 = 0;
			TIM1->CCR1 = -output_LS;
		}
	}
}
