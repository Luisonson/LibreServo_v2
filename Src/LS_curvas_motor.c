/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LS_curvas_motor.c
  * @brief          : motion curve functions
  * @author			: Luis Picó Chausson
  * @version		: 0.1
  * Created on: 4 abr. 2022
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

struct LS_task_scheduler {uint8_t comando[LS_TASK_SCHEDULER_SIZE];long param_1[LS_TASK_SCHEDULER_SIZE];long param_2[LS_TASK_SCHEDULER_SIZE];long param_3[LS_TASK_SCHEDULER_SIZE];long param_4[LS_TASK_SCHEDULER_SIZE];long param_5[LS_TASK_SCHEDULER_SIZE];long time_comando[LS_TASK_SCHEDULER_SIZE];uint8_t iniciado[LS_TASK_SCHEDULER_SIZE];};
extern volatile struct LS_task_scheduler task_scheduler;			//Inicializado en LS_funciones.c

extern volatile uint8_t LS_TS_ESTADO, LS_TEMP_TS_GUARDADO, LS_TS_GUARDADO;			//Inicializado en LS_funciones.c

struct LS_comando_motor {uint8_t comando; long pos_now; float velocidad; float aceleracion; float velocidad_int; float aceleracion_int; long pos_destino; float pos_destino_f; long pos_ini; long pos_final; long time_to_end; float step; long time_ramp1; long time_ramp2; float step_ramp1; float step_ramp2; int output_LS;};
extern volatile struct LS_comando_motor comando_motor;

extern uint16_t t_ramp_t, t_ramp_s, a_ramp_t;			//Inicilizado en LS_flash.c

extern volatile long min_posicion, max_posicion;		//Inicializado en LS_flash.c Min y máx posición (0-4294967294 -> -2147483647 - 2147483647)
extern volatile uint8_t limit_posicion;					//Inicializado en LS_flash.c limites posición (0=False, 1=True)

void comando_M()
{
	float f_temp1, f_now;
	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0)
	{
		if (task_scheduler.comando[LS_TS_ESTADO] == 1)
		{
			f_now = comando_motor.pos_now;
			comando_motor.comando = 1;
		}
		else
		{
			f_now = comando_motor.velocidad;
			comando_motor.comando = 30;
		}
		
		if( limit_posicion == 1 )
		{
			if( max_posicion >= min_posicion )
			{
				if( task_scheduler.param_1[LS_TS_ESTADO] > max_posicion) task_scheduler.param_1[LS_TS_ESTADO] = max_posicion;
				else if( task_scheduler.param_1[LS_TS_ESTADO] < min_posicion) task_scheduler.param_1[LS_TS_ESTADO] = min_posicion;
			}
		}
		
		task_scheduler.iniciado[LS_TS_ESTADO] = 1;
		comando_motor.pos_final = task_scheduler.param_1[LS_TS_ESTADO];
		f_temp1 = comando_motor.pos_final - f_now;
		if(task_scheduler.param_2[LS_TS_ESTADO] > 1)
		{	
			comando_motor.time_to_end = task_scheduler.param_2[LS_TS_ESTADO];
			comando_motor.step = f_temp1 / (float)comando_motor.time_to_end;
			comando_motor.time_to_end = comando_motor.time_to_end - 1;
			comando_motor.pos_destino_f = f_now + comando_motor.step;
			if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
			else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
		}
		else 
		{	
			comando_motor.time_to_end = 0;
			comando_motor.pos_destino_f = comando_motor.pos_final;
			comando_motor.pos_destino = comando_motor.pos_final;
			task_scheduler.iniciado[LS_TS_ESTADO] = 2;
		}
		
	}
	else if (task_scheduler.iniciado[LS_TS_ESTADO] == 1)
	{
		comando_motor.time_to_end = comando_motor.time_to_end - 1;
		if (comando_motor.time_to_end > 0)
		{
			comando_motor.pos_destino_f = comando_motor.pos_destino_f + comando_motor.step;
			if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
			else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
		}
		else
		{
			comando_motor.pos_destino_f = comando_motor.pos_final;
			comando_motor.pos_destino = comando_motor.pos_final;
			task_scheduler.iniciado[LS_TS_ESTADO] = 2;
		}
	}
	else
	{
		comando_motor.time_to_end = 0;
		comando_motor.pos_destino_f = comando_motor.pos_final;
		comando_motor.pos_destino = comando_motor.pos_final;
		task_scheduler.iniciado[LS_TS_ESTADO] = 2;
	}
}

void comando_MT()
{
	float f_temp1, f_now;

	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0)
	{
		if (task_scheduler.comando[LS_TS_ESTADO] != 31)
		{
			f_now = comando_motor.pos_now;
			comando_motor.comando = 2;
		}
		else
		{
			f_now = comando_motor.velocidad;
			comando_motor.comando = 31;
		}

		if (task_scheduler.comando[LS_TS_ESTADO] == 3)
		{
			task_scheduler.param_3[LS_TS_ESTADO] = task_scheduler.param_2[LS_TS_ESTADO];
			task_scheduler.param_2[LS_TS_ESTADO] = task_scheduler.param_3[LS_TS_ESTADO]/2;
		}

		task_scheduler.iniciado[LS_TS_ESTADO] = 1;
		
		if( limit_posicion == 1 )
		{
			if( max_posicion >= min_posicion )
			{
				if( task_scheduler.param_1[LS_TS_ESTADO] > max_posicion) task_scheduler.param_1[LS_TS_ESTADO] = max_posicion;
				else if( task_scheduler.param_1[LS_TS_ESTADO] < min_posicion) task_scheduler.param_1[LS_TS_ESTADO] = min_posicion;
			}
		}
		
		comando_motor.pos_final = task_scheduler.param_1[LS_TS_ESTADO];
		if(task_scheduler.param_3[LS_TS_ESTADO] == LONG_MIN && task_scheduler.param_2[LS_TS_ESTADO] > 0)
		{
			comando_motor.time_ramp1 = t_ramp_t;
			comando_motor.time_ramp2 = 0;
			comando_motor.time_to_end = task_scheduler.param_2[LS_TS_ESTADO];
		}
		else if(task_scheduler.param_3[LS_TS_ESTADO] > 0)
		{
			comando_motor.time_ramp1 = task_scheduler.param_2[LS_TS_ESTADO];
			comando_motor.time_ramp2 = 0;
			comando_motor.time_to_end = task_scheduler.param_3[LS_TS_ESTADO];
		}
		else
		{
			comando_motor.time_to_end = 0;
		}

		if (2*comando_motor.time_ramp1 > comando_motor.time_to_end  && comando_motor.time_to_end > 3)
		{
			comando_motor.time_ramp1 = comando_motor.time_to_end / 2;
			comando_motor.time_ramp2 = 0;
		}
		else if(comando_motor.time_to_end <= 3)
		{
			comando_motor.time_ramp1 = 0;
			comando_motor.time_ramp2 = 0;
			comando_motor.time_to_end = 0;
			comando_motor.pos_final = task_scheduler.param_1[LS_TS_ESTADO];
			comando_motor.pos_destino_f = comando_motor.pos_final;
			comando_motor.pos_destino = comando_motor.pos_final;
			task_scheduler.iniciado[LS_TS_ESTADO] = 2;
		}

		if (comando_motor.time_ramp1 < 0) comando_motor.time_ramp1 = 0;

		if(comando_motor.time_to_end > 0)
		{
			f_temp1 = (float)(comando_motor.pos_final - f_now)/(float)(comando_motor.time_to_end-comando_motor.time_ramp1+1);
			if (comando_motor.time_ramp1 == 0) comando_motor.step_ramp1 = f_temp1;									//Aceleración
			else comando_motor.step_ramp1 = f_temp1/(float)comando_motor.time_ramp1;									//Aceleración
			comando_motor.step = f_temp1;																			//Velocidad lineal

			comando_motor.time_ramp2 = 1;
			comando_motor.time_to_end--;
			if(comando_motor.time_ramp1 == 0) comando_motor.pos_destino_f = f_now + comando_motor.step;
			else comando_motor.pos_destino_f = f_now + comando_motor.step_ramp1;			//e = v·t+at^2/2  v=vo·t+a·t
			if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
			else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
			task_scheduler.iniciado[LS_TS_ESTADO] = 1;
		}

	}
	else if (task_scheduler.iniciado[LS_TS_ESTADO] == 1)
	{
		if(comando_motor.time_ramp2 < comando_motor.time_ramp1)
		{
			comando_motor.time_ramp2++;
			comando_motor.time_to_end--;
			comando_motor.pos_destino_f = comando_motor.pos_destino_f + (comando_motor.step_ramp1 * comando_motor.time_ramp2);
			if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
			else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
		}
		else if (comando_motor.time_ramp2 > comando_motor.time_ramp1)
		{
			comando_motor.time_ramp1--;
			comando_motor.time_to_end--;
			if(comando_motor.time_to_end == 0)
			{
				comando_motor.pos_destino_f = comando_motor.pos_final;
				comando_motor.pos_destino = comando_motor.pos_final;
				task_scheduler.iniciado[LS_TS_ESTADO] = 2;
			}
			else
			{
				comando_motor.pos_destino_f = comando_motor.pos_destino_f + (comando_motor.step_ramp1 * comando_motor.time_ramp1);
				if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
				else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
			}
		}
		else
		{
			if( comando_motor.time_ramp1 < comando_motor.time_to_end )
			{
				comando_motor.time_to_end--;
				if(comando_motor.time_to_end == 0)
				{
					comando_motor.pos_destino_f = comando_motor.pos_final;
					comando_motor.pos_destino = comando_motor.pos_final;
					task_scheduler.iniciado[LS_TS_ESTADO] = 2;
				}
				else
				{
					comando_motor.pos_destino_f = comando_motor.pos_destino_f + comando_motor.step;
					if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
					else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
				}
			}
			else
			{
				comando_motor.time_ramp1--;
				comando_motor.time_to_end--;
				if(comando_motor.time_to_end == 0)
				{
					comando_motor.pos_destino_f = comando_motor.pos_final;
					comando_motor.pos_destino = comando_motor.pos_final;
					task_scheduler.iniciado[LS_TS_ESTADO] = 2;
				}
				else
				{
					comando_motor.pos_destino_f = comando_motor.pos_destino_f + (comando_motor.step_ramp1 * comando_motor.time_ramp1);
					if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
					else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
				}
			}
		}
	}
	else
	{
		comando_motor.time_to_end = 0;
		comando_motor.pos_destino_f = comando_motor.pos_final;
		comando_motor.pos_destino = comando_motor.pos_final;
		task_scheduler.iniciado[LS_TS_ESTADO] = 2;
	}
}

void comando_MS()
{
	float f_now;

	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0)
	{
		if (task_scheduler.comando[LS_TS_ESTADO] != 32)
		{
			f_now = comando_motor.pos_now;
			comando_motor.comando = 3;
		}
		else
		{
			f_now = comando_motor.velocidad;
			comando_motor.comando = 32;
		}

		if (task_scheduler.comando[LS_TS_ESTADO] == 5)
		{
			task_scheduler.param_3[LS_TS_ESTADO] = task_scheduler.param_2[LS_TS_ESTADO];
			task_scheduler.param_2[LS_TS_ESTADO] = task_scheduler.param_3[LS_TS_ESTADO]/2;
		}

		task_scheduler.iniciado[LS_TS_ESTADO] = 1;
		
		if( limit_posicion == 1 )
		{
			if( max_posicion >= min_posicion )
			{
				if( task_scheduler.param_1[LS_TS_ESTADO] > max_posicion) task_scheduler.param_1[LS_TS_ESTADO] = max_posicion;
				else if( task_scheduler.param_1[LS_TS_ESTADO] < min_posicion) task_scheduler.param_1[LS_TS_ESTADO] = min_posicion;
			}
		}
		
		comando_motor.pos_final = task_scheduler.param_1[LS_TS_ESTADO];
		if(task_scheduler.param_3[LS_TS_ESTADO] == LONG_MIN && task_scheduler.param_2[LS_TS_ESTADO] > 0)
		{
			comando_motor.time_ramp1 = t_ramp_s;
			comando_motor.time_ramp2 = 0;
			comando_motor.time_to_end = task_scheduler.param_2[LS_TS_ESTADO];
		}
		else if(task_scheduler.param_3[LS_TS_ESTADO] > 0)
		{
			comando_motor.time_ramp1 = task_scheduler.param_2[LS_TS_ESTADO];
			comando_motor.time_ramp2 = 0;
			comando_motor.time_to_end = task_scheduler.param_3[LS_TS_ESTADO];
		}
		else
		{
			comando_motor.time_to_end = 0;
		}

		if (2*comando_motor.time_ramp1 > comando_motor.time_to_end  && comando_motor.time_to_end > 3)
		{
			comando_motor.time_ramp1 = comando_motor.time_to_end / 2;
			comando_motor.time_ramp2 = 0;
		}
		else if(comando_motor.time_to_end <= 3)
		{
			comando_motor.time_ramp1 = 0;
			comando_motor.time_ramp2 = 0;
			comando_motor.time_to_end = 0;
			comando_motor.pos_final = task_scheduler.param_1[LS_TS_ESTADO];
			comando_motor.pos_destino_f = comando_motor.pos_final;
			comando_motor.pos_destino = comando_motor.pos_final;
			task_scheduler.iniciado[LS_TS_ESTADO] = 2;
		}

		if (comando_motor.time_ramp1 < 0) comando_motor.time_ramp1 = 0;

		if(comando_motor.time_to_end > 0)
		{
			if (comando_motor.time_ramp1 == 0) comando_motor.step_ramp1 = 0;											//paso senoidal
			else comando_motor.step_ramp1 = 180/(float)comando_motor.time_ramp1;										//paso senoidal
			comando_motor.step_ramp2 = -180;

			comando_motor.step = (float)(comando_motor.pos_final-f_now)/(((float)comando_motor.time_ramp1/((float)comando_motor.time_to_end+1.0f))+(1-(float)comando_motor.time_ramp1*2/((float)comando_motor.time_to_end+1.0f)));
			comando_motor.step=comando_motor.step/((float)comando_motor.time_to_end+1.0f);

			if(comando_motor.time_ramp1 > 0)
			{
				comando_motor.time_to_end--;
				comando_motor.time_ramp2 = 1;

				comando_motor.step_ramp2 = comando_motor.step_ramp2+comando_motor.step_ramp1;
				comando_motor.pos_destino_f = f_now + ((cosf(comando_motor.step_ramp2*degree_to_radian)+1)/2)*comando_motor.step;
				if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
				else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
			}
			else
			{
				comando_motor.time_to_end--;
				comando_motor.pos_destino_f = f_now + comando_motor.step;
				if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
				else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
			}
		}

	}
	else if (task_scheduler.iniciado[LS_TS_ESTADO] == 1)
	{
		if(comando_motor.time_ramp2 < comando_motor.time_ramp1)
		{
			comando_motor.time_ramp2++;
			comando_motor.time_to_end--;
			comando_motor.step_ramp2 = comando_motor.step_ramp2+comando_motor.step_ramp1;
			comando_motor.pos_destino_f = comando_motor.pos_destino_f + ((cosf(comando_motor.step_ramp2*degree_to_radian)+1)/2)*comando_motor.step;
			if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
			else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
		}
		else if (comando_motor.time_ramp2 > comando_motor.time_ramp1)
		{
			comando_motor.time_ramp1--;
			comando_motor.time_to_end--;
			if(comando_motor.time_to_end == 0)
			{
				comando_motor.pos_destino_f = comando_motor.pos_final;
				comando_motor.pos_destino = comando_motor.pos_final;
				task_scheduler.iniciado[LS_TS_ESTADO] = 2;
			}
			else
			{
				comando_motor.step_ramp2 = comando_motor.step_ramp2+comando_motor.step_ramp1;
				comando_motor.pos_destino_f = comando_motor.pos_destino_f + ((cosf(comando_motor.step_ramp2*degree_to_radian)+1)/2)*comando_motor.step;
				if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
				else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
			}
		}
		else
		{
			if( comando_motor.time_ramp1 < comando_motor.time_to_end )
			{
				comando_motor.time_to_end--;
				if(comando_motor.time_to_end == 0)
				{
					comando_motor.pos_destino_f = comando_motor.pos_final;
					comando_motor.pos_destino = comando_motor.pos_final;
					task_scheduler.iniciado[LS_TS_ESTADO] = 2;
				}
				else
				{
					comando_motor.pos_destino_f = comando_motor.pos_destino_f + comando_motor.step;
					if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
					else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
				}
			}
			else
			{
				comando_motor.time_ramp1--;
				comando_motor.time_to_end--;
				if(comando_motor.time_to_end == 0)
				{
					comando_motor.pos_destino_f = comando_motor.pos_final;
					comando_motor.pos_destino = comando_motor.pos_final;
					task_scheduler.iniciado[LS_TS_ESTADO] = 2;
				}
				else
				{
					comando_motor.step_ramp2 = comando_motor.step_ramp2+comando_motor.step_ramp1;
					comando_motor.pos_destino_f = comando_motor.pos_destino_f + ((cosf(comando_motor.step_ramp2*degree_to_radian)+1)/2)*comando_motor.step;
					if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
					else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
				}
			}
		}
	}
	else
	{
		comando_motor.time_to_end = 0;
		comando_motor.pos_destino_f = comando_motor.pos_final;
		comando_motor.pos_destino = comando_motor.pos_final;
		task_scheduler.iniciado[LS_TS_ESTADO] = 2;
	}
}

void comando_MA()
{
	float f_now, temp_f;

	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0)
	{
		f_now = comando_motor.pos_now;
		comando_motor.pos_ini = f_now;
		comando_motor.comando = 5;

		task_scheduler.iniciado[LS_TS_ESTADO] = 1;
		
		if( limit_posicion == 1 )
		{
			if( max_posicion >= min_posicion )
			{
				if( task_scheduler.param_1[LS_TS_ESTADO] > max_posicion) task_scheduler.param_1[LS_TS_ESTADO] = max_posicion;
				else if( task_scheduler.param_1[LS_TS_ESTADO] < min_posicion) task_scheduler.param_1[LS_TS_ESTADO] = min_posicion;
			}
		}
		
		comando_motor.pos_final = task_scheduler.param_1[LS_TS_ESTADO];
		if(task_scheduler.param_3[LS_TS_ESTADO] == LONG_MIN && task_scheduler.param_2[LS_TS_ESTADO] > 0)
		{
			comando_motor.step_ramp1 = a_ramp_t/1000.0f;
			comando_motor.time_to_end = task_scheduler.param_2[LS_TS_ESTADO];
		}
		else if(task_scheduler.param_3[LS_TS_ESTADO] > 0)
		{
			comando_motor.step_ramp1 = (float)task_scheduler.param_2[LS_TS_ESTADO]/1000.0f;
			comando_motor.time_to_end = task_scheduler.param_3[LS_TS_ESTADO];
		}
		else
		{
			comando_motor.time_to_end = 0;
		}

		if(comando_motor.pos_final < f_now && comando_motor.step_ramp1 > 0) comando_motor.step_ramp1 = -comando_motor.step_ramp1;
		else if (comando_motor.pos_final > f_now && comando_motor.step_ramp1 < 0) comando_motor.step_ramp1 = -comando_motor.step_ramp1;

		temp_f = (comando_motor.time_to_end*comando_motor.time_to_end-4*(comando_motor.pos_final-f_now)/comando_motor.step_ramp1);			//a=espacio*2/t^2 *2 = esp/t^2*4
		if(temp_f >= 0) temp_f=(comando_motor.time_to_end-sqrtf(temp_f))/2;
		else temp_f = 0;

		comando_motor.time_ramp1 = temp_f;
		comando_motor.time_ramp2 = 0;
		
		temp_f=(comando_motor.pos_final-f_now)/((comando_motor.time_ramp1/comando_motor.time_to_end)+(1-comando_motor.time_ramp1*2/comando_motor.time_to_end));
		comando_motor.step=temp_f/comando_motor.time_to_end;
		
		
		/**********************************************************************************************/
		
		if(comando_motor.time_to_end > 0)
		{
			comando_motor.time_to_end--;
			if(comando_motor.time_ramp1 == 0)
			{
				comando_motor.pos_destino_f = f_now + comando_motor.step;
				comando_motor.time_ramp2 = 0;
			}
			else
			{
				comando_motor.pos_destino_f = f_now + comando_motor.step_ramp1 / 2;									//espacio = 0.5*a*t^2
				comando_motor.time_ramp2 = 1;
			}
			
			if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
			else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
			task_scheduler.iniciado[LS_TS_ESTADO] = 1;
		}

	}
	else if (task_scheduler.iniciado[LS_TS_ESTADO] == 1)
	{
		if(comando_motor.time_ramp2 < comando_motor.time_ramp1)
		{
			comando_motor.time_ramp2++;
			comando_motor.time_to_end--;
			
			comando_motor.pos_destino = comando_motor.pos_ini + ((comando_motor.time_ramp2 * comando_motor.time_ramp2) * comando_motor.step_ramp1 / 2);		//espacio = 0.5*a*t^2
			
			if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
			else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
		}
		else if (comando_motor.time_ramp2 > comando_motor.time_ramp1)
		{
			comando_motor.time_ramp1++;
			comando_motor.time_to_end--;
			
			if(comando_motor.time_to_end == 0)
			{
				comando_motor.pos_destino_f = comando_motor.pos_final;
				comando_motor.pos_destino = comando_motor.pos_final;
				task_scheduler.iniciado[LS_TS_ESTADO] = 2;
			}
			else
			{
				comando_motor.pos_destino = (comando_motor.time_ramp1 * comando_motor.time_ramp1) * comando_motor.step_ramp1 / 2;		//espacio = 0.5*a*t^2
			}
			
			if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
			else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
		}
		else
		{
			if (comando_motor.time_ramp1 < comando_motor.time_to_end )
			{
				comando_motor.time_to_end--;
				if(comando_motor.time_to_end == 0)
				{
					comando_motor.pos_destino_f = comando_motor.pos_final;
					comando_motor.pos_destino = comando_motor.pos_final;
					task_scheduler.iniciado[LS_TS_ESTADO] = 2;
				}
				else
				{
					comando_motor.pos_destino_f = comando_motor.pos_destino_f + comando_motor.step;
					
					if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
					else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
				}
			}
			else
			{
				comando_motor.time_ramp1 = 1;
				comando_motor.time_to_end--;
				if(comando_motor.time_to_end == 0)
				{
					comando_motor.pos_destino_f = comando_motor.pos_final;
					comando_motor.pos_destino = comando_motor.pos_final;
					task_scheduler.iniciado[LS_TS_ESTADO] = 2;
				}
				else
				{
					comando_motor.pos_destino_f = comando_motor.step_ramp1 / 2;									//espacio = 0.5*a*t^2
				}
			}
		}
	}
	else
	{
		comando_motor.time_to_end = 0;
		comando_motor.pos_destino_f = comando_motor.pos_final;
		comando_motor.pos_destino = comando_motor.pos_final;
		task_scheduler.iniciado[LS_TS_ESTADO] = 2;
	}
	
}

void comando_MH()
{
	float t, t_cuadrado, t_cubo;
	float f_now, f_temp;

	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0)
	{
		if (task_scheduler.comando[LS_TS_ESTADO] != 33)
		{
			f_now = comando_motor.pos_now;
			f_temp = comando_motor.velocidad_int;
			comando_motor.comando = 4;
		}
		else
		{
			f_now = comando_motor.velocidad;
			f_temp = comando_motor.aceleracion_int;
			comando_motor.comando = 33;
		}

		task_scheduler.iniciado[LS_TS_ESTADO] = 1;
		
		if(task_scheduler.param_2[LS_TS_ESTADO] == LONG_MIN && task_scheduler.param_1[LS_TS_ESTADO] > 0)
		{
			if( limit_posicion == 1 )
			{
				if( max_posicion >= min_posicion )
				{
					if( task_scheduler.param_1[LS_TS_ESTADO] > max_posicion) task_scheduler.param_1[LS_TS_ESTADO] = max_posicion;
					else if( task_scheduler.param_1[LS_TS_ESTADO] < min_posicion) task_scheduler.param_1[LS_TS_ESTADO] = min_posicion;
				}
			}
			
			comando_motor.pos_final = task_scheduler.param_1[LS_TS_ESTADO];
		}
		else if(task_scheduler.param_3[LS_TS_ESTADO] == LONG_MIN && task_scheduler.param_2[LS_TS_ESTADO] > 0)
		{
			if( limit_posicion == 1 )
			{
				if( max_posicion >= min_posicion )
				{
					if( task_scheduler.param_1[LS_TS_ESTADO] > max_posicion) task_scheduler.param_1[LS_TS_ESTADO] = max_posicion;
					else if( task_scheduler.param_1[LS_TS_ESTADO] < min_posicion) task_scheduler.param_1[LS_TS_ESTADO] = min_posicion;
				}
			}
			
			comando_motor.step_ramp1 = f_temp;
			comando_motor.step_ramp2 = 0;
			comando_motor.pos_final = task_scheduler.param_1[LS_TS_ESTADO];
			comando_motor.time_to_end = task_scheduler.param_2[LS_TS_ESTADO];
			comando_motor.pos_ini = f_now;
		}
		else if(task_scheduler.param_4[LS_TS_ESTADO] == LONG_MIN && task_scheduler.param_3[LS_TS_ESTADO] > 0)
		{
			if( limit_posicion == 1 )
			{
				if( max_posicion >= min_posicion )
				{
					if( task_scheduler.param_2[LS_TS_ESTADO] > max_posicion) task_scheduler.param_2[LS_TS_ESTADO] = max_posicion;
					else if( task_scheduler.param_2[LS_TS_ESTADO] < min_posicion) task_scheduler.param_2[LS_TS_ESTADO] = min_posicion;
				}
			}
			
			comando_motor.step_ramp1 = f_temp;
			comando_motor.step_ramp2 = 0;
			comando_motor.pos_final = task_scheduler.param_2[LS_TS_ESTADO];
			comando_motor.time_to_end = task_scheduler.param_3[LS_TS_ESTADO];
			comando_motor.pos_ini = task_scheduler.param_1[LS_TS_ESTADO];
		}
		else if(task_scheduler.param_5[LS_TS_ESTADO] > 0)
		{
			if( limit_posicion == 1 )
			{
				if( max_posicion >= min_posicion )
				{
					if( task_scheduler.param_3[LS_TS_ESTADO] > max_posicion) task_scheduler.param_3[LS_TS_ESTADO] = max_posicion;
					else if( task_scheduler.param_3[LS_TS_ESTADO] < min_posicion) task_scheduler.param_3[LS_TS_ESTADO] = min_posicion;
				}
			}
			
			comando_motor.step_ramp1 = task_scheduler.param_2[LS_TS_ESTADO];
			comando_motor.step_ramp2 = task_scheduler.param_4[LS_TS_ESTADO];
			comando_motor.pos_final = task_scheduler.param_3[LS_TS_ESTADO];
			comando_motor.time_to_end = task_scheduler.param_5[LS_TS_ESTADO];
			comando_motor.pos_ini = task_scheduler.param_1[LS_TS_ESTADO];
		}
		else
		{
			comando_motor.time_to_end = 0;
		}

		if(comando_motor.time_to_end > 0)
		{
			t = comando_motor.time_to_end + 1;
			f_temp = comando_motor.step_ramp1*t/1000;
			comando_motor.step_ramp1 = f_temp;
			f_temp = comando_motor.step_ramp2*t/1000;
			comando_motor.step_ramp2 = f_temp;
			
			comando_motor.time_ramp1 = 1;
			comando_motor.time_ramp2 = t;									//Ajuste por algoritmo
			comando_motor.time_to_end--;

			t = 1.0f / ((float)comando_motor.time_ramp2 - 1.0f);
			t_cuadrado = t*t;
			t_cubo = t_cuadrado*t;
			comando_motor.pos_destino_f = (2.0f * t_cubo - 3.0f * t_cuadrado + 1.0f) * comando_motor.pos_ini
				+ (t_cubo - 2.0f * t_cuadrado + t) * comando_motor.step_ramp1
				+ (-2.0f * t_cubo + 3.0f * t_cuadrado) * (float)comando_motor.pos_final
				+ (t_cubo - t_cuadrado) * comando_motor.step_ramp2;
			if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
			else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
			
			if( limit_posicion == 1 )
			{
				if( max_posicion >= min_posicion )
				{
					if( comando_motor.pos_destino > max_posicion) comando_motor.pos_destino = max_posicion;
					else if( comando_motor.pos_destino < min_posicion) comando_motor.pos_destino = min_posicion;
				}
			}
		}

	}
	else if (task_scheduler.iniciado[LS_TS_ESTADO] == 1)
	{
		comando_motor.time_to_end--;
		if(comando_motor.time_to_end > 0)
		{
			comando_motor.time_ramp1++;

			t = (float)comando_motor.time_ramp1 / ((float)comando_motor.time_ramp2 - 1.0f);
			t_cuadrado = t*t;
			t_cubo = t_cuadrado*t;
			comando_motor.pos_destino_f = (2.0f * t_cubo - 3.0f * t_cuadrado + 1.0f) * comando_motor.pos_ini
				+ (t_cubo - 2.0f * t_cuadrado + t) * comando_motor.step_ramp1
				+ (-2.0f * t_cubo + 3.0f * t_cuadrado) * (float)comando_motor.pos_final
				+ (t_cubo - t_cuadrado) * comando_motor.step_ramp2;
			if (comando_motor.pos_destino_f >= 0) comando_motor.pos_destino = comando_motor.pos_destino_f + 0.5f;
			else comando_motor.pos_destino = comando_motor.pos_destino_f - 0.5f;
			
			if( limit_posicion == 1 )
			{
				if( max_posicion >= min_posicion )
				{
					if( comando_motor.pos_destino > max_posicion) comando_motor.pos_destino = max_posicion;
					else if( comando_motor.pos_destino < min_posicion) comando_motor.pos_destino = min_posicion;
				}
			}
		}
		else
		{
			comando_motor.pos_destino_f = comando_motor.pos_final;
			comando_motor.pos_destino = comando_motor.pos_final;
			task_scheduler.iniciado[LS_TS_ESTADO] = 2;
		}
	}
	else
	{
		comando_motor.time_to_end = 0;
		comando_motor.pos_destino_f = comando_motor.pos_final;
		comando_motor.pos_destino = comando_motor.pos_final;
		task_scheduler.iniciado[LS_TS_ESTADO] = 2;
	}
}
void comando_MP()
{
	long TIM_autoreload = TIM1->ARR;
	
	if( task_scheduler.param_1[LS_TS_ESTADO] > TIM_autoreload ) comando_motor.output_LS = TIM_autoreload;
	else if( task_scheduler.param_1[LS_TS_ESTADO] < -TIM_autoreload ) comando_motor.output_LS = -TIM_autoreload;
	else comando_motor.output_LS = task_scheduler.param_1[LS_TS_ESTADO];
	
	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0) task_scheduler.iniciado[LS_TS_ESTADO] = 1;
	
	if (task_scheduler.param_2[LS_TS_ESTADO] == LONG_MIN) task_scheduler.iniciado[LS_TS_ESTADO] = 2;
	else
	{
		if (task_scheduler.param_2[LS_TS_ESTADO] > 0) task_scheduler.param_2[LS_TS_ESTADO]--;
		else task_scheduler.param_2[LS_TS_ESTADO] = 0;

		if (task_scheduler.param_2[LS_TS_ESTADO] == 0)
		{
			task_scheduler.iniciado[LS_TS_ESTADO] = 2;
			comando_motor.output_LS = 0;
		}
	}
}
void comando_MW()
{
	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0) task_scheduler.iniciado[LS_TS_ESTADO] = 1;
	
	if (task_scheduler.param_1[LS_TS_ESTADO] > 0) task_scheduler.param_1[LS_TS_ESTADO]--;
	else task_scheduler.param_1[LS_TS_ESTADO] = 0;
	
	if (task_scheduler.param_1[LS_TS_ESTADO] == 0) task_scheduler.iniciado[LS_TS_ESTADO] = 2;
}
void comando_MF()
{
	TIM1->CCR2 = 0;
	TIM1->CCR1 = 0;
	comando_motor.output_LS = 0;
	task_scheduler.iniciado[LS_TS_ESTADO] = 2;
}
