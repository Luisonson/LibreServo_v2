/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LS_flash.c
  * @brief          : flash functions
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

extern uint16_t version_LS;
extern volatile uint16_t ID_SERVO;			//Inicializado en main.c

struct LS_task_scheduler {uint8_t comando[LS_TASK_SCHEDULER_SIZE];long param_1[LS_TASK_SCHEDULER_SIZE];long param_2[LS_TASK_SCHEDULER_SIZE];long param_3[LS_TASK_SCHEDULER_SIZE];long param_4[LS_TASK_SCHEDULER_SIZE];long param_5[LS_TASK_SCHEDULER_SIZE];long time_comando[LS_TASK_SCHEDULER_SIZE];uint8_t iniciado[LS_TASK_SCHEDULER_SIZE];};
extern volatile struct LS_task_scheduler task_scheduler;			//Inicializado en LS_funciones.c
extern volatile uint8_t LS_TS_ESTADO;		//Inicializado en LS_funciones.c

extern uint16_t q_corriente, q_temp_ext, q_temp_int, q_volts, q_encoder;
extern uint16_t varianza_corriente, varianza_temp_ext, varianza_temp_int, varianza_volts, varianza_encoder;
extern uint32_t vel_serie;					//Inicializado en main.c
extern uint16_t corte_temp_ext, corte_temp_int, corte_volts_alto, corte_volts_bajo, corte_corriente;		//Inicializado en main.c

volatile uint16_t t_ramp_t = 200, t_ramp_s = 200, a_ramp_t = 500;

extern volatile uint16_t uso_crc; 			//Iniciliazado en main.c 0=False, 1=True, 2=Both
extern volatile uint16_t CRC_START_CCITT;	//Iniciliazado en LS_funciones.c
extern volatile uint16_t CRC_POLY_CCITT;	//Inicializado en LS_funciones.c

volatile uint16_t K_P_LS, K_D_LS, K_I_LS;
extern volatile float K_P_LS_f, K_D_LS_f, K_I_LS_f;		//Inicializado en LS_PID.c
volatile uint16_t K_P_M, K_D_M, K_I_M;
extern volatile float K_P_M_f, K_D_M_f, K_I_M_f;		//Inicializado en LS_PID.c

volatile int offset_corriente;
volatile float offset_temp_int;
volatile float offset_temp_ext;

volatile uint16_t envio_si_leo;					//No enviar si se reciben datos (0=False, 1=True, 2=True envío actual)

volatile uint8_t saludo_inicial = TRUE;			//Saludo Inicial (0=False, 1=True)

volatile long min_posicion, max_posicion;		//Min y máx posición (0-4294967294 -> -2147483647 - 2147483647)
volatile uint8_t limit_posicion;				//limites posición (0=False, 1=True)
volatile uint16_t deadband;
volatile uint8_t direccion_motor;				//Dirección motor (0/1)

extern volatile int correccion_enc[257];		//Inicializado en main.c

uint16_t write_32_flash(uint16_t VirtAddress,uint32_t variable)
{
	uint16_t temp_16;
	uint16_t resultado;
	temp_16 = (uint16_t)(variable>>16);
	resultado = EE_WriteVariable(VirtAddress,temp_16);

	temp_16 = (uint16_t)(variable);
	resultado += EE_WriteVariable(VirtAddress+1,temp_16);
	return(resultado);
}
uint16_t read_32_flash(uint16_t VirtAddress,uint32_t* variable)
{
	uint16_t temp_16;
	uint16_t resultado;
	resultado = EE_ReadVariable(VirtAddress,&temp_16);
	*variable = temp_16;
	*variable = *variable<<16;
	resultado += EE_ReadVariable(VirtAddress+1,&temp_16);
	*variable = *variable + temp_16;
	return(resultado);
}
uint16_t write_init_flash()
{
	uint16_t resultado, i_correccion, d_correccion;

	resultado = EE_WriteVariable(0x0002,1);		//ID_SERVO

	resultado += EE_WriteVariable(0x0003,1200);	//Incertidumbre proceso corriente (x1000)
	resultado += EE_WriteVariable(0x0004,1200);	//Incertidumbre proceso temp_ext (x1000)
	resultado += EE_WriteVariable(0x0005,1200);	//Incertidumbre proceso temp_int (x1000)
	resultado += EE_WriteVariable(0x0006,1200);	//Incertidumbre proceso voltaje (x1000)
	resultado += EE_WriteVariable(0x0007,1200);	//Incertidumbre proceso encoder (x1000)
	resultado += EE_WriteVariable(0x0008,1000);	//Varianza corriente (x1000)
	resultado += EE_WriteVariable(0x0009,2500);	//Varianza proceso temp_ext (x1000)
	resultado += EE_WriteVariable(0x000A,150);	//Varianza proceso temp_int (x1000)
	resultado += EE_WriteVariable(0x000B,150);	//Varianza proceso voltaje (x1000)
	resultado += EE_WriteVariable(0x000C,5000);	//Varianza proceso encoder (x1000)

	resultado += write_32_flash(0x000D,115200);	//velocidad serie 0x000D y 0x000E

	resultado += EE_WriteVariable(0x000F,800);	//corte_temp_ext (x10)
	resultado += EE_WriteVariable(0x0010,800);	//corte_temp_int (x10)
	resultado += EE_WriteVariable(0x0011,1600);	//corte_volt_alto (x100)
	resultado += EE_WriteVariable(0x0012,450);	//corte_volt_bajo (x100)
	resultado += EE_WriteVariable(0x0013,7000);	//corte_corriente (x1000)

	resultado += EE_WriteVariable(0x0014,2);		//Uso CRC (0=False, 1=True, 2=Both)
	resultado += EE_WriteVariable(0x0015,0x1D0F);	//CRC_START_CCITT_1D0F	0x1D0F
	resultado += EE_WriteVariable(0x0016,0x1021);	//CRC_POLY_CCITT		0x1021

	resultado += EE_WriteVariable(0x0017,200);	//K_P_LS		2.00
	resultado += EE_WriteVariable(0x0018,1000);	//K_D_LS		10.00
	resultado += EE_WriteVariable(0x0019,170);	//K_I_LS		0.017
	resultado += EE_WriteVariable(0x001A,200);	//K_P_M			2.00
	resultado += EE_WriteVariable(0x001B,1000);	//K_D_M			10.00
	resultado += EE_WriteVariable(0x001C,170);	//K_I_M			0.017

	resultado += EE_WriteVariable(0x001D,111);	//offset_corriente	111		(0-222 -> -1000 - 1000)
	resultado += EE_WriteVariable(0x001E,150);	//offset_temp_int	150		(0-300 -> -15.0 - 15.0)
	resultado += EE_WriteVariable(0x001F,150);	//offset_temp_ext	150		(0-300 -> -15.0 - 15.0)

	resultado += EE_WriteVariable(0x0020,1);	//No enviar si se reciben datos (0=False, 1=True, 2=True envío actual)

	resultado += EE_WriteVariable(0x0021,1);	//Saludo Inicial (0=False, 1=True)
	
	resultado += write_32_flash(0x0022,-LONG_MIN);		//max_posicion 0x0022 y 0x0023
	resultado += write_32_flash(0x0024,-LONG_MIN);		//min_posicion 0x0024 y 0x0025
	resultado += EE_WriteVariable(0x0026,0);			//limites posición (0=False, 1=True)
	
	resultado += EE_WriteVariable(0x0027,200);			//tiempo_rampa_trapezoidal 200ms
	resultado += EE_WriteVariable(0x0028,200);			//tiempo_rampa_senoidal 200ms
	resultado += EE_WriteVariable(0x0029,100);			//aceleración_rampa_trapezoidal 100

	resultado += EE_WriteVariable(0x002A,5);			//deadband 5

	resultado += EE_WriteVariable(0x002B,0);			//direccion_motor 0
	
	for(d_correccion=0x1000,i_correccion=0;i_correccion<256;d_correccion++,i_correccion++)
	{
		resultado += EE_WriteVariable(d_correccion,0);
	}

	return(resultado);
}
void reset_flash()
{
	FLASH_ErasePage(PAGE0_BASE_ADDRESS);
	FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
	FLASH_ErasePage(PAGE1_BASE_ADDRESS);
	EE_WriteVariable(0x0001,version_LS);
	/*Escribir todas las variables*/
	write_init_flash();
	read_init_flash();
	init_crcccitt_tab();			//Inicializa la tabla de CRC. Por defecto CRC-16/AUG-CCITT
	get_kalman_gains();
	/*uint16_t i_correccion, d_correccion;
	read_init_flash();

	if (ID_SERVO != 1) EE_WriteVariable(0x0002,1);					//ID_SERVO

	if (q_corriente != 1200) EE_WriteVariable(0x0003,1200);			//Incertidumbre proceso corriente (x1000)
	if (q_temp_ext != 1200) EE_WriteVariable(0x0004,1200);			//Incertidumbre proceso temp_ext (x1000)
	if (q_temp_int != 1200) EE_WriteVariable(0x0005,1200);			//Incertidumbre proceso temp_int (x1000)
	if (q_volts != 1200) EE_WriteVariable(0x0006,1200);				//Incertidumbre proceso voltaje (x1000)
	if (q_encoder != 1200) EE_WriteVariable(0x0007,1200);			//Incertidumbre proceso encoder (x1000)
	if (varianza_corriente != 1000) EE_WriteVariable(0x0008,1000);	//Varianza corriente (x1000)
	if (varianza_temp_ext != 2500) EE_WriteVariable(0x0009,2500);	//Varianza proceso temp_ext (x1000)
	if (varianza_temp_int != 150) EE_WriteVariable(0x000A,150);		//Varianza proceso temp_int (x1000)
	if (varianza_volts != 150) EE_WriteVariable(0x000B,150);		//Varianza proceso voltaje (x1000)
	if (varianza_encoder != 5000) EE_WriteVariable(0x000C,5000);	//Varianza proceso encoder (x1000)

	if (vel_serie != 115200) write_32_flash(0x000D,115200);			//velocidad serie 0x000D y 0x000E

	if (corte_temp_ext != 800) EE_WriteVariable(0x000F,800);		//corte_temp_ext (x10)
	if (corte_temp_int != 800) EE_WriteVariable(0x0010,800);		//corte_temp_int (x10)
	if (corte_volts_alto != 1600) EE_WriteVariable(0x0011,1600);	//corte_volt_alto (x100)
	if (corte_volts_bajo != 450) EE_WriteVariable(0x0012,450);	//corte_volt_bajo (x100)
	if (corte_corriente != 7000) EE_WriteVariable(0x0013,7000);		//corte_corriente (x1000)

	if (uso_crc != 2) EE_WriteVariable(0x0014,2);					//Uso CRC (0=False, 1=True, 2=Both)
	if (CRC_START_CCITT != 0x1D0F) EE_WriteVariable(0x0015,0x1D0F);	//CRC_START_CCITT_1D0F	0x1D0F
	if (CRC_POLY_CCITT != 0x1021) EE_WriteVariable(0x0016,0x1021);	//CRC_POLY_CCITT		0x1021

	if (K_P_LS != 200) EE_WriteVariable(0x0017,200);				//K_P_LS		2.00
	if (K_D_LS != 1000) EE_WriteVariable(0x0018,1000);				//K_D_LS		10.00
	if (K_I_LS != 170) EE_WriteVariable(0x0019,170);				//K_I_LS		0.017

	if (K_P_M != 200) EE_WriteVariable(0x001A,200);					//K_P_M			2.00
	if (K_D_M != 1000) EE_WriteVariable(0x001B,1000);				//K_D_M			10.00
	if (K_I_M != 170) EE_WriteVariable(0x001C,170);					//K_I_M			0.017

	if (offset_corriente != 0) EE_WriteVariable(0x001D,111);		//offset_corriente	111		(0-222 -> -1000 - 1000)
	if (offset_temp_int != 0) EE_WriteVariable(0x001E,150);			//offset_temp_int	150		(0-300 -> -15.0 - 15.0)
	if (offset_temp_ext != 0) EE_WriteVariable(0x001F,150);			//offset_temp_ext	150		(0-300 -> -15.0 - 15.0)

	if (envio_si_leo != 1) EE_WriteVariable(0x0020,1);				//No enviar si se reciben datos (0=False, 1=True, 2=True envío actual)

	if (saludo_inicial != 1) EE_WriteVariable(0x0021,1);			//Saludo Inicial (0=False, 1=True)

	if (max_posicion != 0) write_32_flash(0x0022,-LONG_MIN);		//max_posicion 0x0022 y 0x0023		(0-4294967294 -> -2147483647 - 2147483647)
	if (min_posicion != 0) write_32_flash(0x0024,-LONG_MIN);		//min_posicion 0x0024 y 0x0025		(0-4294967294 -> -2147483647 - 2147483647)
	if (limit_posicion != 0) EE_WriteVariable(0x0026,0);			//limites posición (0=False, 1=True)

	if (t_ramp_t != 200) EE_WriteVariable(0x0027,200);				//tiempo_rampa_trapezoidal 200ms
	if (t_ramp_s != 200) EE_WriteVariable(0x0028,200);				//tiempo_rampa_senoidal 200ms
	if (a_ramp_t != 100) EE_WriteVariable(0x0029,100);				//aceleración_rampa_trapezoidal 100

	if(deadband != 5) EE_WriteVariable(0x002A,5);					//deadband 5

	if(direccion_motor != 0) EE_WriteVariable(0x002B,0);			//direccion_motor 0
	
	for(d_correccion=0x1000,i_correccion=0;i_correccion<256;d_correccion++,i_correccion++)
	{
		if(correccion_enc[i_correccion] != 0) EE_WriteVariable(d_correccion,0);
	}

	read_init_flash();*/
}
void reset_value_flash()
{
	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0)
	{
		switch (task_scheduler.param_1[LS_TS_ESTADO])
		{
			case 100:
				EE_WriteVariable(0x0002,1);					//ID_SERVO
				ID_SERVO = 1;
			break;
			case 101:
				EE_WriteVariable(0x0003,1200);				//Incertidumbre proceso corriente (x1000)
				q_corriente = 1200;
			break;
			case 102:
				EE_WriteVariable(0x0004,1200);				//Incertidumbre proceso temp_ext (x1000)
				q_temp_ext = 1200;
			break;
			case 103:
				EE_WriteVariable(0x0005,1200);				//Incertidumbre proceso temp_int (x1000)
				q_temp_int = 1200;
			break;
			case 104:
				EE_WriteVariable(0x0006,1200);				//Incertidumbre proceso voltaje (x1000)
				q_volts = 1200;
			break;
			case 105:
				EE_WriteVariable(0x0007,1200);				//Incertidumbre proceso encoder (x1000)
				q_encoder = 1200;
			break;
			case 106:
				EE_WriteVariable(0x0008,1000);				//Varianza corriente (x1000)
				varianza_corriente = 1000;
			break;
			case 107:
				EE_WriteVariable(0x0009,2500);				//Varianza proceso temp_ext (x1000)
				varianza_temp_ext = 2500;
			break;
			case 108:
				EE_WriteVariable(0x000A,150);				//Varianza proceso temp_int (x1000)
				varianza_temp_int =150;
			break;
			case 109:
				EE_WriteVariable(0x000B,150);				//Varianza proceso voltaje (x1000)
				varianza_volts = 150;
			break;
			case 110:
				EE_WriteVariable(0x000C,5000);				//Varianza proceso encoder (x1000)
				varianza_encoder = 5000;
			break;
			case 111:
				write_32_flash(0x000D,115200);				//velocidad serie 0x000D y 0x000E
				vel_serie = 115200;
			break;
			case 112:
				EE_WriteVariable(0x000F,800);				//corte_temp_ext (x10)
				corte_temp_ext = 800;
			break;
			case 113:
				EE_WriteVariable(0x0010,800);				//corte_temp_int (x10)
				corte_temp_int = 800;
			break;
			case 114:
				EE_WriteVariable(0x0011,1600);				//corte_volt_alto (x100)
				corte_volts_alto = 1600;
			break;
			case 115:
				EE_WriteVariable(0x0012,450);				//corte_volt_bajo (x100)
				corte_volts_bajo = 450;
			break;
			case 116:
				EE_WriteVariable(0x0013,7000);				//corte_corriente (x1000)
				corte_corriente = 7000;
			break;
			case 117:
				EE_WriteVariable(0x0014,2);					//Uso CRC (0=False, 1=True, 2=Both)
				uso_crc = 2;
			break;
			case 118:
				EE_WriteVariable(0x0015,0x1D0F);			//CRC_START_CCITT_1D0F	0x1D0F
				CRC_START_CCITT = 0x1D0F;
			break;
			case 119:
				EE_WriteVariable(0x0016,0x1021);			//CRC_POLY_CCITT		0x1021
				CRC_POLY_CCITT = 0x1021;
			break;
			case 120:
				EE_WriteVariable(0x0017,200);				//K_P_LS		2.00
				K_P_LS = 200;
			break;
			case 121:
				EE_WriteVariable(0x0018,1000);				//K_D_LS		10.00
				K_D_LS = 1000;
			break;
			case 122:
				EE_WriteVariable(0x0019,170);				//K_I_LS		0.017
				K_I_LS = 170;
			break;
			case 123:
				EE_WriteVariable(0x001A,200);				//K_P_M			2.00
				K_P_M = 200;
			break;
			case 124:
				EE_WriteVariable(0x001B,1000);				//K_D_M			10.00
				K_D_M = 1000;
			break;
			case 125:
				EE_WriteVariable(0x001C,170);				//K_I_M			0.017
				K_I_M = 170;
			break;
			case 126:
				EE_WriteVariable(0x001D,111);				//offset_corriente	111		(0-222 -> -1000 - 1000)
				offset_corriente = 0;
			break;
			case 127:
				EE_WriteVariable(0x001E,150);				//offset_temp_int	150		(0-300 -> -15.0 - 15.0)
				offset_temp_int = 0;
			break;
			case 128:
				EE_WriteVariable(0x001F,150);				//offset_temp_ext	150		(0-300 -> -15.0 - 15.0)
				offset_temp_ext = 0;
			break;
			case 129:
				EE_WriteVariable(0x0020,1);					//No enviar si se reciben datos (0=False, 1=True, 2=True envío actual)
				envio_si_leo = 1;
			break;
			case 130:
				EE_WriteVariable(0x0021,1);					//Saludo Inicial (0=False, 1=True)
				saludo_inicial = 1;
			break;
			case 131:
				write_32_flash(0x0022,-LONG_MIN);			//max_posicion 0x0022 y 0x0023		(0-4294967294 -> -2147483647 - 2147483647L)
				max_posicion = 0;
			break;
			case 132:
				write_32_flash(0x0024,-LONG_MIN);			//min_posicion 0x0024 y 0x0025		(0-4294967294 -> -2147483647 - 2147483647L)
				min_posicion = 0;
			break;
			case 133:
				EE_WriteVariable(0x0026,0);					//limites posición (0=False, 1=True)
				limit_posicion = 0;
			break;
			case 134:
				EE_WriteVariable(0x0027,200);				//tiempo_rampa_trapezoidal 200ms
				t_ramp_t = 200;
			break;                                              
			case 135:                                           
				EE_WriteVariable(0x0028,200);				//tiempo_rampa_senoidal 200ms
				t_ramp_s = 200;
			break;
			case 136:
				EE_WriteVariable(0x0029,100);				//aceleración_rampa_trapezoidal 100
				a_ramp_t = 100;
			break;
			case 137:
				EE_WriteVariable(0x002A,5);					//deadband 5
				deadband = 5;
			break;
			case 138:
				EE_WriteVariable(0x002B,0);					//direccion_motor 0
				direccion_motor = 0;
			default:
			break;
		}
		/*if(task_scheduler.param_1[LS_TS_ESTADO]>=0x1000 && task_scheduler.param_1[LS_TS_ESTADO]<=0x1100)
		{
			EE_WriteVariable(task_scheduler.param_1[LS_TS_ESTADO],0);
			correccion_enc[task_scheduler.param_1[LS_TS_ESTADO]-0x1000] = 0;
		}*/
		//read_init_value_flash();
	}
	task_scheduler.iniciado[LS_TS_ESTADO] =2;	//No haría falta porque read_init_value_flash ya lo hace.
}
void add_new_values_flash()
{
	uint16_t value;
	uint32_t value_32;
	uint16_t i_correccion, d_correccion;

	if (EE_ReadVariable(0x0002,&value) != 0) EE_WriteVariable(0x0002,1);		//ID_SERVO

	if (EE_ReadVariable(0x0003,&value) != 0) EE_WriteVariable(0x0003,1200);	//Incertidumbre proceso corriente (x1000)
	if (EE_ReadVariable(0x0004,&value) != 0) EE_WriteVariable(0x0004,1200);	//Incertidumbre proceso temp_ext (x1000)
	if (EE_ReadVariable(0x0005,&value) != 0) EE_WriteVariable(0x0005,1200);	//Incertidumbre proceso temp_int (x1000)
	if (EE_ReadVariable(0x0006,&value) != 0) EE_WriteVariable(0x0006,1200);	//Incertidumbre proceso voltaje (x1000)
	if (EE_ReadVariable(0x0007,&value) != 0) EE_WriteVariable(0x0007,1200);	//Incertidumbre proceso encoder (x1000)
	if (EE_ReadVariable(0x0008,&value) != 0) EE_WriteVariable(0x0008,1000);	//Varianza corriente (x1000)
	if (EE_ReadVariable(0x0009,&value) != 0) EE_WriteVariable(0x0009,2500);	//Varianza proceso temp_ext (x1000)
	if (EE_ReadVariable(0x000A,&value) != 0) EE_WriteVariable(0x000A,150);	//Varianza proceso temp_int (x1000)
	if (EE_ReadVariable(0x000B,&value) != 0) EE_WriteVariable(0x000B,150);	//Varianza proceso voltaje (x1000)
	if (EE_ReadVariable(0x000C,&value) != 0) EE_WriteVariable(0x000C,5000);	//Varianza proceso encoder (x1000)

	if (read_32_flash(0x000D,&value_32) != 0) write_32_flash(0x000D,115200);	//velocidad serie 0x000D y 0x000E

	if (EE_ReadVariable(0x000F,&value) != 0) EE_WriteVariable(0x000F,800);		//corte_temp_ext (x10)
	if (EE_ReadVariable(0x0010,&value) != 0) EE_WriteVariable(0x0010,800);		//corte_temp_int (x10)
	if (EE_ReadVariable(0x0011,&value) != 0) EE_WriteVariable(0x0011,1600);		//corte_volt_alto (x100)
	if (EE_ReadVariable(0x0012,&value) != 0) EE_WriteVariable(0x0012,450);		//corte_volt_bajo (x100)
	if (EE_ReadVariable(0x0013,&value) != 0) EE_WriteVariable(0x0013,7000);		//corte_corriente (x1000)

	if (EE_ReadVariable(0x0014,&value) != 0) EE_WriteVariable(0x0014,2);		//Uso CRC (0=False, 1=True, 2=Both)
	if (EE_ReadVariable(0x0015,&value) != 0) EE_WriteVariable(0x0015,0x1D0F);	//CRC_START_CCITT_1D0F	0x1D0F
	if (EE_ReadVariable(0x0016,&value) != 0) EE_WriteVariable(0x0016,0x1021);	//CRC_POLY_CCITT		0x1021

	if (EE_ReadVariable(0x0017,&value) != 0) EE_WriteVariable(0x0017,200);		//K_P_LS		2.00
	if (EE_ReadVariable(0x0018,&value) != 0) EE_WriteVariable(0x0018,1000);		//K_D_LS		10.00
	if (EE_ReadVariable(0x0019,&value) != 0) EE_WriteVariable(0x0019,170);		//K_I_LS		0.017

	if (EE_ReadVariable(0x001A,&value) != 0) EE_WriteVariable(0x001A,200);		//K_P_M			2.00
	if (EE_ReadVariable(0x001B,&value) != 0) EE_WriteVariable(0x001B,1000);		//K_D_M			10.00
	if (EE_ReadVariable(0x001C,&value) != 0) EE_WriteVariable(0x001C,170);		//K_I_M			0.017

	if (EE_ReadVariable(0x001D,&value) != 0) EE_WriteVariable(0x001D,111);		//offset_corriente	111		(0-222 -> -1000 - 1000)
	if (EE_ReadVariable(0x001E,&value) != 0) EE_WriteVariable(0x001E,150);		//offset_temp_int	150		(0-300 -> -15.0 - 15.0)
	if (EE_ReadVariable(0x001F,&value) != 0) EE_WriteVariable(0x001F,150);		//offset_temp_ext	150		(0-300 -> -15.0 - 15.0)

	if (EE_ReadVariable(0x0020,&value) != 0) EE_WriteVariable(0x0020,1);		//No enviar si se reciben datos (0=False, 1=True, 2=True envío actual)

	if (EE_ReadVariable(0x0021,&value) != 0) EE_WriteVariable(0x0021,1);		//Saludo Inicial (0=False, 1=True)
	
	if (read_32_flash(0x0022,&value_32)) write_32_flash(0x0022,-LONG_MIN);		//max_posicion 0x0022 y 0x0023		(0-4294967294 -> -2147483647 - 2147483647L)
	if (read_32_flash(0x0024,&value_32)) write_32_flash(0x0024,-LONG_MIN);		//min_posicion 0x0024 y 0x0025		(0-4294967294 -> -2147483647 - 2147483647L)
	if (EE_ReadVariable(0x0026,&value)) EE_WriteVariable(0x0026,0);				//limites posición (0=False, 1=True)
		
	if (EE_ReadVariable(0x0027,&value) != 0) EE_WriteVariable(0x0027,200);		//tiempo_rampa_trapezoidal 200ms
	if (EE_ReadVariable(0x0028,&value) != 0) EE_WriteVariable(0x0028,200);		//tiempo_rampa_senoidal 200ms
	if (EE_ReadVariable(0x0029,&value) != 0) EE_WriteVariable(0x0029,100);		//aceleración_rampa_trapezoidal 100

	if (EE_ReadVariable(0x002A,&value) != 0) EE_WriteVariable(0x002A,5);		//deadband

	if (EE_ReadVariable(0x002B,&value) != 0) EE_WriteVariable(0x002B,0);		//direccion_motor 0
	
	for(d_correccion=0x1000,i_correccion=0;i_correccion<256;d_correccion++,i_correccion++)
	{
		if (EE_ReadVariable(d_correccion,&value) != 0) EE_WriteVariable(d_correccion,0);
	}
}
uint16_t read_init_flash()
{
	uint16_t resultado, temp_value;
	uint32_t temp_value_l;
	//uint16_t i_correccion, d_correccion;

	resultado = EE_ReadVariable(0x0002,&temp_value);
	ID_SERVO = temp_value;

	resultado += EE_ReadVariable(0x0003,&q_corriente);
	resultado += EE_ReadVariable(0x0004,&q_temp_ext);
	resultado += EE_ReadVariable(0x0005,&q_temp_int);
	resultado += EE_ReadVariable(0x0006,&q_volts);
	resultado += EE_ReadVariable(0x0007,&q_encoder);
	resultado += EE_ReadVariable(0x0008,&varianza_corriente);
	resultado += EE_ReadVariable(0x0009,&varianza_temp_ext);
	resultado += EE_ReadVariable(0x000A,&varianza_temp_int);
	resultado += EE_ReadVariable(0x000B,&varianza_volts);
	resultado += EE_ReadVariable(0x000C,&varianza_encoder);

	resultado += read_32_flash(0x000D,&vel_serie);

	resultado += EE_ReadVariable(0x000F,&corte_temp_ext);
	resultado += EE_ReadVariable(0x0010,&corte_temp_int);
	resultado += EE_ReadVariable(0x0011,&corte_volts_alto);
	resultado += EE_ReadVariable(0x0012,&corte_volts_bajo);
	resultado += EE_ReadVariable(0x0013,&corte_corriente);

	resultado += EE_ReadVariable(0x0014,&temp_value);
	uso_crc = temp_value;
	resultado += EE_ReadVariable(0x0015,&temp_value);
	CRC_START_CCITT = temp_value;
	resultado += EE_ReadVariable(0x0016,&temp_value);
	CRC_POLY_CCITT = temp_value;

	resultado += EE_ReadVariable(0x0017,&temp_value);
	K_P_LS = temp_value;
	K_P_LS_f = (float)K_P_LS/100;
	resultado += EE_ReadVariable(0x0018,&temp_value);
	K_D_LS = temp_value;
	K_D_LS_f = (float)K_D_LS/100;
	resultado += EE_ReadVariable(0x0019,&temp_value);
	K_I_LS = temp_value;
	K_I_LS_f = (float)K_I_LS/10000;

	resultado += EE_ReadVariable(0x001A,&temp_value);
	K_P_M = temp_value;
	K_P_M_f = (float)K_P_M/100;
	resultado += EE_ReadVariable(0x001B,&temp_value);
	K_D_M = temp_value;
	K_D_M_f = (float)K_D_M/100;
	resultado += EE_ReadVariable(0x001C,&temp_value);
	K_I_M = temp_value;
	K_I_M_f = (float)K_I_M/10000;

	resultado += EE_ReadVariable(0x001D,&temp_value);
	offset_corriente = temp_value - 111;
	resultado += EE_ReadVariable(0x001E,&temp_value);
	offset_temp_int = (temp_value - 150) / 10 ;
	resultado += EE_ReadVariable(0x001F,&temp_value);
	offset_temp_ext = (temp_value - 150) / 10;

	resultado += EE_ReadVariable(0x0020,&temp_value);
	envio_si_leo = temp_value;

	resultado += EE_ReadVariable(0x0021,&temp_value);
	saludo_inicial = temp_value;
	
	resultado += read_32_flash(0x0022,&temp_value_l);
	max_posicion = temp_value_l + LONG_MIN;
	resultado += read_32_flash(0x0024,&temp_value_l);
	min_posicion = temp_value_l + LONG_MIN;
	resultado += EE_ReadVariable(0x0026,&temp_value);
	limit_posicion = temp_value;
	
	resultado += EE_ReadVariable(0x0027,&temp_value);
	t_ramp_t = temp_value;
	resultado += EE_ReadVariable(0x0028,&temp_value);
	t_ramp_s = temp_value;
	resultado += EE_ReadVariable(0x0029,&temp_value);
	a_ramp_t = temp_value;

	resultado += EE_ReadVariable(0x002A,&temp_value);
	deadband = temp_value;

	resultado += EE_ReadVariable(0x002B,&temp_value);
	direccion_motor = temp_value;
	
	/*for(d_correccion=0x1000,i_correccion=0;i_correccion<256;d_correccion++,i_correccion++)
	{
		resultado += EE_ReadVariable(d_correccion,&temp_value);
		correccion_enc[i_correccion] = temp_value;
	}*/

	return(resultado);
}

void read_init_value_flash()
{
	uint16_t temp_value;
	uint32_t temp_value_l;
	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0)
	{
		switch (task_scheduler.param_1[LS_TS_ESTADO])
		{
			case 100:
				EE_ReadVariable(0x0002,&temp_value);
				ID_SERVO = temp_value;
			break;
			case 101:
				EE_ReadVariable(0x0003,&q_corriente);
				get_kalman_gains();
			break;
			case 102:
				EE_ReadVariable(0x0004,&q_temp_ext);
				get_kalman_gains();
			break;
			case 103:
				EE_ReadVariable(0x0005,&q_temp_int);
				get_kalman_gains();
			break;
			case 104:
				EE_ReadVariable(0x0006,&q_volts);
				get_kalman_gains();
			break;
			case 105:
				EE_ReadVariable(0x0007,&q_encoder);
				get_kalman_gains();
			break;
			case 106:
				EE_ReadVariable(0x0008,&varianza_corriente);
				get_kalman_gains();
			break;
			case 107:
				EE_ReadVariable(0x0009,&varianza_temp_ext);
				get_kalman_gains();
			break;
			case 108:
				EE_ReadVariable(0x000A,&varianza_temp_int);
				get_kalman_gains();
			break;
			case 109:
				EE_ReadVariable(0x000B,&varianza_volts);
				get_kalman_gains();
			break;
			case 110:
				EE_ReadVariable(0x000C,&varianza_encoder);
				get_kalman_gains();
			break;
			case 111:
				read_32_flash(0x000D,&vel_serie);
				change_USART1_speed();
			break;
			case 112:
				EE_ReadVariable(0x000F,&corte_temp_ext);
			break;
			case 113:
				EE_ReadVariable(0x0010,&corte_temp_int);
			break;
			case 114:
				EE_ReadVariable(0x0011,&corte_volts_alto);
			break;
			case 115:
				EE_ReadVariable(0x0012,&corte_volts_bajo);
			break;
			case 116:
				EE_ReadVariable(0x0013,&corte_corriente);
			break;
			case 117:
				EE_ReadVariable(0x0014,&temp_value);
				uso_crc = temp_value;
			break;
			case 118:
				EE_ReadVariable(0x0015,&temp_value);
				CRC_START_CCITT = temp_value;
			break;
			case 119:
				EE_ReadVariable(0x0016,&temp_value);
				CRC_POLY_CCITT = temp_value;
				init_crcccitt_tab();
			break;
			case 120:
				EE_ReadVariable(0x0017,&temp_value);
				K_P_LS = temp_value;
				K_P_LS_f = (float)K_P_LS/100;
			break;
			case 121:
				EE_ReadVariable(0x0018,&temp_value);
				K_D_LS = temp_value;
				K_D_LS_f = (float)K_D_LS/100;
			break;
			case 122:
				EE_ReadVariable(0x0019,&temp_value);
				K_I_LS = temp_value;
				K_I_LS_f = (float)K_I_LS/10000;
			break;
			case 123:
				EE_ReadVariable(0x001A,&temp_value);
				K_P_M = temp_value;
				K_P_M_f = (float)K_P_M/100;
			break;
			case 124:
				EE_ReadVariable(0x001B,&temp_value);
				K_D_M = temp_value;
				K_D_M_f = (float)K_D_M/100;
			break;
			case 125:
				EE_ReadVariable(0x001C,&temp_value);
				K_I_M = temp_value;
				K_I_M_f = (float)K_I_M/10000;
			break;
			case 126:
				EE_ReadVariable(0x001D,&temp_value);
				offset_corriente = temp_value - 111;
			break;
			case 127:
				EE_ReadVariable(0x001E,&temp_value);
				offset_temp_int = (temp_value - 150) / 10;
			break;
			case 128:
				EE_ReadVariable(0x001F,&temp_value);
				offset_temp_ext = (temp_value - 150) / 10;
			break;
			case 129:
				EE_ReadVariable(0x0020,&temp_value);
				envio_si_leo = temp_value;
			break;
			case 130:
				EE_ReadVariable(0x0021,&temp_value);
				saludo_inicial = temp_value;
			break;
			case 131:
				read_32_flash(0x0022,&temp_value_l);
				max_posicion = temp_value_l + LONG_MIN;
			break;
			case 132:
				read_32_flash(0x0024,&temp_value_l);
				min_posicion = temp_value_l + LONG_MIN;
			break;
			case 133:
				EE_ReadVariable(0x0026,&temp_value);
				limit_posicion = temp_value;
			break;
			case 134:
				EE_ReadVariable(0x0027,&temp_value);
				t_ramp_t = temp_value;
			break;
			case 135:
				EE_ReadVariable(0x0028,&temp_value);
				t_ramp_s = temp_value;
			break;
			case 136:
				EE_ReadVariable(0x0029,&temp_value);
				a_ramp_t = temp_value;
			break;
			case 137:
				EE_ReadVariable(0x002A,&temp_value);
				deadband = temp_value;
			break;
			case 138:
				EE_ReadVariable(0x002B,&temp_value);
				direccion_motor = temp_value;
			break;
			default:
			break;
		}
		/*if(task_scheduler.param_1[LS_TS_ESTADO]>=0x1000 && task_scheduler.param_1[LS_TS_ESTADO]<=0x1100)
		{
			EE_ReadVariable(task_scheduler.param_1[LS_TS_ESTADO],&temp_value);
			correccion_enc[task_scheduler.param_1[LS_TS_ESTADO]-0x1000] = temp_value;
		}*/
	}
	task_scheduler.iniciado[LS_TS_ESTADO] = 2;
}
void save_new_values_flash()
{
	uint16_t value;
	uint32_t value_32;
	//uint16_t i_correccion, d_correccion;

	value = 0;
	EE_ReadVariable(0x0001,&value);
	if (value != version_LS) EE_WriteVariable(0x0001,version_LS);		//version LS

	EE_ReadVariable(0x0002,&value);
	if (value != ID_SERVO) EE_WriteVariable(0x0002,ID_SERVO);		//ID_SERVO

	EE_ReadVariable(0x0003,&value);
	if (value != q_corriente) EE_WriteVariable(0x0003,q_corriente);	//Incertidumbre proceso corriente (x1000)
	EE_ReadVariable(0x0004,&value);
	if (value != q_temp_ext) EE_WriteVariable(0x0004,q_temp_ext);	//Incertidumbre proceso temp_ext (x1000)
	EE_ReadVariable(0x0005,&value);
	if (value != q_temp_int) EE_WriteVariable(0x0005,q_temp_int);	//Incertidumbre proceso temp_int (x1000)
	EE_ReadVariable(0x0006,&value);
	if (value != q_volts) EE_WriteVariable(0x0006,q_volts);	//Incertidumbre proceso voltaje (x1000)
	EE_ReadVariable(0x0007,&value);
	if (value != q_encoder) EE_WriteVariable(0x0007,q_encoder);	//Incertidumbre proceso encoder (x1000)
	EE_ReadVariable(0x0008,&value);
	if (value != varianza_corriente) EE_WriteVariable(0x0008,varianza_corriente);	//Varianza corriente (x1000)
	EE_ReadVariable(0x0009,&value);
	if (value != varianza_temp_ext) EE_WriteVariable(0x0009,varianza_temp_ext);	//Varianza proceso temp_ext (x1000)
	EE_ReadVariable(0x000A,&value);
	if (value != varianza_temp_int) EE_WriteVariable(0x000A,varianza_temp_int);	//Varianza proceso temp_int (x1000)
	EE_ReadVariable(0x000B,&value);
	if (value != varianza_volts) EE_WriteVariable(0x000B,varianza_volts);	//Varianza proceso voltaje (x1000)
	EE_ReadVariable(0x000C,&value);
	if (value != varianza_encoder) EE_WriteVariable(0x000C,varianza_encoder);	//Varianza proceso encoder (x1000)

	value_32 = 0;
	read_32_flash(0x000D,&value_32);
	if (value != vel_serie) write_32_flash(0x000D,vel_serie);	//velocidad serie 0x000D y 0x000E

	EE_ReadVariable(0x000F,&value);
	if (value != corte_temp_ext) EE_WriteVariable(0x000F,corte_temp_ext);	//corte_temp_ext (x10)
	EE_ReadVariable(0x0010,&value);
	if (value != corte_temp_int) EE_WriteVariable(0x0010,corte_temp_int);	//corte_temp_int (x10)
	EE_ReadVariable(0x0011,&value);
	if (value != corte_volts_alto) EE_WriteVariable(0x0011,corte_volts_alto);	//corte_volt_alto (x100)
	EE_ReadVariable(0x0012,&value);
	if (value != corte_volts_bajo) EE_WriteVariable(0x0012,corte_volts_bajo);	//corte_volt_bajo (x100)
	EE_ReadVariable(0x0013,&value);
	if (value != corte_corriente) EE_WriteVariable(0x0013,corte_corriente);	//corte_corriente (x1000)

	EE_ReadVariable(0x0014,&value);
	if (value != uso_crc) EE_WriteVariable(0x0014,uso_crc);	//Uso CRC (0=False, 1=True, 2=Both)
	EE_ReadVariable(0x0015,&value);
	if (value != CRC_START_CCITT) EE_WriteVariable(0x0015,CRC_START_CCITT);	//CRC_START_CCITT_1D0F	0x1D0F
	EE_ReadVariable(0x0016,&value);
	if (value != CRC_POLY_CCITT) EE_WriteVariable(0x0016,CRC_POLY_CCITT);	//CRC_POLY_CCITT		0x1021

	EE_ReadVariable(0x0017,&value);
	if (value != K_P_LS) EE_WriteVariable(0x0017,K_P_LS);		//K_P_LS		(/100)
	EE_ReadVariable(0x0018,&value);
	if (value != K_D_LS) EE_WriteVariable(0x0018,K_D_LS);		//K_D_LS		(/100)
	EE_ReadVariable(0x0019,&value);
	if (value != K_I_LS) EE_WriteVariable(0x0019,K_I_LS);		//K_I_LS		(/10000)

	EE_ReadVariable(0x001A,&value);
	if (value != K_P_M) EE_WriteVariable(0x001A,K_P_M);			//K_P_M		(/100)
	EE_ReadVariable(0x001B,&value);
	if (value != K_D_M) EE_WriteVariable(0x001B,K_D_M);			//K_D_M		(/100)
	EE_ReadVariable(0x001C,&value);
	if (value != K_I_M) EE_WriteVariable(0x001C,K_I_M);			//K_I_M		(/10000)

	EE_ReadVariable(0x001D,&value);
	if (value != (offset_corriente + 111)) EE_WriteVariable(0x001D,(offset_corriente + 111));					//offset_corriente	111		(0-222 -> -1000 - 1000)
	EE_ReadVariable(0x001E,&value);
	if (value != ((offset_temp_int * 10) + 150 )) EE_WriteVariable(0x001E,(offset_temp_int * 10) + 150 );		//offset_temp_int	150		(0-300 -> -15.0 - 15.0)
	EE_ReadVariable(0x001F,&value);
	if (value != ((offset_temp_ext * 10) + 150 )) EE_WriteVariable(0x001F,(offset_temp_ext * 10) + 150 );		//offset_temp_ext	150		(0-300 -> -15.0 - 15.0)

	EE_ReadVariable(0x0020,&value);
	if (value != envio_si_leo) EE_WriteVariable(0x0020,envio_si_leo);											//No enviar si se reciben datos (0=False, 1=True, 2=True envío actual)

	EE_ReadVariable(0x0021,&value);
	if (value != saludo_inicial) EE_WriteVariable(0x0021,saludo_inicial);										//Saludo Inicial (0=False, 1=True)
		
	read_32_flash(0x0022,&value_32);
	if (value_32 != (max_posicion - LONG_MIN)) write_32_flash(0x0022,(max_posicion - LONG_MIN));				//max_posicion 0x0022 y 0x0023		(0-4294967294 -> -2147483647 - 2147483647)
	read_32_flash(0x0024,&value_32);
	if (value_32 != (min_posicion - LONG_MIN)) write_32_flash(0x0024,(min_posicion - LONG_MIN));				//min_posicion 0x0024 y 0x0025		(0-4294967294 -> -2147483647 - 2147483647)
	EE_ReadVariable(0x0026,&value);
	if (value != limit_posicion) EE_WriteVariable(0x0026,limit_posicion);										//limites posición (0=False, 1=True)
	
	EE_ReadVariable(0x0027,&value);
	if (value != t_ramp_t) EE_WriteVariable(0x0027,t_ramp_t);													//tiempo_rampa_trapezoidal 200ms
	EE_ReadVariable(0x0028,&value);
	if (value != t_ramp_s) EE_WriteVariable(0x0028,t_ramp_s);													//tiempo_rampa_senoidal 200ms
	EE_ReadVariable(0x0029,&value);
	if (value != a_ramp_t) EE_WriteVariable(0x0029,a_ramp_t);													//aceleración_rampa_trapezoidal 100

	EE_ReadVariable(0x002A,&value);
	if (value != deadband) EE_WriteVariable(0x002A,deadband);													//deadband 5

	EE_ReadVariable(0x002B,&value);
	if (value != direccion_motor) EE_WriteVariable(0x002B,direccion_motor);										//direccion_motor 0
	
	/*for(d_correccion=0x1000,i_correccion=0;i_correccion<256;d_correccion++,i_correccion++)
	{
		EE_ReadVariable(d_correccion,&value);
		if(correccion_enc[i_correccion] != value) EE_WriteVariable(d_correccion,correccion_enc[i_correccion]);
	}*/
}
