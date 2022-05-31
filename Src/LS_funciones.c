/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LS_funciones.c
  * @brief          : All generic functions are in this file
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

volatile uint32_t ticks = 0;
//uint32_t tiempo_bat = 0;

struct s_led_rgb {uint32_t pin[3];uint8_t brillo[3];};
volatile struct s_led_rgb led_rgb = {			//The actual state
	{LED_GREEN,LED_RED,LED_BLUE} ,
	{255,0,0} };								//The three leds must sum 255, if the first led is already 255 means everything is shutdown
volatile struct s_led_rgb led_rgb_temp = {		//The next loop state
	{LED_GREEN,LED_RED,LED_BLUE} ,
	{255,0,0} };
volatile int estado_led = 2;					//Which Led is shining
volatile uint8_t new_led_rgb = TRUE;
volatile int brillado_led_rgb = 0;

volatile boolean LED_modo_RAINBOW = FALSE;
volatile uint8_t rgbColor[3] = {255,0,0}; //Empieza con Rojo el arcoíris
volatile uint8_t decColor = 0;
volatile uint8_t incColor = 1;
volatile uint8_t brillo_led = 0;

extern volatile boolean status_tone;			//Inicializado en main.c

volatile uint32_t data_SSI_IN;
uint8_t error_SSI;
uint8_t paridad_SSI;

uint16_t q_corriente, q_temp_ext, q_temp_int, q_volts, q_encoder;
uint16_t varianza_corriente, varianza_temp_ext, varianza_temp_int, varianza_volts, varianza_encoder;
volatile float k_corriente = 0.0;
float k_temp_ext = 0.0;
float k_temp_int = 0.0;
float k_volts = 0.0;
volatile float k_encoder = 0.0;

struct LS_temp_task_scheduler {uint8_t comando[LS_TASK_SCHEDULER_SIZE];long param_1[LS_TASK_SCHEDULER_SIZE];long param_2[LS_TASK_SCHEDULER_SIZE];long param_3[LS_TASK_SCHEDULER_SIZE];long param_4[LS_TASK_SCHEDULER_SIZE];long param_5[LS_TASK_SCHEDULER_SIZE];long time_comando[LS_TASK_SCHEDULER_SIZE];boolean prioritaria[LS_TASK_SCHEDULER_SIZE];};
volatile struct LS_temp_task_scheduler temp_task_scheduler = {{0},{0},{0},{0},{0},{0},{0},{0}};

struct LS_task_scheduler {uint8_t comando[LS_TASK_SCHEDULER_SIZE];long param_1[LS_TASK_SCHEDULER_SIZE];long param_2[LS_TASK_SCHEDULER_SIZE];long param_3[LS_TASK_SCHEDULER_SIZE];long param_4[LS_TASK_SCHEDULER_SIZE];long param_5[LS_TASK_SCHEDULER_SIZE];long time_comando[LS_TASK_SCHEDULER_SIZE];uint8_t iniciado[LS_TASK_SCHEDULER_SIZE];};

//volatile struct LS_task_scheduler task_scheduler = {{0},{0},{0},{0},{0},{0},{0},{[0 ... (LS_TASK_SCHEDULER_SIZE-1)] = 5}};
volatile struct LS_task_scheduler task_scheduler = {{0},{0},{0},{0},{0},{0},{0},{0}};
volatile uint8_t LS_TS_ESTADO = 0, LS_TEMP_TS_GUARDADO = 0, LS_TS_GUARDADO = 0;
struct LS_comando_motor {uint8_t comando; long pos_now; float velocidad; float aceleracion; float velocidad_int; float aceleracion_int; long pos_destino; float pos_destino_f; long pos_ini; long pos_final; long time_to_end; float step; long time_ramp1; long time_ramp2; float step_ramp1; float step_ramp2; int output_LS;};
volatile struct LS_comando_motor comando_motor = {0,0,0.0,0.0,0,0.0,0.0,0.0,0,0,0,0.0,0,0,0.0,0.0,0};

volatile uint16_t crc_tabccitt[256];

struct LS_temp_comandos_serial {uint8_t orden_sensores[LS_TASK_SERIAL_SIZE][TAM_BUFF_SENSORES]; long times[LS_TASK_SERIAL_SIZE]; long time[LS_TASK_SERIAL_SIZE]; boolean prioritaria[LS_TASK_SERIAL_SIZE]; boolean cabecera[LS_TASK_SERIAL_SIZE];};
volatile struct LS_temp_comandos_serial temp_comandos_serial = {{},{0},{0},{0},{0}};
struct LS_comandos_serial {uint8_t orden_sensores[LS_TASK_SERIAL_SIZE][TAM_BUFF_SENSORES]; long times[LS_TASK_SERIAL_SIZE]; long time[LS_TASK_SERIAL_SIZE];uint8_t iniciado[LS_TASK_SERIAL_SIZE]; boolean cabecera[LS_TASK_SERIAL_SIZE];};
volatile struct LS_comandos_serial comandos_serial = {{},{0},{0},{0},{0}};
volatile uint8_t LS_TS_ESTADO_SERIAL = 0, LS_TEMP_TS_GUARDADO_SERIAL = 0, LS_TS_GUARDADO_SERIAL = 0;

volatile boolean send_RS485 = FALSE;
uint32_t sum_enviados = 0;
uint32_t serial_sum_a_enviar = 0;
uint8_t LS_ESTADO_RS485 = 0;
volatile uint8_t serial_RS485_datos_enviar = 0;
uint32_t tiempo_RS485 = 0;
volatile uint8_t enviado_USART1;
volatile uint8_t enviado_USART1 = TRUE;
volatile uint8_t recibido_USART1 = FALSE;
boolean enviado_cabecera = FALSE;
long buff_sensores[TAM_BUFF_SENSORES];
volatile uint8_t temp_orden_sensores[TAM_BUFF_SENSORES] = {};
volatile int corriente = 0;
volatile float corriente_raw = 0;
volatile float corriente_kalman = 0;
extern volatile long data_SSI_IN_internal;					//Inicializado en main.c
extern char dma_tx_buff[DMA_TX_BUFF_SIZE+12];				//Inicializado en main.c
extern uint16_t version_LS;									//Inicializado en main.c
extern volatile uint16_t dma_adc1_buff[TAM_DMA_ADC1_BUFF];	//inicializado en main.c
volatile float voltaje_bat_raw_kalman = 0.0;
volatile float voltaje_bat_raw = 0;
volatile uint16_t voltaje_bat = 0;
volatile uint8_t bat_estado = 0;							//Estado batería 0=Good, 1=LOW, 2=HIGH

/*volatile float internal_temp_C = 0.0;
volatile float external_temp_C = 0.0;
volatile float internal_temp = 0.0;
//volatile float internal_temp2 = 0.0;
volatile float external_temp = 0.0;
volatile float external_temp_A = 0.0;*/
//volatile float external_temp_B = 0.0;
float media_pendiente_temp = 0.0;
volatile float internal_temp_kalman = 0.0;
volatile float external_temp_kalman = 0.0;
volatile uint8_t temp_estado = 0;							//Estado temperature 0=Good, 1=HIGH_external, 2=HIGH_internal, 3=HIGH_BOTH

volatile uint16_t CRC_POLY_CCITT;

extern volatile uint16_t ID_SERVO;						//Inicializado en main.c
extern uint32_t vel_serie;								//Inicializado en main.c
uint16_t corte_temp_ext, corte_temp_int, corte_volts_alto, corte_volts_bajo, corte_corriente;
volatile uint16_t uso_crc; 								//0=False, 1=True, 2=Both
volatile uint16_t CRC_START_CCITT;
extern volatile uint16_t K_P_LS, K_D_LS, K_I_LS;		//Inicializado en LS_flash.c
extern volatile float K_P_LS_f, K_D_LS_f, K_I_LS_f;		//Inicializado en LS_PID.c
extern volatile uint16_t K_P_M, K_D_M, K_I_M;			//Inicializado en LS_flash.c
extern volatile float K_P_M_f, K_D_M_f, K_I_M_f;		//Inicializado en LS_PID.c
extern volatile int offset_corriente;					//Inicializado en LS_flash.c
extern volatile float offset_temp_int;					//Inicializado en LS_flash.c
extern volatile float offset_temp_ext;					//Inicializado en LS_flash.c
extern volatile uint16_t envio_si_leo;					//Inicializado en main.c No enviar si se reciben datos (0=False, 1=True, 2=True envío actual)
extern volatile uint8_t saludo_inicial;					//Inicializado en LS_flash.c Saludo Inicial (0=False, 1=True)

extern volatile long min_posicion, max_posicion;		//Inicializado en LS_flash.c Min y máx posición (0-4294967294 -> -2147483647 - 2147483647)
extern volatile uint8_t limit_posicion;					//Inicializado en LS_flash.c limites posición (0=False, 1=True)

extern volatile uint16_t t_ramp_t, t_ramp_s, a_ramp_t;	//Inicializado en LS_flash.c

extern volatile uint16_t deadband; 						//Inicializado en LS_flash.c

char cabeceras[NUM_VARIABLES][11] =
{ "T_int,",
  "T_ext,",
  "T_int_RAW,",
  "T_int_K,",
  "T_ext_RAW,",
  "T_ext_K,",
  "Volts,",
  "Volt_RAW,",
  "Volt_K,",
  "I,",
  "I_RAW,",
  "I_K,",
  "Enc_RAW,",
  "Enc_K,",
  "Speed,",
  "Accel,",
  "Speed_int,",
  "Accel_int,",
  "Tarjet,",
  "Error,",
  "D_Error,",
  "PD_Term,",
  "I_term,",
  "PWM,",
  "PWM_pre,",
  "LS_VER,",
  "ID,",
  "Q_Current,",
  "Q_T_ext,",
  "Q_T_int,",
  "Q_Volts,",
  "Q_Enc,",
  "V_Current,",
  "V_T_ext,",
  "V_T_int,",
  "V_Volts,",
  "V_Enc,",
  "Vel_serie,",
  "C_T_ext,",
  "C_T_int,",
  "C_Volts_H,",
  "C_Volts_L,",
  "C_Current,",
  "Use_CRC,",
  "CRC_START,",
  "CRC_POLY,",
  "K_P_LS,",
  "K_D_LS,",
  "K_I_LS,",
  "K_P_M,",
  "K_D_M,",
  "K_I_M,",
  "OFF_I,",
  "OFF_T_Int,",
  "OFF_T_Ext,",
  "TX_If_RX,",
  "HELLO,",
  "MIN_Pos,",
  "MAX_Pos,",
  "LIMIT_Pos,",
  "t_ramp_t,",
  "t_ramp_s,",
  "a_ramp_t,",
  "deadband,"
};
  
  

/*
 * Declare task_scheduler and initialize as {[0 ... (LS_TASK_SCHEDULER_SIZE-1)] = 5}, cost 4% of Flash!!
 * the below function less than 0.01%... Ridiculous
 */
void init_variables()
{
	uint16_t i;
	for(i = 0;i < LS_TASK_SCHEDULER_SIZE;i++) {task_scheduler.iniciado[i] = 5;}
	for(i = 0;i < LS_TASK_SERIAL_SIZE;i++) {comandos_serial.iniciado[i] = 5;}
	for(i = 0;i < TAM_BUFF_SENSORES;i++) {temp_orden_sensores[i] = 255;}

	media_pendiente_temp = 80/(float)((int)*TEMPSENSOR_CAL2_ADDR-(int)*TEMPSENSOR_CAL1_ADDR);
}
void change_USART1_speed()
{
	LL_USART_SetBaudRate(USART1,
						 LL_RCC_GetUSARTClockFreq(LL_RCC_USART1_CLKSOURCE),
						 LL_USART_OVERSAMPLING_8,
						 vel_serie);
}
void init_USART1_rx_as_INPUT()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void Delay_ms(uint32_t nCount)
{
	uint32_t start = ticks ;

	while( ticks - start < nCount )
	{
	// do nothing
	}
}

uint32_t Get_Micros()
{
    uint32_t ms;
    uint32_t st;

    do
    {
        ms = ticks;
        st = SysTick->VAL;
        asm volatile("nop");
        asm volatile("nop");
    } while (ms != ticks);			//Asegurarnos que una interrupción no haya cambiado justo ticks al leerlo

    return ms * 1000 - st / 72;
}

void SET_PIN (GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
	GPIOx->BSRR=PinMask;
}
void CLEAR_PIN (GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
	GPIOx->BRR=PinMask;
}
void FLIP_PIN (GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
	GPIOx->ODR^=PinMask;
}
uint32_t READ_PIN (GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
	return (READ_BIT(GPIOx->IDR, PinMask) == (PinMask));
}

void F_LED_RGB(uint8_t Z_RED, uint8_t Z_GREEN, uint8_t Z_BLUE)
{
	led_rgb_temp = (struct s_led_rgb){
		{LED_RED,LED_GREEN,LED_BLUE} ,
		{Z_RED,Z_GREEN,Z_BLUE} };
	uint8_t z_brillo;
	uint32_t z_pin;

	for(int a=0;a<=1;a++)
	{
		for(int b=a+1;b<=2;b++)
		{
			if(led_rgb_temp.brillo[a]>led_rgb_temp.brillo[b])				//Order from less to more brightness
			{
				z_brillo=led_rgb_temp.brillo[a];
				z_pin=led_rgb_temp.pin[a];
				led_rgb_temp.brillo[a]=led_rgb_temp.brillo[b];
				led_rgb_temp.pin[a]=led_rgb_temp.pin[b];
				led_rgb_temp.brillo[b]=z_brillo;
				led_rgb_temp.pin[b]=z_pin;
			}
		}
	}

	for(int a=1;a<3;a++)
	{
		led_rgb_temp.brillo[a]=led_rgb_temp.brillo[a]-(led_rgb_temp.brillo[0]);
	}
	led_rgb_temp.brillo[2]=led_rgb_temp.brillo[2]-led_rgb_temp.brillo[1];

	/*for(int a=0;a<3;a++)
	{
		led_rgb_temp.brillo[a]=255-led_rgb_temp.brillo[a];
	}*/
	new_led_rgb=TRUE;
}

void comando_LED_RGB()
{
	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0)
	{
		F_LED_RGB(task_scheduler.param_1[LS_TS_ESTADO],task_scheduler.param_2[LS_TS_ESTADO],task_scheduler.param_3[LS_TS_ESTADO]);
	}
	task_scheduler.iniciado[LS_TS_ESTADO] = 2;
	LED_modo_RAINBOW = FALSE;
}
void comando_LED_RAINBOW()
{
	rgbColor[0] = 255;
	rgbColor[1] = 0;
	rgbColor[2] = 0;
	decColor = 0;
	incColor = 1;
	brillo_led = 0;
	task_scheduler.iniciado[LS_TS_ESTADO] = 2;
	LED_modo_RAINBOW = TRUE;
	estado_led = 2;
	brillado_led_rgb = 255;
}
void RAINBOW()
{
	if(brillo_led < 255)
	{
		brillo_led++;
		rgbColor[decColor]--;
		rgbColor[incColor]++;
		F_LED_RGB(rgbColor[0], rgbColor[1], rgbColor[2]);
	}
	else
	{
		decColor++;
		incColor++;
		if(incColor == 3) incColor = 0;
		if(decColor == 3) decColor = 0;
		brillo_led = 0;
	}
}

void tone(uint16_t hz, uint16_t duracion_ms, uint16_t  amplitud)
{
	uint32_t ciclos, micros, periodo_us;

	periodo_us = 1000000 / (uint32_t) hz;
	ciclos = (uint32_t) duracion_ms * hz / 1000;

	//if (amplitud > TIM1->ARR) amplitud = TIM1->ARR;

	for (uint32_t n = 0; n < ciclos; n++)
	{
		TIM1->CCR2 = 0;
		TIM1->CCR1 = amplitud;
		micros = Get_Micros();
		while ((Get_Micros() - micros) < periodo_us) {/*Espera*/}
		TIM1->CCR1 = 0;
		TIM1->CCR2 = amplitud;
		micros = Get_Micros();
		while ((Get_Micros() - micros) < periodo_us) {/*Espera*/}
	}
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
}

void comando_tone()
{
	uint16_t amplitud;
	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0)
	{
		task_scheduler.iniciado[LS_TS_ESTADO] = 1;
		amplitud = task_scheduler.param_3[LS_TS_ESTADO];
		if (amplitud > 100) amplitud = 100;
		amplitud = amplitud * TIM1->ARR/100;
		//task_scheduler.param_3[LS_TS_ESTADO] = amplitud;
		tone(task_scheduler.param_1[LS_TS_ESTADO],task_scheduler.param_2[LS_TS_ESTADO],amplitud);
		task_scheduler.iniciado[LS_TS_ESTADO] = 2;
	}
}

void int_to_char(char *str, long my_int)
{
	//uint8_t len_string = 10;
	char temp_string[11] = {0};
	int i = 11;
	int b = 0;
	boolean negativo = FALSE;
	if (my_int<0)
	{
		negativo = TRUE;
		my_int=-my_int;
	}

	if (my_int == 0)
	{
		temp_string[10] = '0';		//len(temp_string) -1
		i--;
	}
	if (!negativo)
	{
		while (my_int != 0)
		{
			i--;
			temp_string[i] = (my_int % 10) + '0';
			my_int /= 10;
		}
	}
	else
	{
		while (my_int != 0)
		{
			i--;
			temp_string[i] = (my_int % 10) + '0';
			my_int /= 10;
		}
		i--;
		temp_string[i] = '-';
	}
	while (i != 11)
	{
		str[b] = temp_string[i];
		i++;
		b++;
	}
}

void read_bat()
{
	voltaje_bat_raw = (dma_adc1_buff[3]+dma_adc1_buff[7]+dma_adc1_buff[11]+dma_adc1_buff[15]+dma_adc1_buff[19]+dma_adc1_buff[23]+dma_adc1_buff[27]+dma_adc1_buff[31])/8;
	voltaje_bat_raw_kalman = voltaje_bat_raw_kalman + k_volts*(voltaje_bat_raw-voltaje_bat_raw_kalman);
	voltaje_bat = voltaje_bat_raw * 2015 / 4096;

	if( bat_estado == 0 )
	{
		if( voltaje_bat < corte_volts_bajo ) bat_estado = 1;
		if( voltaje_bat > corte_volts_alto ) bat_estado = 2;
	}
	else
	{
		if(bat_estado == 1 && voltaje_bat > (corte_volts_bajo + 30)) bat_estado = 0;
		if(bat_estado == 2 && voltaje_bat < (corte_volts_alto - 50)) bat_estado = 0;
	}
	buff_sensores[6] = voltaje_bat;
	buff_sensores[7] = (uint16_t)voltaje_bat_raw;
	buff_sensores[8] = (uint16_t)voltaje_bat_raw_kalman;
}

void read_temps()
{
	float internal_temp_C = 0.0;
	float external_temp_C = 0.0;
	float internal_temp = 0.0;
	float external_temp = 0.0;
	float external_temp_A = 0.0;
	
	internal_temp = (dma_adc1_buff[0]+dma_adc1_buff[4]+dma_adc1_buff[8]+dma_adc1_buff[12]+dma_adc1_buff[16]+dma_adc1_buff[20]+dma_adc1_buff[24]+dma_adc1_buff[28])/8;
	internal_temp_kalman = internal_temp_kalman + k_temp_int*(internal_temp-internal_temp_kalman);

	external_temp = (dma_adc1_buff[1]+dma_adc1_buff[5]+dma_adc1_buff[9]+dma_adc1_buff[13]+dma_adc1_buff[17]+dma_adc1_buff[21]+dma_adc1_buff[25]+dma_adc1_buff[29])/8;
	external_temp_kalman = external_temp_kalman + k_temp_ext*(external_temp-external_temp_kalman);
	
	internal_temp_C = (internal_temp_kalman - *TEMPSENSOR_CAL1_ADDR)*media_pendiente_temp + 30;
	internal_temp_C = internal_temp_C + offset_temp_int;

	if(external_temp_kalman==2048) external_temp_A = 2047*3.3/4096;		//Volt=lectura*3.3V*4096
	else external_temp_A = external_temp_kalman*3.3/4096;				//Volt=lectura*3.3V*4096

	external_temp_C = 3.3-external_temp_A;				//Caida_Volt=3.3-Volt
	external_temp_C = external_temp_C/10000;			//Corriente=Caida_Volt/10KOhms
	external_temp_C = external_temp_A/external_temp_C;	//R2=Volt/Corriente
	external_temp_A = 3435/log(10000/external_temp_C);
	external_temp_C = ((25+273.15)*external_temp_A)/(external_temp_A-(25+273.15))-273.15;
	external_temp_C = external_temp_C + offset_temp_ext;
	
	if( temp_estado == 0 )
	{
		if( internal_temp_C > corte_temp_int ) temp_estado = 1;
		if( external_temp_C > corte_temp_ext ) temp_estado += 2;
	}
	else
	{
		temp_estado = 0;
		if( internal_temp_C > (corte_temp_int + 50) ) temp_estado = 1;
		if( external_temp_C > (corte_temp_ext + 50) ) temp_estado += 2;
	}
	
	buff_sensores[0] = (uint16_t)internal_temp_C;
	buff_sensores[1] = (uint16_t)external_temp_C;
	buff_sensores[2] = (uint16_t)internal_temp;
	buff_sensores[3] = (uint16_t)internal_temp_kalman;
	buff_sensores[4] = (uint16_t)external_temp;
	buff_sensores[5] = (uint16_t)external_temp_kalman;
}

void comando_get()
{
	if(send_RS485 == TRUE && comandos_serial.iniciado[LS_TS_ESTADO_SERIAL] == 1)
	{
		int string_len = 0, num_cabecera;

		if(ticks - tiempo_RS485 >= comandos_serial.time[LS_TS_ESTADO_SERIAL])
		{
		  if(enviado_USART1 == TRUE && comandos_serial.orden_sensores[LS_TS_ESTADO_SERIAL][0] != 255)
		  {
			  buff_sensores[9] = (uint16_t)corriente;
			  buff_sensores[10] = (uint16_t)corriente_raw;
			  buff_sensores[11] = (uint16_t)corriente_kalman;
			  buff_sensores[12] = (uint16_t)data_SSI_IN_internal;
			  buff_sensores[13] = comando_motor.pos_now;
			  buff_sensores[14] = (long)(comando_motor.velocidad*1000);
			  buff_sensores[15] = (long)(comando_motor.aceleracion*1000);
			  buff_sensores[16] = (long)(comando_motor.velocidad_int*1000);
			  buff_sensores[17] = (long)(comando_motor.aceleracion_int*1000);
			  buff_sensores[18] = comando_motor.pos_destino;
			  if (TIM1->CCR1 == 0) buff_sensores[23] = TIM1->CCR2;
			  else buff_sensores[23] = TIM1->CCR1;
			  buff_sensores[24] = comando_motor.output_LS;

			  tiempo_RS485 = ticks;

			  comandos_serial.times[LS_TS_ESTADO_SERIAL]--;
			  if(comandos_serial.times[LS_TS_ESTADO_SERIAL] == 0) send_RS485 = FALSE;

			  memset(dma_tx_buff,0,strlen(dma_tx_buff));  //Vaciamos buffer de envío
			  //orden_sensores[LS_TASK_SERIAL_SIZE][TAM_BUFF_SENSORES]

			  if (comandos_serial.cabecera[LS_TS_ESTADO_SERIAL] == TRUE && enviado_cabecera == FALSE)
			  {
				  enviado_cabecera = TRUE;
				  //for(uint8_t i_RS485 = 0;i_RS485 < TAM_BUFF_SENSORES && comandos_serial.orden_sensores[LS_TS_ESTADO_SERIAL][i_RS485] != 255 && string_len < DMA_TX_BUFF_SIZE;i_RS485++)
				  for(uint8_t i_RS485 = 0;i_RS485 < TAM_BUFF_SENSORES && string_len < DMA_TX_BUFF_SIZE;i_RS485++)
				  {
					  num_cabecera = comandos_serial.orden_sensores[LS_TS_ESTADO_SERIAL][i_RS485];
					  if (num_cabecera == 255) break;
					  if(num_cabecera >= 99) num_cabecera = num_cabecera - (99 - TAM_BUFF_SENSORES);
					  if(num_cabecera <= NUM_VARIABLES)
					  {
						  for(uint8_t i_array = 0;cabeceras[num_cabecera][i_array] != '\0';i_array++)
						  {
							  dma_tx_buff[string_len]=cabeceras[num_cabecera][i_array];
							  string_len++;
						  }
					  }
				  }

				  if (string_len > 0)
				  {
					dma_tx_buff[string_len-1]=10;
					dma_tx_buff[string_len]=13;
					string_len++;
				  }
			  }

			  for(uint8_t i_RS485 = 0;i_RS485 < TAM_BUFF_SENSORES && comandos_serial.orden_sensores[LS_TS_ESTADO_SERIAL][i_RS485] != 255 && string_len < DMA_TX_BUFF_SIZE;i_RS485++)
			  {
				  if (comandos_serial.orden_sensores[LS_TS_ESTADO_SERIAL][i_RS485] < TAM_BUFF_SENSORES && comandos_serial.orden_sensores[LS_TS_ESTADO_SERIAL][i_RS485] >= 0)
				  {
					  int_to_char(&dma_tx_buff[string_len],(long)(buff_sensores[comandos_serial.orden_sensores[LS_TS_ESTADO_SERIAL][i_RS485]]));
					  string_len = strlen(dma_tx_buff);
					  dma_tx_buff[string_len]=',';
					  string_len++;
				  }
				  else
				  {
					  switch (comandos_serial.orden_sensores[LS_TS_ESTADO_SERIAL][i_RS485])
					  {
						  case 99:
							int_to_char(&dma_tx_buff[string_len],(long)(version_LS));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 100:
							int_to_char(&dma_tx_buff[string_len],(long)(ID_SERVO));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 101:
							int_to_char(&dma_tx_buff[string_len],(long)(q_corriente));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 102:
							int_to_char(&dma_tx_buff[string_len],(long)(q_temp_ext));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 103:
							int_to_char(&dma_tx_buff[string_len],(long)(q_temp_int));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 104:
							int_to_char(&dma_tx_buff[string_len],(long)(q_volts));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 105:
							int_to_char(&dma_tx_buff[string_len],(long)(q_encoder));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 106:
							int_to_char(&dma_tx_buff[string_len],(long)(varianza_corriente));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 107:
							int_to_char(&dma_tx_buff[string_len],(long)(varianza_temp_ext));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 108:
							int_to_char(&dma_tx_buff[string_len],(long)(varianza_temp_int));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 109:
							int_to_char(&dma_tx_buff[string_len],(long)(varianza_volts));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 110:
							int_to_char(&dma_tx_buff[string_len],(long)(varianza_encoder));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 111:
							int_to_char(&dma_tx_buff[string_len],(long)(vel_serie));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 112:
							int_to_char(&dma_tx_buff[string_len],(long)(corte_temp_ext));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 113:
							int_to_char(&dma_tx_buff[string_len],(long)(corte_temp_int));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 114:
							int_to_char(&dma_tx_buff[string_len],(long)(corte_volts_alto));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 115:
							int_to_char(&dma_tx_buff[string_len],(long)(corte_volts_bajo));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 116:
							int_to_char(&dma_tx_buff[string_len],(long)(corte_corriente));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 117:
							int_to_char(&dma_tx_buff[string_len],(long)(uso_crc));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 118:
							int_to_char(&dma_tx_buff[string_len],(long)(CRC_START_CCITT));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 119:
							int_to_char(&dma_tx_buff[string_len],(long)(CRC_POLY_CCITT));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 120:
							int_to_char(&dma_tx_buff[string_len],(long)(K_P_LS));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 121:
							int_to_char(&dma_tx_buff[string_len],(long)(K_D_LS));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 122:
							int_to_char(&dma_tx_buff[string_len],(long)(K_I_LS));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 123:
							int_to_char(&dma_tx_buff[string_len],(long)(K_P_M));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 124:
							int_to_char(&dma_tx_buff[string_len],(long)(K_D_M));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 125:
							int_to_char(&dma_tx_buff[string_len],(long)(K_I_M));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 126:
							int_to_char(&dma_tx_buff[string_len],(long)(offset_corriente*9));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 127:
							int_to_char(&dma_tx_buff[string_len],(long)(offset_temp_int*10+150));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 128:
							int_to_char(&dma_tx_buff[string_len],(long)(offset_temp_ext*10+150));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 129:
							int_to_char(&dma_tx_buff[string_len],(long)(envio_si_leo));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 130:
							int_to_char(&dma_tx_buff[string_len],(long)(saludo_inicial));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 131:
							int_to_char(&dma_tx_buff[string_len],(long)(min_posicion + LONG_MIN));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 132:
							int_to_char(&dma_tx_buff[string_len],(long)(max_posicion + LONG_MIN));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 133:
							int_to_char(&dma_tx_buff[string_len],(long)(limit_posicion));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 134:
							int_to_char(&dma_tx_buff[string_len],(long)(t_ramp_t));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 135:
							int_to_char(&dma_tx_buff[string_len],(long)(t_ramp_s));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 136:
							int_to_char(&dma_tx_buff[string_len],(long)(a_ramp_t));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  case 137:
							int_to_char(&dma_tx_buff[string_len],(long)(deadband));
							string_len = strlen(dma_tx_buff);
							dma_tx_buff[string_len]=',';
							string_len++;
						  break;
						  default:
						  break;
					  }
				  }
			  }
			  if (string_len > 0)
			  {
				dma_tx_buff[string_len-1]=10;
				dma_tx_buff[string_len]=13;
			  }
			  if( recibido_USART1 == FALSE)
			  {
				  enviado_USART1 = FALSE;
				  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, strlen(dma_tx_buff));
				  GPIOB->BSRR=LL_GPIO_PIN_5;
				  LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_4);
			  }
			  else if ( envio_si_leo == 2)		//Dejo de enviar el comando de envío actual
			  {
				  comandos_serial.iniciado[LS_TS_ESTADO_SERIAL] = 3;
				  send_RS485 = FALSE;
				  recibido_USART1 = FALSE;
			  }
			  else if ( envio_si_leo == 1)		//Dejo de enviar.
			  {
				  for ( uint8_t i = 0; i < LS_TASK_SERIAL_SIZE; i++)
				  {
					  if(comandos_serial.iniciado[i] == 1 || comandos_serial.iniciado[i] == 0) comandos_serial.iniciado[i] = 3;
				  }
				  send_RS485 = FALSE;
				  if(LS_TS_GUARDADO_SERIAL == 0) LS_TS_ESTADO_SERIAL = LS_TASK_SERIAL_SIZE - 1;
				  else LS_TS_ESTADO_SERIAL = LS_TS_GUARDADO_SERIAL - 1;
			  }
			  else				//Envío de todos modos [NO ACONSEJABLE]
			  {
				  recibido_USART1 = FALSE;
				  enviado_USART1 = FALSE;
				  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, strlen(dma_tx_buff));
				  GPIOB->BSRR=LL_GPIO_PIN_5;
				  LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_4);
			  }
		  }
		  else if (comandos_serial.orden_sensores[LS_TS_ESTADO_SERIAL][0] == 255)
		  {
			  tiempo_RS485 = ticks;

			  comandos_serial.times[LS_TS_ESTADO_SERIAL] = 0;
			  send_RS485 = FALSE;
		  }
		  if( send_RS485 == FALSE )
		  {
			  comandos_serial.iniciado[LS_TS_ESTADO_SERIAL] = 2;
			  LS_TS_ESTADO_SERIAL++;
			  if( LS_TS_ESTADO_SERIAL == LS_TASK_SERIAL_SIZE) LS_TS_ESTADO_SERIAL = 0;
		  }
		}
	}
	else if(send_RS485 == FALSE && comandos_serial.iniciado[LS_TS_ESTADO_SERIAL] == 1)
	{
	  comandos_serial.iniciado[LS_TS_ESTADO_SERIAL] = 2;
	  LS_TS_ESTADO_SERIAL++;
	  if( LS_TS_ESTADO_SERIAL == LS_TASK_SERIAL_SIZE) LS_TS_ESTADO_SERIAL = 0;
	}
}

void ReadSSI()
{
    uint8_t bit,i;

    GPIOA->BRR=LL_GPIO_PIN_0;
    paridad_SSI=0;
    data_SSI_IN=0;
    error_SSI=0;
    for (i=20;i>0;i--)
    {
    	GPIOA->BRR=LL_GPIO_PIN_2;
    	bit=!!(GPIOA -> IDR & (LL_GPIO_PIN_1));

    	data_SSI_IN=data_SSI_IN | (bit<<(i-1));
    	GPIOA->BSRR=LL_GPIO_PIN_2;

    	if(bit==1)paridad_SSI ^= 1;
    }
    GPIOA->BSRR=LL_GPIO_PIN_0;
    error_SSI=(data_SSI_IN & 0x0f);
    data_SSI_IN=data_SSI_IN>>4;
    error_SSI=error_SSI>>1;

}

void get_kalman_gains()
{
	float f_q_corriente = q_corriente;
	float f_q_temp_ext = q_temp_ext;
	float f_q_temp_int = q_temp_int;
	float f_q_volts = q_volts;
	float f_q_encoder = q_encoder;
	f_q_corriente = f_q_corriente/1000;
	f_q_temp_ext = f_q_temp_ext/1000;
	f_q_temp_int = f_q_temp_int/1000;
	f_q_volts = f_q_volts/1000;
	f_q_encoder = f_q_encoder/1000;

	float p_corriente = 100 + f_q_corriente;	//incertidumbre_inicial
	float p_temp_ext = 100 + f_q_temp_ext;		//incertidumbre_inicial
	float p_temp_int = 100 + f_q_temp_int;		//incertidumbre_inicial
	float p_volts = 100 + f_q_volts;			//incertidumbre_inicial
	float p_encoder = 100 + f_q_encoder;		//incertidumbre_inicial

	float f_varianza_corriente = varianza_corriente;
	float f_varianza_temp_ext = varianza_temp_ext;
	float f_varianza_temp_int = varianza_temp_int;
	float f_varianza_volts = varianza_volts;
	float f_varianza_encoder = varianza_encoder;
	f_varianza_corriente = f_varianza_corriente/1000;
	f_varianza_temp_ext = f_varianza_temp_ext/1000;
	f_varianza_temp_int = f_varianza_temp_int/1000;
	f_varianza_volts = f_varianza_volts/1000;
	f_varianza_encoder = f_varianza_encoder/1000;
	{
		uint16_t i_kalman=0;
		while(i_kalman<100)
		{
			k_corriente=p_corriente/(p_corriente+f_varianza_corriente);
			p_corriente=(1-k_corriente)*p_corriente;
			p_corriente=p_corriente+q_corriente;

			k_temp_ext=p_temp_ext/(p_temp_ext+f_varianza_temp_ext);
			p_temp_ext=(1-k_temp_ext)*p_temp_ext;
			p_temp_ext=p_temp_ext+q_temp_ext;

			k_temp_int=p_temp_int/(p_temp_int+f_varianza_temp_int);
			p_temp_int=(1-k_temp_int)*p_temp_int;
			p_temp_int=p_temp_int+q_temp_int;

			k_volts=p_volts/(p_volts+f_varianza_volts);
			p_volts=(1-k_volts)*p_volts;
			p_volts=p_volts+q_volts;

			k_encoder=p_encoder/(p_encoder+f_varianza_encoder);
			p_encoder=(1-k_encoder)*p_encoder;
			p_encoder=p_encoder+q_encoder;

			i_kalman++;
		}
	}
}

void new_temp_task(uint8_t n_comando,long n_param_1,long n_param_2,long n_param_3,long n_param_4,long n_param_5,boolean prioritaria)
{
	temp_task_scheduler.comando[LS_TEMP_TS_GUARDADO] = n_comando;
	temp_task_scheduler.param_1[LS_TEMP_TS_GUARDADO] = n_param_1;
	temp_task_scheduler.param_2[LS_TEMP_TS_GUARDADO] = n_param_2;
	temp_task_scheduler.param_3[LS_TEMP_TS_GUARDADO] = n_param_3;
	temp_task_scheduler.param_4[LS_TEMP_TS_GUARDADO] = n_param_4;
	temp_task_scheduler.param_5[LS_TEMP_TS_GUARDADO] = n_param_5;
	temp_task_scheduler.prioritaria[LS_TEMP_TS_GUARDADO] = prioritaria;
	temp_task_scheduler.time_comando[LS_TEMP_TS_GUARDADO] = 0;
	LS_TEMP_TS_GUARDADO++;
	if(LS_TEMP_TS_GUARDADO == LS_TASK_SCHEDULER_SIZE) LS_TEMP_TS_GUARDADO = LS_TASK_SCHEDULER_SIZE-1;
}
void new_temp_task_time(uint8_t n_comando,long n_param_1,long n_param_2,long n_param_3,long n_param_4,long n_param_5,long n_time_comando, boolean prioritaria)
{
	temp_task_scheduler.comando[LS_TEMP_TS_GUARDADO] = n_comando;
	temp_task_scheduler.param_1[LS_TEMP_TS_GUARDADO] = n_param_1;
	temp_task_scheduler.param_2[LS_TEMP_TS_GUARDADO] = n_param_2;
	temp_task_scheduler.param_3[LS_TEMP_TS_GUARDADO] = n_param_3;
	temp_task_scheduler.param_4[LS_TEMP_TS_GUARDADO] = n_param_4;
	temp_task_scheduler.param_5[LS_TEMP_TS_GUARDADO] = n_param_5;
	temp_task_scheduler.prioritaria[LS_TEMP_TS_GUARDADO] = prioritaria;
	temp_task_scheduler.time_comando[LS_TEMP_TS_GUARDADO] = n_time_comando;
	LS_TEMP_TS_GUARDADO++;
	if(LS_TEMP_TS_GUARDADO == LS_TASK_SCHEDULER_SIZE) LS_TEMP_TS_GUARDADO = LS_TASK_SCHEDULER_SIZE-1;
}
void new_task(uint8_t n_comando,long n_param_1,long n_param_2,long n_param_3,long n_param_4,long n_param_5,boolean prioritaria)
{
	task_scheduler.comando[LS_TS_GUARDADO] = n_comando;
	task_scheduler.param_1[LS_TS_GUARDADO] = n_param_1;
	task_scheduler.param_2[LS_TS_GUARDADO] = n_param_2;
	task_scheduler.param_3[LS_TS_GUARDADO] = n_param_3;
	task_scheduler.param_4[LS_TS_GUARDADO] = n_param_4;
	task_scheduler.param_5[LS_TS_GUARDADO] = n_param_5;
	task_scheduler.time_comando[LS_TS_GUARDADO] = 0;
	LS_TS_GUARDADO++;
	if(LS_TS_GUARDADO == LS_TASK_SCHEDULER_SIZE) LS_TS_GUARDADO = 0;
}
void new_task_time(uint8_t n_comando,long n_param_1,long n_param_2,long n_param_3,long n_param_4,long n_param_5,long n_time_comando,boolean prioritaria)
{
	task_scheduler.comando[LS_TS_GUARDADO] = n_comando;
	task_scheduler.param_1[LS_TS_GUARDADO] = n_param_1;
	task_scheduler.param_2[LS_TS_GUARDADO] = n_param_2;
	task_scheduler.param_3[LS_TS_GUARDADO] = n_param_3;
	task_scheduler.param_4[LS_TS_GUARDADO] = n_param_4;
	task_scheduler.param_5[LS_TS_GUARDADO] = n_param_5;
	task_scheduler.time_comando[LS_TS_GUARDADO] = n_time_comando;
	LS_TS_GUARDADO++;
	if(LS_TS_GUARDADO == LS_TASK_SCHEDULER_SIZE) LS_TS_GUARDADO = 0;
}
void copy_temp_to_task()
{
	uint8_t b;
	if(temp_task_scheduler.prioritaria[0] == TRUE)
	{
		if(task_scheduler.iniciado[LS_TS_GUARDADO] == 1) task_scheduler.iniciado[LS_TS_GUARDADO] = 3;		//Marcar como tarea interrumpida
		LS_TS_GUARDADO = 0;
		LS_TS_ESTADO = 0;
	}
	for(b=0;(b < LS_TEMP_TS_GUARDADO) && (task_scheduler.iniciado[LS_TS_GUARDADO] != 1);b++)
	{
		task_scheduler.comando[LS_TS_GUARDADO] = temp_task_scheduler.comando[b];
		task_scheduler.param_1[LS_TS_GUARDADO] = temp_task_scheduler.param_1[b];
		task_scheduler.param_2[LS_TS_GUARDADO] = temp_task_scheduler.param_2[b];
		task_scheduler.param_3[LS_TS_GUARDADO] = temp_task_scheduler.param_3[b];
		task_scheduler.param_4[LS_TS_GUARDADO] = temp_task_scheduler.param_4[b];
		task_scheduler.param_5[LS_TS_GUARDADO] = temp_task_scheduler.param_5[b];
		task_scheduler.time_comando[LS_TS_GUARDADO] = temp_task_scheduler.time_comando[b];
		task_scheduler.iniciado[LS_TS_GUARDADO] = 0;
		//task_scheduler.time_step[LS_TS_GUARDADO] = 0;
		LS_TS_GUARDADO++;
		if(LS_TS_GUARDADO == LS_TASK_SCHEDULER_SIZE) LS_TS_GUARDADO = 0;
	}
}

void new_temp_serial_task(volatile uint8_t n_orden_sensores[TAM_BUFF_SENSORES],uint8_t num_sensores,long n_param_1,long n_param_2, boolean prioritaria, boolean cabecera)
{
	uint8_t i;
	if(LS_TEMP_TS_GUARDADO_SERIAL == LS_TASK_SERIAL_SIZE) LS_TEMP_TS_GUARDADO_SERIAL = LS_TASK_SERIAL_SIZE - 1;
	for(i = 0;i < num_sensores; i++)
	{
		temp_comandos_serial.orden_sensores[LS_TEMP_TS_GUARDADO_SERIAL][i] = n_orden_sensores[i];
	}
	for(i=i;i < TAM_BUFF_SENSORES; i++)
	{
		temp_comandos_serial.orden_sensores[LS_TEMP_TS_GUARDADO_SERIAL][i] = 255;
	}
	temp_comandos_serial.times[LS_TEMP_TS_GUARDADO_SERIAL] = n_param_1;
	temp_comandos_serial.time[LS_TEMP_TS_GUARDADO_SERIAL] = n_param_2;
	temp_comandos_serial.prioritaria[LS_TEMP_TS_GUARDADO_SERIAL] = prioritaria;
	temp_comandos_serial.cabecera[LS_TEMP_TS_GUARDADO_SERIAL] = cabecera;
	LS_TEMP_TS_GUARDADO_SERIAL++;
}
void copy_temp_to_task_serial()
{
	uint8_t b,i;
	if(temp_comandos_serial.prioritaria[0] == TRUE)
	{
		if(comandos_serial.iniciado[LS_TS_GUARDADO_SERIAL] == 1) comandos_serial.iniciado[LS_TS_GUARDADO_SERIAL] = 3;		//Marcar como tarea interrumpida
		LS_TS_GUARDADO_SERIAL = 0;
		LS_TS_ESTADO_SERIAL = 0;
	}
	for(b=0;(b < LS_TEMP_TS_GUARDADO_SERIAL) && (comandos_serial.iniciado[LS_TS_GUARDADO_SERIAL] != 1);b++)
	{
		for(i = 0;i < TAM_BUFF_SENSORES; i++)
		{
			comandos_serial.orden_sensores[LS_TS_GUARDADO_SERIAL][i] = temp_comandos_serial.orden_sensores[b][i];
			temp_comandos_serial.orden_sensores[b][i] = 255;
		}
		/*if(temp_comandos_serial.times[b] > 0) comandos_serial.times[LS_TS_GUARDADO_SERIAL] = temp_comandos_serial.times[b];
		else comandos_serial.times[LS_TS_GUARDADO_SERIAL] = DEFAULT_TIMES;
		if(temp_comandos_serial.time[b] > 0) comandos_serial.time[LS_TS_GUARDADO_SERIAL] = temp_comandos_serial.time[b];
		else comandos_serial.time[LS_TS_GUARDADO_SERIAL] = DEFAULT_TIME;*/
		comandos_serial.times[LS_TS_GUARDADO_SERIAL] = temp_comandos_serial.times[b];
		comandos_serial.time[LS_TS_GUARDADO_SERIAL] = temp_comandos_serial.time[b];
		comandos_serial.cabecera[LS_TS_GUARDADO_SERIAL] = temp_comandos_serial.cabecera[b];
		comandos_serial.iniciado[LS_TS_GUARDADO_SERIAL] = 0;
		LS_TS_GUARDADO_SERIAL++;
		if(LS_TS_GUARDADO_SERIAL == LS_TASK_SERIAL_SIZE) LS_TS_GUARDADO_SERIAL = 0;
	}
}

void comando_set_value()
{
	long temp_param2;
	temp_param2 = task_scheduler.param_2[LS_TS_ESTADO];
	if (task_scheduler.iniciado[LS_TS_ESTADO] == 0)
	{
		if (temp_param2 >= 0)
		{
			switch (task_scheduler.param_1[LS_TS_ESTADO])
			{
			  case 100:
				if(temp_param2 <= 255)ID_SERVO = temp_param2;
			  break;
			  case 101:
				q_corriente = temp_param2;
				get_kalman_gains();
			  break;
			  case 102:
				q_temp_ext = temp_param2;
				get_kalman_gains();
			  break;
			  case 103:
				q_temp_int = temp_param2;
				get_kalman_gains();
			  break;
			  case 104:
				q_volts = temp_param2;
				get_kalman_gains();
			  break;
			  case 105:
				q_encoder = temp_param2;
				get_kalman_gains();
			  break;
			  case 106:
				varianza_corriente = temp_param2;
				get_kalman_gains();
			  break;
			  case 107:
				varianza_temp_ext = temp_param2;
				get_kalman_gains();
			  break;
			  case 108:
				varianza_temp_int = temp_param2;
				get_kalman_gains();
			  break;
			  case 109:
				varianza_volts = temp_param2;
				get_kalman_gains();
			  break;
			  case 110:
				varianza_encoder = temp_param2;
				get_kalman_gains();
			  break;
			  case 111:
				vel_serie = temp_param2;
				change_USART1_speed();
			  break;
			  case 112:
				corte_temp_ext = temp_param2;
			  break;
			  case 113:
				corte_temp_int = temp_param2;
			  break;
			  case 114:
				corte_volts_alto = temp_param2;
			  break;
			  case 115:
				corte_volts_bajo = temp_param2;
			  break;
			  case 116:
				corte_corriente = temp_param2;
			  break;
			  case 117:
				if( temp_param2 < 3 ) uso_crc = temp_param2;
			  break;
			  case 118:
				CRC_START_CCITT = temp_param2;
			  break;
			  case 119:
				CRC_POLY_CCITT = temp_param2;
				init_crcccitt_tab();
			  break;
			  case 120:
				K_P_LS = temp_param2;
				K_P_LS_f = (float)K_P_LS/100;
			  break;
			  case 121:
				K_D_LS = temp_param2;
				K_D_LS_f = (float)K_D_LS/100;
			  break;
			  case 122:
				K_I_LS = temp_param2;
				K_I_LS_f = (float)K_I_LS/10000;
			  break;
			  case 123:
				K_P_M = temp_param2;
				K_P_M_f = (float)K_P_M/100;
			  break;
			  case 124:
				K_D_M = temp_param2;
				K_D_M_f = (float)K_D_M/100;
			  break;
			  case 125:
				K_I_M = temp_param2;
				K_I_M_f = (float)K_I_M/10000;
			  break;
			  case 126:
				if(temp_param2 <= 1000) offset_corriente = temp_param2 / 9;
			  break;
			  case 127:
				if(temp_param2 <= 150) offset_temp_int = temp_param2 / 10;
			  break;
			  case 128:
				if(temp_param2 <= 150) offset_temp_ext = temp_param2 / 10;
			  break;
			  case 129:
				if( temp_param2 < 3 ) envio_si_leo = temp_param2;
			  break;
			  case 130:
				if( temp_param2 < 2 ) saludo_inicial = temp_param2;
			  break;
			  case 131:
				min_posicion = temp_param2;
			  break;
			  case 132:
				max_posicion = temp_param2;
			  break;
			  case 133:
				if( temp_param2 < 2 ) limit_posicion = temp_param2;
			  break;
			  case 134:
				t_ramp_t = temp_param2;
			  break;
			  case 135:
				t_ramp_s = temp_param2;
			  break;
			  case 136:
				a_ramp_t = temp_param2;
			  break;
			  case 137:
				if(temp_param2 <= 1000) deadband = temp_param2;
			  break;
			  default:
			  break;
			}
		}
		else
		{
			switch (task_scheduler.param_1[LS_TS_ESTADO])
			{
			  case 126:
				if(temp_param2 >= -1000) offset_corriente = temp_param2 / 9;
			  break;
			  case 127:
				if(temp_param2 >= -150) offset_temp_int = temp_param2 / 10;
			  break;
			  case 128:
				if(temp_param2 >= -150) offset_temp_ext = temp_param2 / 10;
			  break;
			  case 131:
				min_posicion = temp_param2;
			  break;
			  case 132:
				max_posicion = temp_param2;
			  break;
			  default:
			  break;
			}
		}
	}
	task_scheduler.iniciado[LS_TS_ESTADO] = 2;
}

/*
 * static void init_crcccitt_tab( void );
 *
 * For optimal performance, the routine to calculate the CRC-CCITT uses a
 * lookup table with pre-compiled values that can be directly applied in the
 * XOR action. This table is created at the first call of the function by the
 * init_crcccitt_tab() routine.
 * CHECK WITH: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html OR https://crccalc.com/
 * Default parametrization in LS: CRC-16/AUG-CCITT
 */

void init_crcccitt_tab()
{
	uint16_t i;
	uint16_t j;
	uint16_t crc;
	uint16_t c;

	for (i=0; i<256; i++)
	{
		crc = 0;
		c   = i << 8;

		for (j=0; j<8; j++)
		{
			if ( (crc ^ c) & 0x8000 ) crc = ( crc << 1 ) ^ CRC_POLY_CCITT;
			else                      crc =   crc << 1;

			c = c << 1;
		}

		crc_tabccitt[i] = crc;
	}
}  /* init_crcccitt_tab */

/*
 * uint16_t update_crc_ccitt( uint16_t crc, unsigned char c );
 *
 * The function update_crc_ccitt() calculates a new CRC-CCITT value based on
 * the previous value of the CRC and the next byte of the data to be checked.
 * CHECK WITH: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html OR https://crccalc.com/
 * Default parametrization in LS: CRC-16/AUG-CCITT
 */
uint16_t update_crc_ccitt( uint16_t crc, unsigned char c )
{
	return ((crc << 8) ^ crc_tabccitt[ ((crc >> 8) ^ (uint16_t) c) & 0x00FF ]);

}  /* update_crc_ccitt */
