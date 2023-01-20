/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : stm32f3xx_it.c
  * @brief          : Interrupt Service Routines
  * @author			: Luis Picó Chausson
  * @version		: 0.1
  * Created on: 24 jun. 2018
  ******************************************************************************
  * @attention
  *
  * LibreServo by Luis Picó is licensed under a Creative Commons 
  * Attribution-NonCommercial-ShareAlike 4.0 International License
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void tratar_rx();
void tratar_caracter_serial(unsigned char c);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern volatile boolean LED_modo_RAINBOW;				//Inicializado en LS_funciones.c
extern volatile int brillado_led_rgb;
extern volatile int estado_led;
extern volatile uint8_t new_led_rgb;

struct s_led_rgb {uint32_t pin[3];uint8_t brillo[3];};
extern volatile struct s_led_rgb led_rgb;
extern volatile struct s_led_rgb led_rgb_temp;

extern volatile boolean status_tone;			//Inicializado en main.c

extern volatile uint8_t enviado_USART1;
extern volatile uint8_t recibido_USART1;

extern int pos_buff_rx;
extern char dma_tx_buff[DMA_TX_BUFF_SIZE];
extern char dma_rx_buff[DMA_RX_BUFF_SIZE];
static int bytes_rx = 0;
static char serial_buff[SERIAL_BUFF_SIZE] = { 0 };

extern volatile uint16_t dma_adc1_buff[TAM_DMA_ADC1_BUFF];		//Inicializado en main.c

struct LS_comandos_serial {uint16_t orden_sensores[LS_TASK_SERIAL_SIZE][TAM_BUFF_SENSORES]; long times[LS_TASK_SERIAL_SIZE]; long time[LS_TASK_SERIAL_SIZE];uint8_t iniciado[LS_TASK_SERIAL_SIZE];boolean cabecera[LS_TASK_SERIAL_SIZE];};
extern volatile struct LS_comandos_serial comandos_serial;			//Inicializado en LS_funciones.c
extern volatile uint8_t LS_TS_ESTADO_SERIAL, LS_TS_GUARDADO_SERIAL;	//Inicializado en LS_funciones.c

extern volatile float corriente_raw;							//Inicializado en LS_funciones.c
extern volatile float corriente_kalman;							//Inicializado en LS_funciones.c
extern volatile float k_corriente;
extern volatile int offset_corriente;
extern volatile int corriente;									//Inicializado en LS_funciones.c

extern volatile float k_encoder;
extern volatile uint32_t data_SSI_IN;
extern volatile uint32_t data_SSI_IN_old;
extern volatile long data_SSI_IN_internal;
extern volatile long data_SSI_IN_vuelta;

//extern volatile int correccion_enc[257];				//Inicializado en main.c

volatile float grados_encoder_kalman = 0.0;
volatile float grados_encoder_kalman_antes = 0.0;
volatile float velocidad_antes = 0.0, velocidad_antes_int = 0.0, pos_destino_f_antes = 0.0;

extern volatile uint32_t ticks;							//Inicializado en funciones.c
uint32_t tiempo_temp;									//Tiempo entre lecturas temperaturas

extern volatile uint16_t ID_SERVO;

static uint16_t estado_serial_LS = 1;
static uint16_t estado_serial_LS_volver;
static uint8_t estado_serial_num;

static uint8_t serial_LS_ID, serial_LS_ID_fin;
static boolean serial_sub_comando_yo;
static long serial_LS_param_1, serial_LS_param_2, serial_LS_param_3, serial_LS_param_4, serial_LS_param_5;
static long serial_num_temp;

struct LS_temp_task_scheduler {uint8_t comando[LS_TASK_SCHEDULER_SIZE];long param_1[LS_TASK_SCHEDULER_SIZE];long param_2[LS_TASK_SCHEDULER_SIZE];long time_comando[LS_TASK_SCHEDULER_SIZE];boolean prioritaria[LS_TASK_SCHEDULER_SIZE];};
extern volatile struct LS_temp_task_scheduler temp_task_scheduler;
struct LS_task_scheduler {uint8_t comando[LS_TASK_SCHEDULER_SIZE];long param_1[LS_TASK_SCHEDULER_SIZE];long param_2[LS_TASK_SCHEDULER_SIZE];long param_3[LS_TASK_SCHEDULER_SIZE];long param_4[LS_TASK_SCHEDULER_SIZE];long param_5[LS_TASK_SCHEDULER_SIZE];long time_comando[LS_TASK_SCHEDULER_SIZE];uint8_t iniciado[LS_TASK_SCHEDULER_SIZE];};
extern volatile struct LS_task_scheduler task_scheduler;
extern volatile uint8_t LS_TS_ESTADO, LS_TS_GUARDADO, LS_TEMP_TS_GUARDADO;
struct LS_comando_motor {uint8_t comando; long pos_now; float velocidad; float aceleracion; float velocidad_int; float aceleracion_int; long pos_destino; float pos_destino_f; long pos_ini; long pos_final; long time_to_end; float step; long time_ramp1; long time_ramp2; float step_ramp1; float step_ramp2; int output_LS;};
extern volatile struct LS_comando_motor comando_motor;
static boolean tratando_serie = FALSE;

extern volatile uint16_t uso_crc;
extern volatile uint16_t CRC_START_CCITT;

static uint16_t crc;
static uint16_t serial_crc;

extern uint32_t vel_serie;					//Inicializado en main.c

static uint16_t comando_serial = 0;
/*
comando_serial:	1	->	Move
				10	->	Get
				11	->	Get Sensors
				12 	->	Get Sensors Without header
				15	->	Set
*/
static boolean serial_comando_imp = FALSE;
////extern volatile uint8_t orden_sensores[TAM_BUFF_SENSORES];
extern volatile uint16_t temp_orden_sensores[TAM_BUFF_SENSORES];		//Inicializado en LS_funciones.c
static uint8_t i_temp_orden_sensores = 0;

extern volatile boolean send_RS485;
extern volatile uint8_t serial_RS485_datos_enviar;
extern uint32_t tiempo_RS485; 					//inicializado en LS_funciones.c
extern boolean enviado_cabecera;				//inicializado en LS_funciones.c
extern long buff_sensores[TAM_BUFF_SENSORES];	//inicializado en main.c

static boolean unique_id = FALSE;
static uint8_t PID_MOTOR = 0;

//struct LS_comandos_serial {uint8_t orden_sensores[LS_TASK_SERIAL_SIZE][TAM_BUFF_SENSORES]; long times[LS_TASK_SERIAL_SIZE]; long time[LS_TASK_SERIAL_SIZE];uint8_t iniciado[LS_TASK_SERIAL_SIZE]; boolean cabecera[LS_TASK_SERIAL_SIZE];};
//extern volatile struct LS_comandos_serial comandos_serial;			//Inicializado en LS_funciones.
extern volatile uint8_t LS_TEMP_TS_GUARDADO_SERIAL;
extern volatile uint8_t bat_estado;						//Inicializado en LS_funciones.c Estado batería 0=Good, 1=LOW, 2=HIGH
extern volatile uint8_t temp_estado;					//Inicializado en LS_funciones.c Estado temperature 0=Good, 1=HIGH_external, 2=HIGH_internal, 3=HIGH_BOTH
static boolean deshab_fan3227 = FALSE;
extern volatile boolean estado_FD;						//Inicializado en main.c

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	ticks++;
	read_bat();
	
	if(ticks - tiempo_temp >= 5)
	{
		tiempo_temp = ticks;
		read_temps();
	}
	
	if(deshab_fan3227 == TRUE && bat_estado == 0 && temp_estado == 0)
	{
		GPIOA->BSRR=LL_GPIO_PIN_10;  			//Habilitar el fan3227 (SET_PIN)
		deshab_fan3227 = FALSE;
	}
	else if(bat_estado != 0 || temp_estado != 0)
	{
		GPIOA->BRR=LL_GPIO_PIN_10;  			//Deshabilitar el fan3227 (CLEAR_PIN)
		deshab_fan3227 = TRUE;
	}
	
	if (comandos_serial.iniciado[LS_TS_ESTADO_SERIAL] == 0)
	{
		send_RS485 = TRUE;
		recibido_USART1 = FALSE;
		comandos_serial.iniciado[LS_TS_ESTADO_SERIAL] = 1;
		enviado_cabecera = FALSE;
		if (comandos_serial.orden_sensores[LS_TS_ESTADO_SERIAL][0] == 255) tiempo_RS485 = ticks;			//Si es un comando de WAIT, "pongo a cero" el contador de tiempo
	}
	comando_get();
  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */
	if(LL_DMA_IsActiveFlag_TC4(DMA1))
	{
		LL_DMA_ClearFlag_TC4(DMA1);
	}
	enviado_USART1 = TRUE;
	LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_4);
  /* USER CODE END DMA1_Channel4_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */
	if ( LL_DMA_IsActiveFlag_HT5(DMA1) )
	{
		LL_DMA_ClearFlag_HT5(DMA1);

		tratar_rx();

	}
	if ( LL_DMA_IsActiveFlag_TC5(DMA1) )
	{
		LL_DMA_ClearFlag_TC5(DMA1);

		tratar_rx();

	}

  /* USER CODE END DMA1_Channel5_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break and TIM15 interrupts.
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */
	//float t_float, t_float2, t_float3;
	//uint8_t t_uint8;
	//long t_long;
	if(LL_TIM_IsActiveFlag_UPDATE(TIM15) == TRUE)
	{
		LL_TIM_ClearFlag_UPDATE(TIM15);
		corriente_raw = ((long)(dma_adc1_buff[2]+dma_adc1_buff[6]+dma_adc1_buff[10]+dma_adc1_buff[14]+dma_adc1_buff[18]+dma_adc1_buff[22]+dma_adc1_buff[26]+dma_adc1_buff[30])/8)-(2048-offset_corriente);
		corriente_kalman = corriente_kalman + k_corriente*(corriente_raw-corriente_kalman);
		
		corriente = (corriente_kalman)*15500/1731;			//15.5A 90mv/A
		if(corriente < 0) corriente = 0;
		
		/************************************************************
		LL_TIM_DisableIT_UPDATE(TIM15);
		comando_motor.output_LS = 300;
		PID_MOTOR = 0;
		if(ReadSSI() == 1)
		{
			int correction_enc [512];
			uint16_t index = data_SSI_IN/128;
			data_SSI_IN = data_SSI_IN + correction_enc[index];
			
			leer_enc, calcular velocidad y meter en pre_correccion_enc[512]
			velocidad_correccion = data_SSI_IN - pre_data_SSI_IN;
			uint16_t index_pre = data_SSI_IN/128;
			if(pre_correccion_enc[index_pre] == 0)pre_correccion_enc[index_pre] = velocidad_correccion;
			else pre_correccion_enc[index_pre] = pre_correccion_enc[index_pre] * 0.5 + velocidad_correccion * 0.5;
		}
		
		
		************************************************************/
		/*ReadSSI();
		if(data_SSI_IN<1024 && (data_SSI_IN_old > 65000)) data_SSI_IN_vuelta++;
		else if(data_SSI_IN>65000 && data_SSI_IN_old<1024) data_SSI_IN_vuelta--;

		data_SSI_IN_old = data_SSI_IN;
		
		data_SSI_IN_internal = (long)(data_SSI_IN) + data_SSI_IN_vuelta*65536;
		grados_encoder_kalman = grados_encoder_kalman+k_encoder*((float)(data_SSI_IN_internal)-grados_encoder_kalman);*/
		/*if(task_scheduler.comando[LS_TS_ESTADO] != 65)
		{
			t_float3 = data_SSI_IN/256.0;
			t_uint8 = t_float3;
			t_float = correccion_enc[t_uint8];
			t_uint8++;
			t_float2 = correccion_enc[t_uint8];
			t_long = t_float3;
			t_float3 = t_float3 - t_long;
			t_float = t_float*(1-t_float3) + t_float2*t_float3;
			t_float = t_float/4;

			grados_encoder_kalman = grados_encoder_kalman + t_float;
		}*/
		
		comando_motor.velocidad = comando_motor.velocidad*0.9+(grados_encoder_kalman - grados_encoder_kalman_antes)*0.1;
		comando_motor.aceleracion = comando_motor.velocidad - velocidad_antes;
		velocidad_antes = comando_motor.velocidad;
		grados_encoder_kalman_antes = grados_encoder_kalman;
		
		comando_motor.velocidad_int = comando_motor.pos_destino_f - pos_destino_f_antes;
		comando_motor.aceleracion_int = comando_motor.velocidad_int - velocidad_antes_int;
		pos_destino_f_antes = comando_motor.pos_destino_f;
		velocidad_antes_int = comando_motor.velocidad_int;

		if (grados_encoder_kalman >= 0) comando_motor.pos_now = grados_encoder_kalman + 0.5f;
		else comando_motor.pos_now = grados_encoder_kalman - 0.5f;

		//comando_motor.pos_now = 0;

		if( PID_MOTOR == 1 ) PID_LS();
		else if( PID_MOTOR == 2 ) PID_M();
		else not_PID();

	}
  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */

  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update and TIM16 interrupts.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM16) == TRUE)
	{
		LL_TIM_ClearFlag_UPDATE(TIM16);
		if(task_scheduler.iniciado[LS_TS_ESTADO] <= 1)
		{
			switch(task_scheduler.comando[LS_TS_ESTADO])
			{
				case 1:							//Movimiento lineal						S12M12000[:1000]
					comando_M();
					PID_MOTOR = 1;
					break;
				case 2:							//Rampa trapezoidal						S12MT12000[:150]:1000;
					comando_MT();
					PID_MOTOR = 1;
					break;
				case 3:							//Rampa trapezoidal						S12Mt12000:1000;
					comando_MT();
					PID_MOTOR = 1;
					break;
				case 4:							//Rampa Senoidal						S12MS12000[:150]:1000;
					comando_MS();
					PID_MOTOR = 1;
					break;
				case 5:							//Rampa Senoidal						S12Ms12000:1000;
					comando_MS();
					PID_MOTOR = 1;
					break;
				case 6:							//Rampa Hermítica						S12MH1300:0:-12000:-300:4000;	S12MH1300:-12000:4000;	S12MH-12000:4000;
					comando_MH();
					PID_MOTOR = 1;
					break;
				case 7:							//Rampa trapezoidal Acel.				S12MA12000[:150]:1000;
					comando_MA();
					PID_MOTOR = 1;
					break;
				case 15:						//Cambia un valor interno				S12S111:9600; Cambia la velocidad del puerto serie a 9600
					comando_set_value();
					break;
				case 16:						//Guarda configuración actual a Flash	S12SV;
					save_new_values_flash();
					break;
				case 20:						//reset un parámetro (a última vez arrancado)				S12RV105;
					read_init_value_flash();
					break;
				case 21:						//reset un parámetro (a valor original)						S12Rv105;
					reset_value_flash();
					break;
				case 22:						//reset todo el servomotor (a valor última vez arrancado)	S12RS;
					read_init_flash();
					get_kalman_gains();
					change_USART1_speed();
					break;
				case 23:						//reset todo el servomotor (a valor original)				S12Rs;
					reset_flash();
					get_kalman_gains();
					change_USART1_speed();
					break;
				case 30:						//Movimiento Contínuo					S12MC2800[:1000];
					comando_M();
					PID_MOTOR = 2;
					break;
				case 31:						//Movimiento Contínuo trapezoidal		S12MCT2800[:500]:1500
					comando_MT();
					PID_MOTOR = 2;
					break;
				case 32:						//Movimiento Contínuo senoidal			S12MCT2800[:500]:1500
					comando_MS();
					PID_MOTOR = 2;
					break;
				case 33:						//Movimiento Contínuo Hermítico			S12MCH2800:4000;	S12MCH2800:0:5000:0:4000;
					comando_MH();
					PID_MOTOR = 2;
					break;
				case 36:						//FindDirection							S12FD;
					if(task_scheduler.iniciado[LS_TS_ESTADO] == 0)
					{
						estado_FD = TRUE;
						PID_MOTOR = 0;
					}
					break;
				case 37:						//Motor PWM								S12MP-500;		-900 <-> 900
					comando_MP();
					PID_MOTOR = 0;
					break;
				case 38:						//Motor Wait							S12MW1000;
					comando_MW();
					break;
				case 39:						//Movimiento libre						S12MF;
					comando_MF();
					PID_MOTOR = 0;
					break;
				case 40:
					comando_LED_RGB();			//Encender el LED RGB según comando
					break;
				case 41:
					comando_LED_RAINBOW();		//Encender el LED RGB en modo arcoíris
					break;
				case 60:						//Tone
					if(task_scheduler.iniciado[LS_TS_ESTADO] == 0) status_tone = TRUE;
					break;
				/*case 65:						//Calibrar el Encoder
					comando_CE();
					PID_MOTOR = 0;
					break;*/
				default:
					break;
			}
		}
		else if(task_scheduler.iniciado[LS_TS_ESTADO] == 2)
		{
			task_scheduler.iniciado[LS_TS_ESTADO] = 5;
			LS_TS_ESTADO++;

			if (LS_TS_ESTADO == LS_TASK_SCHEDULER_SIZE) LS_TS_ESTADO = 0;
		}
	}

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger, commutation and TIM17 interrupts.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM17)==TRUE)
	{
		LL_TIM_ClearFlag_UPDATE(TIM17);

		if(estado_led==2||brillado_led_rgb>=255)
		{
			GPIOA->BSRR=LED_GREEN;			//Set_pin
			GPIOA->BSRR=LED_RED;			//Set_pin
			GPIOA->BSRR=LED_BLUE;			//Set_pin
			if(brillado_led_rgb>=255)
			{
				estado_led=0;
				brillado_led_rgb=0;

				if(LED_modo_RAINBOW) RAINBOW();

				if(new_led_rgb==TRUE)							//Copy the new LED configuration
				{
					new_led_rgb=FALSE;
					for(int i_a=0;i_a<3;i_a++)
					{
						led_rgb.pin[i_a]=led_rgb_temp.pin[i_a];
						led_rgb.brillo[i_a]=led_rgb_temp.brillo[i_a];
					}
				}
			}
			else
			{
				TIM17->CNT=0+brillado_led_rgb;
				brillado_led_rgb=255;
			}
		}
		else
		{
			GPIOA->BSRR=led_rgb.pin[estado_led];			//Set_pin
			estado_led++;
		}

		if(brillado_led_rgb<255)
		{
			if(estado_led==0)
			{
				if(led_rgb.brillo[0]>0)
				{
					GPIOA->BRR=LED_GREEN;				//Clear_pin
					GPIOA->BRR=LED_RED;					//Clear_pin
					GPIOA->BRR=LED_BLUE;				//Clear_pin
				}
				else if(led_rgb.brillo[1]>0)
				{
					estado_led=1;
					GPIOA->BRR=led_rgb.pin[1];			//Clear_pin
					GPIOA->BRR=led_rgb.pin[2];			//Clear_pin
				}
				else if(led_rgb.brillo[2]>0)
				{
					estado_led=2;
					GPIOA->BRR=led_rgb.pin[2];			//Clear_pin
				}
				else
				{
					estado_led=2;
				}
			}

			brillado_led_rgb=brillado_led_rgb+led_rgb.brillo[estado_led];
			if(estado_led==1)
			{
				if(led_rgb.brillo[estado_led]==0)
				{
					GPIOA->BSRR=led_rgb.pin[estado_led];
					estado_led=2;
				}
			}

			if(led_rgb.brillo[estado_led]>0)
			{
				TIM17->CNT=255-led_rgb.brillo[estado_led];
			}
			else
			{
				estado_led=2;
				TIM17->CNT=0+brillado_led_rgb;
				brillado_led_rgb=255;
				GPIOA->BSRR=led_rgb.pin[estado_led];
			}
		}
	}
  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	/*float t_float, t_float2, t_float3;
	uint8_t t_uint8;
	long t_long;*/

	if(LL_TIM_IsActiveFlag_UPDATE(TIM2)==TRUE)
	{
		LL_TIM_ClearFlag_UPDATE(TIM2);
		if(ReadSSI() == 0)
		{
			/*if(task_scheduler.comando[LS_TS_ESTADO] != 65)
			{
				t_float3 = data_SSI_IN/256.0;
				t_uint8 = t_float3;
				t_float = correccion_enc[t_uint8];
				t_uint8++;
				t_float2 = correccion_enc[t_uint8];
				t_long = t_float3;
				t_float3 = t_float3 - t_long;
				t_float = t_float*(1-t_float3) + t_float2*t_float3;
				t_float = t_float/4;

				if(t_float >=0) t_float = t_float + 0.5;
				else t_float = t_float - 0.5;
				t_long = data_SSI_IN;
				t_long = t_long + t_float;
				if ( t_long < 0 ) data_SSI_IN = 65536 + t_long;
				else data_SSI_IN = t_long;
				//grados_encoder_kalman = grados_encoder_kalman + t_float;
			}*/

			if(data_SSI_IN<1024 && (data_SSI_IN_old > 64511)) data_SSI_IN_vuelta++;
			else if(data_SSI_IN>64511 && data_SSI_IN_old<1024) data_SSI_IN_vuelta--;

			data_SSI_IN_old = data_SSI_IN;
			
			data_SSI_IN_internal = (long)(data_SSI_IN) + data_SSI_IN_vuelta*65536;
			grados_encoder_kalman = grados_encoder_kalman+k_encoder*((float)(data_SSI_IN_internal)-grados_encoder_kalman);
		}
	}

  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if ( LL_USART_IsActiveFlag_TC(USART1) )
	{
		LL_USART_ClearFlag_TC(USART1);
		GPIOB->BRR=LL_GPIO_PIN_5;
	}
	if ( LL_USART_IsActiveFlag_IDLE(USART1) )
	{
		LL_USART_ClearFlag_IDLE(USART1);

		tratar_rx();
	}

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void tratar_rx()
{
	int new_pos;
	new_pos = DMA_RX_BUFF_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);		//Nos da bytes leídos, si se leen 2, sería tamaño buff (x ej. 10) menos 2 = 8 la vuelta de LL_DMA_GetDataLength
	if(new_pos>pos_buff_rx)
	{
		for(int i = pos_buff_rx; i<new_pos; i++)
		{
			tratar_caracter_serial(dma_rx_buff[i]);
			serial_buff[bytes_rx] = dma_rx_buff[i];
			bytes_rx++;
			if(bytes_rx>(SERIAL_BUFF_SIZE-1))bytes_rx = 0;
		}
	}
	else if(new_pos<pos_buff_rx)
	{
		for(int i = pos_buff_rx; i<DMA_RX_BUFF_SIZE; i++)
		{
			tratar_caracter_serial(dma_rx_buff[i]);
			serial_buff[bytes_rx] = dma_rx_buff[i];
			bytes_rx++;
			if(bytes_rx>(SERIAL_BUFF_SIZE-1))bytes_rx = 0;
		}
		for(int i = 0; i<new_pos; i++)
		{
			tratar_caracter_serial(dma_rx_buff[i]);
			serial_buff[bytes_rx] = dma_rx_buff[i];
			bytes_rx++;
			if(bytes_rx>(SERIAL_BUFF_SIZE-1))bytes_rx = 0;
		}
	}
	pos_buff_rx = new_pos;
}

void tratar_caracter_serial(unsigned char c)
{
	if (c != 10 && c != 13) recibido_USART1 = TRUE;
	if(uso_crc != 0)
	{
		if (tratando_serie == FALSE && c == 'S')
		{
			tratando_serie = TRUE;
			crc = CRC_START_CCITT;
			crc = update_crc_ccitt( crc, c );
			LS_TEMP_TS_GUARDADO = 0;
			estado_serial_num = 0;
			estado_serial_LS = 1;
			LS_TEMP_TS_GUARDADO_SERIAL = 0;
		}
		else if (tratando_serie == TRUE && c != '#')
		{
			crc = update_crc_ccitt( crc, c );
		}
	}
	else
	{
		if (tratando_serie == FALSE && c == 'S')
		{
			tratando_serie = TRUE;
			LS_TEMP_TS_GUARDADO = 0;
			estado_serial_num = 0;
			estado_serial_LS = 1;
			LS_TEMP_TS_GUARDADO_SERIAL = 0;
		}
	}
	switch(estado_serial_num)
	{
		case 1:
			if((c >= '0') && (c <= '9'))
			{
				serial_num_temp = c - 48;
				estado_serial_num++;
			}
			else if(c == '-')
			{
				estado_serial_num = 5;
				serial_num_temp = 0;
			}
			else
			{
				estado_serial_num = 0;
				estado_serial_LS = estado_serial_LS_volver;
				serial_num_temp = LONG_MIN;  //Menor número posible **ERROR**
			}
			break;
		case 2:
			if((c >= '0') && (c <= '9'))
			{
				serial_num_temp = serial_num_temp*10 + c - 48;
			}
			else
			{
				estado_serial_num = 0;
				estado_serial_LS = estado_serial_LS_volver;
			}
			break;
		case 5:
			if((c >= '0') && (c <= '9'))
			{
				serial_num_temp = 0 - (c - 48);
				estado_serial_num++;
			}
			else
			{
				estado_serial_num = 0;
				estado_serial_LS = estado_serial_LS_volver;
				serial_num_temp = LONG_MIN;  //Menor número posible **ERROR**
			}
			break;
		case 6:
			if((c >= '0') && (c <= '9'))
			{
				serial_num_temp = serial_num_temp*10 - (c - 48);
			}
			else
			{
				estado_serial_num = 0;
				estado_serial_LS = estado_serial_LS_volver;
			}
			break;
		case 10:
			if((c >= '0') && (c <= '9'))
			{
				serial_crc = (serial_crc<<4) + c-48;
			}
			else if ((c >= 'A') && (c <= 'F'))
			{
				serial_crc = (serial_crc<<4) + c-55;
			}
			else
			{
				estado_serial_num = 0;
				estado_serial_LS = estado_serial_LS_volver;
			}
			break;
		case 15:
			if((c >= '0') && (c <= '9'))
			{
				serial_num_temp = (serial_num_temp<<4) + (c-48);
			}
			else if ((c >= 'A') && (c <= 'F'))
			{
				serial_num_temp = (serial_num_temp<<4) + (c-55);
			}
			else
			{
				estado_serial_num = 0;
				estado_serial_LS = estado_serial_LS_volver;
			}
			break;
	}
	
	
	/**********************************
	Mover [M]-> 5 {1}				//Movimiento lineal						S12M12000[:1000]
	Mover [MT]-> 10 {2}				//Rampa trapezoidal						S12MT12000[:150]:1000;
	Mover [Mt]-> 15 {3}				//Rampa trapezoidal						S12Mt12000:1000;
	Mover [MS]-> 10 {4}				//Rampa Senoidal						S12MS12000[:150]:1000;
	Mover [Ms]-> 15 {5}				//Rampa Senoidal						S12Ms12000:1000;
	Mover [MH]-> 20 {6}				//Rampa Hermítica						S12MH1300:0:-12000:-300:4000;	S12MH1300:-12000:4000;	S12MH-12000:4000;
	Mover [MA]-> 25 {7}				//Rampa trapezoidal Acel.				S12MA12000[:150]:1000;
	Mover [MC] -> 30 {30}			//Movimiento Contínuo					S12MC2800[:1000];
	Mover [MCT] -> 30 {31}			//Movimiento Contínuo trapezoidal		S12MCT2800[:500]:1500
	Mover [MCS] -> 30 {32}			//Movimiento Contínuo senoidal			S12MCT2800[:500]:1500
	Mover [MCH] -> 30 {33}			//Movimiento Contínuo Hermítico			S12MCH2800:4000;	S12MCH2800:0:5000:0:4000;
	
	Mover [MP] -> 5 {37}			//Mover PWM								S12MP500;	S12MP500:1500;			-900 <-> 900
	
	Motor Free [MF] -> 5 {39}		//Motor libre							S12MF;
	
	Motor Wait [MW] -> 5 {38}		//Espera								S12MW1000;

	FindPolarity [FP] -> 45 {36}	//Encuentra la dirección del motor		S12FP;

	Get [G] -> 50 {10}				//Devolver valor						S12G5;
	GetSeveral [GS] -> 51 {11}		//Devuelve una o varias variables cada X ms Y veces 				S12GS5,7,8,100,102:10:500;	Devuelve 5 valores 10 veces cada 500ms
	GetSeveral [Gs] -> 51 {12}		//Devuelve una o varias variables con cabecera (arduino plotter)	S12Gs5,7,8,100,102:10:500;	Devuelve 5 valores 10 veces cada 500ms
	GetWait [GW] -> 51 {14}			//Espera								S12GW1000;
	
	Set [S] -> 60 {15}				//Cambia un valor interno				S12S111:9600; Cambia la velocidad del puerto serie a 9600
	Save [SV] -> 140 {16}			//Guarda configuración actual a Flash	S12SV;
	Reset [RV] -> 150 {20} 			//un parámetro (a última vez arrancado)					S12RV105;
	Reset [Rv] -> 150 {21} 			//un parámetro (a valor original)						S12Rv105;
	ResetServo [RS]-> 150 {22} 		//Todo el servomotor (a valor última vez arrancado)		S12RS;
	ResetServo [Rs]-> 150 {23} 		//Todo el servomotor (a valor original)					S12Rs;

	LED RGB [L] -> 70 {40}			//Color RGB								S12L150:20:100;
	LED Rainbow [LR] -> 70 {41}		//Color Rainbow 						S12LR;
	Tone [T] -> 69 {60}				//Intentar crear un tono				S12T2093:200:50;  Nota 2093Hz [DO6], 200ms, 50% de amplitud
	
	CalibrateEncoder [CE]-> 80 {65}	//Calibrar Encoder

	**********************************/
	switch(estado_serial_LS)  //S1-25,40,150-100M1500:1000!|S5,2,3M10000:3000|S1M1000!#1415;
	{
		case 1:			//SXXX;
			if(c == 'S')
			{
				serial_LS_ID = 0;
				serial_sub_comando_yo = FALSE;
				estado_serial_LS = 0;
				estado_serial_num = 1;
				estado_serial_LS_volver = 2;
				comando_serial = 0;
				serial_LS_param_1 = LONG_MIN;
				serial_LS_param_2 = LONG_MIN;
				serial_LS_param_3 = LONG_MIN;
				serial_LS_param_4 = LONG_MIN;
				serial_LS_param_5 = LONG_MIN;
				serial_comando_imp = FALSE;
				unique_id = TRUE;
			}
			else tratando_serie = FALSE;
			break;
		case 2:			//S12?XX;
			if((serial_num_temp < 256) && (serial_num_temp >= 0))
			{
				serial_LS_ID = serial_num_temp;
				if(c == '-')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 3;
					unique_id = FALSE;
				}
				else if(c == ',')
				{
					if(serial_LS_ID == ID_SERVO || serial_LS_ID == 0)
					{
						serial_sub_comando_yo = TRUE;
					}
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 2;
					unique_id = FALSE;
				}
				else if(c == 'M')
				{
					if(serial_LS_ID == ID_SERVO || serial_LS_ID == 0)
					{
						serial_sub_comando_yo = TRUE;
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 5;
					comando_serial = 1;
				}
				else if(c == 'G' && unique_id == TRUE)
				{
					if(serial_LS_ID == ID_SERVO)
					{
						serial_sub_comando_yo = TRUE;
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 50;
					comando_serial = 10;
				}
				else if(c == 'S')
				{
					if(serial_LS_ID == ID_SERVO)
					{
						serial_sub_comando_yo = TRUE;
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 60;
					comando_serial = 15;
				}
				else if(c == 'R')
				{
					if(serial_LS_ID == ID_SERVO)
					{
						serial_sub_comando_yo = TRUE;
					}

					estado_serial_LS = 150;
				}
				else if(c == 'T')
				{
					if(serial_LS_ID == ID_SERVO)
					{
						serial_sub_comando_yo = TRUE;
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 69;
					comando_serial = 60;
				}
				else if(c == 'L')
				{
					if(serial_LS_ID == ID_SERVO)
					{
						serial_sub_comando_yo = TRUE;
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 70;
					comando_serial = 40;
				}
				else if(c == 'F')
				{
					if(serial_LS_ID == ID_SERVO)
					{
						serial_sub_comando_yo = TRUE;
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 45;
					comando_serial = 36;
				}
				/*else if(c == 'C')
				{
					if(serial_LS_ID == ID_SERVO)
					{
						serial_sub_comando_yo = TRUE;
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 80;
					comando_serial = 65;
				}*/
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 3:			//S12-98?XX;
			if((serial_num_temp < 256) && (serial_num_temp >= 0))
			{
				serial_LS_ID_fin = serial_num_temp;
				if(c == ',')
				{
					if(serial_LS_ID < serial_LS_ID_fin)
					{
						if((serial_LS_ID <= ID_SERVO) && (serial_LS_ID_fin >= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}
					else
					{
						if((serial_LS_ID >= ID_SERVO) && (serial_LS_ID_fin <= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 2;
				}
				else if(c == 'M')
				{
					if(serial_LS_ID < serial_LS_ID_fin)
					{
						if((serial_LS_ID <= ID_SERVO) && (serial_LS_ID_fin >= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}
					else
					{
						if((serial_LS_ID >= ID_SERVO) && (serial_LS_ID_fin <= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 5;
					comando_serial = 1;
					
				}
				else if(c == 'G')
				{
					if(serial_LS_ID < serial_LS_ID_fin)
					{
						if((serial_LS_ID <= ID_SERVO) && (serial_LS_ID_fin >= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}
					else
					{
						if((serial_LS_ID >= ID_SERVO) && (serial_LS_ID_fin <= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 50;
					comando_serial = 10;
				}
				else if(c == 'S')
				{
					if(serial_LS_ID < serial_LS_ID_fin)
					{
						if((serial_LS_ID <= ID_SERVO) && (serial_LS_ID_fin >= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}
					else
					{
						if((serial_LS_ID >= ID_SERVO) && (serial_LS_ID_fin <= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 60;
					comando_serial = 15;
				}
				else if(c == 'R')
				{
					if(serial_LS_ID < serial_LS_ID_fin)
					{
						if((serial_LS_ID <= ID_SERVO) && (serial_LS_ID_fin >= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}
					else
					{
						if((serial_LS_ID >= ID_SERVO) && (serial_LS_ID_fin <= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}
					estado_serial_LS = 150;
				}
				else if(c == 'T')
				{
					if(serial_LS_ID < serial_LS_ID_fin)
					{
						if((serial_LS_ID <= ID_SERVO) && (serial_LS_ID_fin >= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}
					else
					{
						if((serial_LS_ID >= ID_SERVO) && (serial_LS_ID_fin <= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 145;
					comando_serial = 19;
				}
				else if(c == 'L')
				{
					if(serial_LS_ID < serial_LS_ID_fin)
					{
						if((serial_LS_ID <= ID_SERVO) && (serial_LS_ID_fin >= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}
					else
					{
						if((serial_LS_ID >= ID_SERVO) && (serial_LS_ID_fin <= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 70;
					comando_serial = 40;
				}
				else if(c == 'F')
				{
					if(serial_LS_ID < serial_LS_ID_fin)
					{
						if((serial_LS_ID <= ID_SERVO) && (serial_LS_ID_fin >= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}
					else
					{
						if((serial_LS_ID >= ID_SERVO) && (serial_LS_ID_fin <= ID_SERVO))
						{
							serial_sub_comando_yo = TRUE;
						}
					}

					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 45;
					comando_serial = 36;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 5:			//S12M150?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_1 = serial_num_temp;
				if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 6;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if (c == ';' && (uso_crc != 1))		//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '#' && (uso_crc == 1))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				if (c == 'T')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 10;
					comando_serial = 2;
				}
				else if (c == 't')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 15;
					comando_serial = 3;
				}
				else if (c == 'S')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 10;
					comando_serial = 4;
				}
				else if (c == 's')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 15;
					comando_serial = 5;
				}
				else if (c == 'H')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 20;
					comando_serial = 6;
				}
				else if (c == 'C')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 30;
					comando_serial = 30;
				}
				else if (c == 'F')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 200;
					comando_serial = 39;
				}
				else if (c == 'W')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 155;
					comando_serial = 38;
				}
				else if (c == 'P')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 160;
					comando_serial = 37;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			break;
		case 6:			//S12M150:2000?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_2 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;

		case 10:			//S12MX150?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_1 = serial_num_temp;
				if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 11;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 11:			//S12MX150:100?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_2 = serial_num_temp;
				if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 12;
				}
				else if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 12:			//S12MX150:100:?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_3 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
			
		case 15:			//S12MX150?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_1 = serial_num_temp;
				if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 16;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 16:			//S12MX150:100:?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_2 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		
		case 20:			//S12MH150?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_1 = serial_num_temp;
				if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 21;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 21:			//S12MH150:100?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_2 = serial_num_temp;
				if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 22;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 22:			//S12MX150:100?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_3 = serial_num_temp;
				if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 23;
				}
				else if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 23:			//S12MH150:100?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_4 = serial_num_temp;
				if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 24;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 24:			//S12MX150:100?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_5 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 30:			//S12MC150?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_1 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					serial_LS_param_2 = 60000;			//Se fuerza a un min si no se dice tiempo.
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					serial_LS_param_2 = 60000;			//Se fuerza a un min si no se dice tiempo.
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_LS_param_2 = 60000;			//Se fuerza a un min si no se dice tiempo.
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					serial_LS_param_2 = 60000;			//Se fuerza a un min si no se dice tiempo.
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 6;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				if (c == 'T')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 10;
					comando_serial = 31;
				}
				else if (c == 'S')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 10;
					comando_serial = 32;
				}
				else if (c == 'H')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 20;
					comando_serial = 33;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			break;
		case 45:			//S12F?XXX;
			if((serial_num_temp == LONG_MIN) && (c == 'P'))
			{
				estado_serial_LS = 200;
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 50:			//S12G15?XXX;
			if(serial_num_temp != LONG_MIN && serial_num_temp >= 0)
			{
				serial_LS_param_1 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					if (serial_sub_comando_yo == TRUE)
					{
						//GUARDAR EN TABLA COMANDOS TEMP
						temp_orden_sensores[0] = serial_LS_param_1;
						new_temp_serial_task(temp_orden_sensores,1,1,0,serial_comando_imp,FALSE);		//[0],1 sensor, 1 vez, 0 ms, imp!,cabecera)
						temp_orden_sensores[0] = 255;
					}
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE)
					{
						temp_orden_sensores[0] = serial_LS_param_1;
						new_temp_serial_task(temp_orden_sensores,1,1,0,serial_comando_imp,FALSE);		//[0],1 sensor, 1 vez, 0 ms, imp!,cabecera)
						temp_orden_sensores[0] = 255;
					}
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE)
					{
						temp_orden_sensores[0] = serial_LS_param_1;
						new_temp_serial_task(temp_orden_sensores,1,1,0,serial_comando_imp,FALSE);		//[0],1 sensor, 1 vez, 0 ms, imp!,cabecera)
						temp_orden_sensores[0] = 255;
					}
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE)
					{
						temp_orden_sensores[0] = serial_LS_param_1;
						new_temp_serial_task(temp_orden_sensores,1,1,0,serial_comando_imp,FALSE);		//[0],1 sensor, 1 vez, 0 ms, imp!,cabecera)
						temp_orden_sensores[0] = 255;
					}
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else if((serial_num_temp == LONG_MIN) && (c == 'S'))
			{
				estado_serial_LS = 0;
				estado_serial_num = 1;
				estado_serial_LS_volver = 51;
				comando_serial = 11;
				i_temp_orden_sensores = 0;
			}
			else if((serial_num_temp == LONG_MIN) && (c == 's'))
			{
				estado_serial_LS = 0;
				estado_serial_num = 1;
				estado_serial_LS_volver = 51;
				comando_serial = 12;
				i_temp_orden_sensores = 0;
			}
			else if((serial_num_temp == LONG_MIN) && (c == 'W'))
			{
				estado_serial_LS = 0;
				estado_serial_num = 1;
				estado_serial_LS_volver = 156;
				comando_serial = 14;
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 51:			//S12GS10?XXX;
			if(serial_num_temp != LONG_MIN && serial_num_temp >= 0 && i_temp_orden_sensores<TAM_BUFF_SENSORES)
			{
				if (c == ',')
				{
					temp_orden_sensores[i_temp_orden_sensores] = serial_num_temp;
					i_temp_orden_sensores++;
					if(i_temp_orden_sensores == TAM_BUFF_SENSORES ) i_temp_orden_sensores = TAM_BUFF_SENSORES -1;
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 51;
				}
				else if (c == ':')
				{
					temp_orden_sensores[i_temp_orden_sensores] = serial_num_temp;
					i_temp_orden_sensores++;
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 52;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 52:			//S12GS10,2:15?XXX;
			if(serial_num_temp != LONG_MIN && serial_num_temp > 0)
			{
				serial_LS_param_1 = serial_num_temp;
				if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 53;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 53:			//S12GS10,2:15:100?XXX;
			if(serial_num_temp != LONG_MIN && serial_num_temp > 0)
			{
				serial_LS_param_2 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE)
					{
						if (comando_serial == 12) new_temp_serial_task(temp_orden_sensores,i_temp_orden_sensores,serial_LS_param_1,serial_LS_param_2,serial_comando_imp,TRUE);	//[],X sensor, param_1 veces, param_2 ms, imp!,cabecera)
						else new_temp_serial_task(temp_orden_sensores,i_temp_orden_sensores,serial_LS_param_1,serial_LS_param_2,serial_comando_imp,FALSE);						//[],X sensor, param_1 veces, param_2 ms, imp!,cabecera)
					}

					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					for (i_temp_orden_sensores = 0;i_temp_orden_sensores < TAM_BUFF_SENSORES; i_temp_orden_sensores++)
					{
						temp_orden_sensores[i_temp_orden_sensores] = 255;
					}
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE)
					{
						if (comando_serial == 12) new_temp_serial_task(temp_orden_sensores,i_temp_orden_sensores,serial_LS_param_1,serial_LS_param_2,serial_comando_imp,TRUE);	//[],X sensor, param_1 veces, param_2 ms, imp!,cabecera)
						else new_temp_serial_task(temp_orden_sensores,i_temp_orden_sensores,serial_LS_param_1,serial_LS_param_2,serial_comando_imp,FALSE);						//[],X sensor, param_1 veces, param_2 ms, imp!,cabecera)
					}
					for (i_temp_orden_sensores = 0;i_temp_orden_sensores < TAM_BUFF_SENSORES; i_temp_orden_sensores++)
					{
						temp_orden_sensores[i_temp_orden_sensores] = 255;
					}
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE)
					{
						if (comando_serial == 12) new_temp_serial_task(temp_orden_sensores,i_temp_orden_sensores,serial_LS_param_1,serial_LS_param_2,serial_comando_imp,TRUE);	//[],X sensor, param_1 veces, param_2 ms, imp!,cabecera)
						else new_temp_serial_task(temp_orden_sensores,i_temp_orden_sensores,serial_LS_param_1,serial_LS_param_2,serial_comando_imp,FALSE);						//[],X sensor, param_1 veces, param_2 ms, imp!,cabecera)
					}
					for (i_temp_orden_sensores = 0;i_temp_orden_sensores < TAM_BUFF_SENSORES; i_temp_orden_sensores++)
					{
						temp_orden_sensores[i_temp_orden_sensores] = 255;
					}
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE)
					{
						if (comando_serial == 12) new_temp_serial_task(temp_orden_sensores,i_temp_orden_sensores,serial_LS_param_1,serial_LS_param_2,serial_comando_imp,TRUE);	//[],X sensor, param_1 veces, param_2 ms, imp!,cabecera)
						else new_temp_serial_task(temp_orden_sensores,i_temp_orden_sensores,serial_LS_param_1,serial_LS_param_2,serial_comando_imp,FALSE);						//[],X sensor, param_1 veces, param_2 ms, imp!,cabecera)
					}
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 60:			//S12S8?XXX;
			if(serial_num_temp != LONG_MIN && serial_num_temp >= 100)
			{
				serial_LS_param_1 = serial_num_temp;
				if (c == ':')
				{
					if (serial_LS_param_1 == 118 || serial_LS_param_1 == 119)
					{
						estado_serial_LS = 0;
						estado_serial_num = 15;			//Espera leer hexadecimal
						estado_serial_LS_volver = 61;
						serial_num_temp = 0;
					}
					else
					{
						estado_serial_LS = 0;
						estado_serial_num = 1;
						estado_serial_LS_volver = 61;
					}
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else if(serial_num_temp == LONG_MIN && c == 'V')
			{
				estado_serial_LS = 200;
				comando_serial = 16;
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 61:			//S12S8:1287?XXX
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_2 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;

		case 69:			//S12T440?XXX
			if(serial_num_temp != LONG_MIN && serial_num_temp > 0)
			{
				serial_LS_param_1 = serial_num_temp;
				if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 71;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;

		case 70:			//S12L???XXX;
			if(serial_num_temp != LONG_MIN && serial_num_temp >= 0)
			{
				serial_LS_param_1 = serial_num_temp;
				if ((c == ':'))
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 71;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else if((serial_num_temp == LONG_MIN) && (c == 'R'))
			{
				comando_serial = 41;
				estado_serial_LS = 200;
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 71:			//S12L120:?XXX;
			if(serial_num_temp != LONG_MIN && serial_num_temp >= 0)
			{
				serial_LS_param_2 = serial_num_temp;
				if ((c == ':'))
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 72;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 72:			//S12L120:150:?XXX;
			if(serial_num_temp != LONG_MIN && serial_num_temp >= 0)
			{
				serial_LS_param_3 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		
		/*case 80:
			if((serial_num_temp == LONG_MIN) && (c == 'E'))
			{
				estado_serial_LS = 200;
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;*/
		
	/**********************************
	Reset [RP] -> 150 {20} 			//un parámetro (a última vez arrancado)
	Reset [Rp] -> 150 {21} 			//un parámetro (a valor original)
	ResetServo [RS]-> 150 {22} 		//todo el servomotor (a valor última vez arrancado)
	ResetServo [Rs]-> 150 {23} 		//todo el servomotor (a valor original)
	**********************************/
		case 150:			//S12R?XXX;
			if(c == 'P')
			{
				estado_serial_LS = 0;
				estado_serial_num = 1;
				estado_serial_LS_volver = 151;
				comando_serial = 20;
			}
			else if(c == 'p')
			{
				estado_serial_LS = 0;
				estado_serial_num = 1;
				estado_serial_LS_volver = 151;
				comando_serial = 21;
			}
			else if(c == 'S')
			{
				comando_serial = 22;
				estado_serial_LS = 200;
			}
			else if(c == 's')
			{
				comando_serial = 23;
				estado_serial_LS = 200;
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 151:			//S12R105?XXX;
			if(serial_num_temp != LONG_MIN && serial_num_temp>=100)
			{
				serial_LS_param_1 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
			
		case 155:			//S12XX200?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_1 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 156:			//S12XX200?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_1 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					if (serial_sub_comando_yo == TRUE)
					{
						//GUARDAR EN TABLA COMANDOS TEMP
						new_temp_serial_task(temp_orden_sensores,0,1,serial_LS_param_1,serial_comando_imp,FALSE);		//[0],0 sensores, 1 vez, serial_LS_param_1 ms, imp!,cabecera)
					}
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					
					if (serial_sub_comando_yo == TRUE)
					{
						new_temp_serial_task(temp_orden_sensores,0,1,serial_LS_param_1,serial_comando_imp,FALSE);		//[0],0 sensores, 1 vez, serial_LS_param_1 ms, imp!,cabecera)
					}
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE)
					{
						new_temp_serial_task(temp_orden_sensores,0,1,serial_LS_param_1,serial_comando_imp,FALSE);		//[0],0 sensores, 1 vez, serial_LS_param_1 ms, imp!,cabecera)
					}
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE)
					{
						new_temp_serial_task(temp_orden_sensores,0,1,serial_LS_param_1,serial_comando_imp,FALSE);		//[0],0 sensores, 1 vez, serial_LS_param_1 ms, imp!,cabecera)
					}
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
			
		case 160:			//S12XX200?XXX;
			if(serial_num_temp != LONG_MIN)
			{
				serial_LS_param_1 = serial_num_temp;
				if ((c == ';') && (uso_crc != 1))				//Fin comando
				{
					//GUARDAR EN TABLA COMANDOS TEMP
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
					if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
				else if (c == '|')
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 1;
				}
				else if (c == '!')
				{
					serial_comando_imp = TRUE;
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 201;
				}
				else if ((c == '#') && (uso_crc != 0))		//Mirar CRC
				{
					if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
					estado_serial_LS = 0;
					estado_serial_num = 10;
					estado_serial_LS_volver = 250;
					tratando_serie = FALSE;
					serial_crc=0;
				}
				else if (c == ':')
				{
					estado_serial_LS = 0;
					estado_serial_num = 1;
					estado_serial_LS_volver = 6;
				}
				else
				{
					estado_serial_LS = 1;
					tratando_serie = FALSE;
				}
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		
		case 200:			//S12XXXXX??XXX;
			if ((c == ';') && (uso_crc != 1))				//Fin comando
			{
				//GUARDAR EN TABLA COMANDOS TEMP
				if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
				if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
				if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			if (c == '!')
			{
				serial_comando_imp = TRUE;
				if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
				estado_serial_LS = 201;
			}
			else if (c == '|')
			{
				if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
				estado_serial_LS = 1;
			}
			else if ((c == '#') && (uso_crc != 0))			//Mirar CRC
			{
				if (serial_sub_comando_yo == TRUE) new_temp_task(comando_serial,serial_LS_param_1,serial_LS_param_2,serial_LS_param_3,serial_LS_param_4,serial_LS_param_5,serial_comando_imp);
				estado_serial_LS = 0;
				estado_serial_num = 10;
				estado_serial_LS_volver = 250;
				tratando_serie = FALSE;
				serial_crc=0;
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		case 201:			//S12XXXXX!?XXX;
			if ((c == ';') && (uso_crc != 1))				//Fin comando
			{
				//GUARDAR EN TABLA COMANDOS TEMP
				if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
				if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			else if (c == '|')
			{
				estado_serial_LS = 1;
			}
			else if ((c == '#') && (uso_crc != 0))			//Mirar CRC
			{
				estado_serial_LS = 0;
				estado_serial_num = 10;
				estado_serial_LS_volver = 250;
				tratando_serie = FALSE;
				serial_crc=0;
			}
			else
			{
				estado_serial_LS = 1;
				tratando_serie = FALSE;
			}
			break;
		
		case 250:
			if (c == ';' && (crc == serial_crc))
			{	
				if (LS_TEMP_TS_GUARDADO > 0) copy_temp_to_task();
				if (LS_TEMP_TS_GUARDADO_SERIAL > 0) copy_temp_to_task_serial();
			}
			
			estado_serial_LS = 1;
			tratando_serie = FALSE;
			break;
		
		default:
			break;
	}
}

/* USER CODE END 1 */
