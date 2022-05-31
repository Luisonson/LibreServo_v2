/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char dma_tx_buff[DMA_TX_BUFF_SIZE+12] = { 0 };			//Se da 12 extra para facilitar el código y nunca salirnos de memoria
char dma_rx_buff[DMA_RX_BUFF_SIZE] = { 0 };

int pos_buff_rx = 0;

volatile uint16_t dma_adc1_buff[TAM_DMA_ADC1_BUFF] = { 0 };

extern volatile uint8_t enviado_USART1;					//Inicializado en LS_funciones.c

volatile uint32_t data_SSI_IN_old = 0;
volatile long data_SSI_IN_internal = 0;
volatile long data_SSI_IN_vuelta = 0;


////float grados_encoder = 0;
/*volatile float grados_encoder_kalman = 0.0;
volatile float grados_encoder_kalman_antes = 0.0;
volatile float velocidad_antes = 0.0;*/

uint32_t tiempo_ms = 0;

extern volatile uint32_t ticks;					//Inicializado en funciones.c

/*extern volatile float k_corriente;				//Inicializado en funciones.c
extern float k_temp_ext;						//Inicializado en funciones.c
extern float k_temp_int;						//Inicializado en funciones.c
extern float k_volts;							//Inicializado en funciones.c
extern volatile float k_encoder;				//Inicializado en funciones.c*/
/********************************************************************************************************/
uint16_t version_LS = 6;
/********************************************************************************************************/
volatile uint16_t ID_SERVO;

/*extern uint16_t q_corriente, q_temp_ext, q_temp_int, q_volts, q_encoder;
extern uint16_t varianza_corriente, varianza_temp_ext, varianza_temp_int, varianza_volts, varianza_encoder;*/
uint32_t vel_serie;
////uint16_t corte_temp_ext, corte_temp_int, corte_volts_alto, corte_volts_bajo, corte_corriente;

extern volatile uint8_t bat_estado;						//Inicializado en LS_funciones.c Estado batería 0=Good, 1=LOW, 2=HIGH
extern volatile uint8_t temp_estado;					//Inicializado en LS_funciones.c Estado temperature 0=Good, 1=HIGH_external, 2=HIGH_internal, 3=HIGH_BOTH

/*volatile uint16_t uso_crc; 			//0=False, 1=True, 2=Both
volatile uint16_t CRC_START_CCITT;

extern volatile uint16_t K_P_LS, K_D_LS, K_I_LS;		//Inicializado en LS_PID.c
extern volatile uint16_t K_P_M, K_D_M, K_I_M;			//Inicializado en LS_PID.c*/

//volatile int offset_corriente;

////volatile uint16_t envio_si_leo;			//No enviar si se reciben datos (0=False, 1=True, 2=True envío actual)
/*boolean LED_modo_RAINBOW = FALSE;
uint8_t rgbColor[3] = {255,0,0}; //Empieza con Rojo el arcoíris
uint8_t decColor = 0;
uint8_t incColor = 1;
uint8_t brillo_led = 0;*/

volatile boolean status_tone = FALSE;
extern volatile uint8_t saludo_inicial;		//inicializado en LS_flash.h Saludo Inicial (0=False, 1=True)

/*uint16_t version_LS = 4;
volatile uint16_t ID_SERVO = 1;

uint16_t q_corriente, q_temp_ext, q_temp_int, q_volts, q_encoder;
uint16_t varianza_corriente, varianza_temp_ext, varianza_temp_int, varianza_volts, varianza_encoder;
uint32_t vel_serie;
uint16_t corte_temp_ext, corte_temp_int, corte_volts_alto, corte_volts_bajo, corte_corriente;*/

/* Values of Variable1, Variable2 and Variable3 */
uint16_t VarValue1/*, VarValue2, VarValue3*/;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x0001, 0x0002, 0x0003};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};

boolean error_read_init_flash = FALSE;

/*struct LS_temp_task_scheduler {uint8_t comando[LS_TASK_SCHEDULER_SIZE];long param_1[LS_TASK_SCHEDULER_SIZE];long param_2[LS_TASK_SCHEDULER_SIZE];long param_3[LS_TASK_SCHEDULER_SIZE];long param_4[LS_TASK_SCHEDULER_SIZE];long param_5[LS_TASK_SCHEDULER_SIZE];long time_comando[LS_TASK_SCHEDULER_SIZE];boolean prioritaria[LS_TASK_SCHEDULER_SIZE];};
volatile struct LS_temp_task_scheduler temp_task_scheduler = {{0},{0},{0},{0},{0},{0},{0},{0}};

struct LS_task_scheduler {uint8_t comando[LS_TASK_SCHEDULER_SIZE];long param_1[LS_TASK_SCHEDULER_SIZE];long param_2[LS_TASK_SCHEDULER_SIZE];long param_3[LS_TASK_SCHEDULER_SIZE];long param_4[LS_TASK_SCHEDULER_SIZE];long param_5[LS_TASK_SCHEDULER_SIZE];long time_comando[LS_TASK_SCHEDULER_SIZE];uint8_t iniciado[LS_TASK_SCHEDULER_SIZE];};
//extern volatile struct LS_task_scheduler task_scheduler;
volatile struct LS_task_scheduler task_scheduler = {{0},{0},{0},{0},{0},{0},{0},{0}};
volatile uint8_t LS_TS_ESTADO = 0, LS_TEMP_TS_GUARDADO = 0, LS_TS_GUARDADO = 0;
struct LS_comando_motor {uint8_t comando; long pos_now; float velocidad; float aceleracion; long pos_destino; float pos_destino_f; long pos_ini; long pos_final; long time_to_end; float step; long time_ramp1; long time_ramp2; float step_ramp1; float step_ramp2;};
volatile struct LS_comando_motor comando_motor = {0,0,0.0,0.0,0,0.0,0,0,0,0.0,0,0,0.0,0.0};

volatile uint16_t crc_tabccitt[256];*/
/*volatile uint16_t uso_crc = 2; 			//0=False, 1=True, 2=Both
volatile uint16_t CRC_START_CCITT;
volatile uint16_t CRC_POLY_CCITT;*/

//struct LS_comando_motor {uint8_t comando; long pos_now; float velocidad; float aceleracion; long pos_destino; float pos_destino_f; long pos_ini; long pos_final; long time_to_end; float step; long time_ramp1; long time_ramp2; float step_ramp1; float step_ramp2;};
//extern volatile struct LS_comando_motor comando_motor;			//Inicializado en LS_Funciones.c

////extern volatile uint16_t CRC_POLY_CCITT;

//volatile uint16_t envio_si_leo = 1;


/*********************************************************
 * [0] -> Temp_interna
 * [1] -> Temp_externa
 * [2] -> Temp_interna_raw
 * [3] -> Temp_interna_kalman
 * [4] -> Temp_externa_raw
 * [5] -> Temp_externa_kalman
 * [6] -> Voltage
 * [7] -> Voltage_raw
 * [8] -> Voltage_kalman
 * [9] -> Corriente
 * [10] -> Corriente_raw
 * [11] -> Corriente_kalman
 * [12] -> Encoder_raw
 * [13] -> Encoder_kalman
 * [14] -> Velocidad_enc
 * [15] -> Aceleración_enc
 * [16] -> Velocidad_sim
 * [17] -> Aceleración_sim
 * [18] -> Pos_objetivo
 * [19] -> Error
 * [20] -> DError
 * [21] -> PDTerm
 * [22] -> Iterm
 * [23] -> PWM
 * [24] -> output_LS
 *********************************************************/
////long buff_sensores[TAM_BUFF_SENSORES] = { 0 };
////volatile uint8_t orden_sensores[TAM_BUFF_SENSORES] = { 0 };
//volatile uint8_t temp_orden_sensores[TAM_BUFF_SENSORES] = {};
//uint8_t orden_sensores_print[TAM_BUFF_SENSORES];
/*struct LS_temp_comandos_serial {uint8_t orden_sensores[LS_TASK_SERIAL_SIZE][TAM_BUFF_SENSORES]; long times[LS_TASK_SERIAL_SIZE]; long time[LS_TASK_SERIAL_SIZE]; boolean prioritaria[LS_TASK_SERIAL_SIZE];};
volatile struct LS_temp_comandos_serial temp_comandos_serial = {{},{0},{0},{0}};
struct LS_comandos_serial {uint8_t orden_sensores[LS_TASK_SERIAL_SIZE][TAM_BUFF_SENSORES]; long times[LS_TASK_SERIAL_SIZE]; long time[LS_TASK_SERIAL_SIZE];uint8_t iniciado[LS_TASK_SERIAL_SIZE];};
volatile struct LS_comandos_serial comandos_serial = {{},{0},{0},{5}};
volatile uint8_t LS_TS_ESTADO_SERIAL = 0, LS_TEMP_TS_GUARDADO_SERIAL = 0, LS_TS_GUARDADO_SERIAL = 0;*/
////struct LS_comandos_serial {uint8_t orden_sensores[LS_TASK_SERIAL_SIZE][TAM_BUFF_SENSORES]; long times[LS_TASK_SERIAL_SIZE]; long time[LS_TASK_SERIAL_SIZE];uint8_t iniciado[LS_TASK_SERIAL_SIZE];boolean cabecera[LS_TASK_SERIAL_SIZE];};
////extern volatile struct LS_comandos_serial comandos_serial;			//Inicializado en LS_funciones.c
////extern volatile uint8_t LS_TS_ESTADO_SERIAL, LS_TS_GUARDADO_SERIAL;	//Inicializado en LS_funciones.c


/*volatile boolean send_RS485 = FALSE;
uint32_t sum_enviados = 0;
uint32_t serial_sum_a_enviar = 0;
uint8_t LS_ESTADO_RS485 = 0;
volatile uint8_t serial_RS485_datos_enviar = 0;*/

//volatile uint16_t K_P_LS, K_D_LS, K_I_LS;
//volatile float K_P_LS_f, K_D_LS_f, K_I_LS_f;
//volatile float ITerm_LS = 0.0;
//volatile uint16_t K_P_M, K_D_M, K_I_M;
//volatile float K_P_M_f, K_D_M_f, K_I_M_f;
//volatile float ITerm_M = 0.0;
//long pos_bef_PID_LS,pos_bef_PID_M;

//uint16_t t_ramp_t = 200, t_ramp_s = 200, a_ramp_t = 500;

////extern volatile int pos_buff_rx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),7, 1));

  /* USER CODE BEGIN Init */
  F_LED_RGB(0,0,0); //Inicializar LED
  
  init_variables();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick_Config(72000);			//Para que interrumpa cada milisegundo (ms). ticks++ . Configura SysTick como la interrupción de prioriordad MÁS BAJA
  MX_GPIO_Init();					//Se inicializa unas pocas líneas abajo de nuevo... no pasa nada
  init_USART1_rx_as_INPUT();


  /* Unlock the Flash Program Erase controller */
  FLASH_Unlock();

  /* EEPROM Init */
  EE_Init();

  if(READ_PIN(GPIOB,LL_GPIO_PIN_7) == 0)		//Pin USART1_RX
  {
	  uint32_t tiempo_reset, tiempo_led;
	  int tiempo_flash_led;
	  boolean ciclo_led = FALSE;
	  tiempo_reset = ticks;
	  tiempo_led = ticks;

	  tiempo_flash_led = 600;

	  GPIOA->BRR=LED_GREEN;				//Clear_pin
	  GPIOA->BRR=LED_RED;				//Clear_pin
	  GPIOA->BRR=LED_BLUE;				//Clear_pin

	  while (READ_PIN(GPIOB,LL_GPIO_PIN_7) == 0 && (ticks - tiempo_reset) < 6000)		//Wait 6 secs to restore LibreServo
	  {
		  if((ticks - tiempo_led) > tiempo_flash_led)
		  {
			  tiempo_led = ticks;
			  FLIP_PIN(GPIOA,LED_GREEN);
			  FLIP_PIN(GPIOA,LED_RED);
			  FLIP_PIN(GPIOA,LED_BLUE);
			  if(ciclo_led == FALSE) ciclo_led = TRUE;
			  else
			  {
				  ciclo_led = FALSE;
				  tiempo_flash_led = tiempo_flash_led / 1.3;
			  }
		  }
	  }
	  if ((ticks - tiempo_reset) >= 6000)
	  {
		  reset_flash();
	  }
  }

  if(EE_ReadVariable(0x0001,&VarValue1) == 1) 	//No se encuentra la variable
  {
	  EE_WriteVariable(0x0001,version_LS);
	  /*Escribir todas las variables*/
	  write_init_flash();
  }
  else if (VarValue1 != version_LS)
  {
	  EE_WriteVariable(0x0001,version_LS);
  }
  /*Escribir variables nuevas si faltaran*/
  add_new_values_flash();
  if(read_init_flash() != 0) error_read_init_flash = TRUE;

  init_crcccitt_tab();			//Inicializa la tabla de CRC. Por defecto CRC-16/AUG-CCITT
  
  get_kalman_gains();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  
  while(dma_adc1_buff[31] == 0) {/*Wait to fill dma_adc1_buff*/}

  LL_TIM_EnableCounter(TIM17);			//Timer17 starts running (TIM17->CR1.CEN)
  LL_TIM_EnableIT_UPDATE(TIM17);		//Enable the Update interrupt in TIM17 (TIM17->DIER.UIE ---> TIM17->SR.UIF)
  LL_TIM_EnableCounter(TIM16);			//Timer16 starts running (TIM16->CR1.CEN)
  LL_TIM_EnableIT_UPDATE(TIM16);		//Enable the Update interrupt in TIM16 (TIM16->DIER.UIE ---> TIM16->SR.UIF)
  LL_TIM_EnableCounter(TIM15);			//Timer16 starts running (TIM15->CR1.CEN)
  LL_TIM_EnableIT_UPDATE(TIM15);		//Enable the Update interrupt in TIM15 (TIM15->DIER.UIE ---> TIM15->SR.UIF)

  SET_PIN(GPIOA,LL_GPIO_PIN_10);  		//Habilitar el fan3227
  Delay_ms(20);
  
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1); /* Enable output on channel */
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N); /* Enable output on channel */
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N); /* Enable output on channel */
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2); /* Enable output on channel */

  LL_TIM_EnableCounter(TIM1);

  LL_TIM_EnableAllOutputs(TIM1);

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, sizeof(dma_tx_buff));
  LL_USART_ClearFlag_TC(USART1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  if(saludo_inicial)
  {
	  F_LED_RGB(255,0,0);
	  tone(2093, 50, 1000);				//2093Hz(DO6, 50ms, 1000 de amplitud)
	  F_LED_RGB(0,0,0);
	  tiempo_ms=ticks;
	  while( ticks - tiempo_ms < 20 ){/*Espera 20ms*/}
	  F_LED_RGB(0,255,0);
	  tone(3136, 50, 1000);				//3136Hz(SOL6, 50ms, 1000 de amplitud)
	  F_LED_RGB(0,0,0);
	  tiempo_ms=ticks;
	  while( ticks - tiempo_ms < 20 ){/*Espera 20ms*/}
	  F_LED_RGB(0,0,255);
	  tone(2637, 50, 1000);				//2637Hz(MI6, 50ms, 1000 de amplitud)
	  F_LED_RGB(255,255,255);
	  tone(4186, 100, 1800);			//4186Hz(DO7, 100ms, 1800 de amplitud [máxima amplitud])
	  F_LED_RGB(0,0,0);
  }

  memset(dma_tx_buff,0,strlen(dma_tx_buff));			//fill dma_tx_buff with 0s
  if(error_read_init_flash)
  {
	dma_tx_buff[0] = 'F';
	dma_tx_buff[1] = 'L';
	dma_tx_buff[2] = 'A';
	dma_tx_buff[3] = 'S';
	dma_tx_buff[4] = 'H';
	dma_tx_buff[5] = ' ';
	dma_tx_buff[6] = 'E';
	dma_tx_buff[7] = 'R';
	dma_tx_buff[8] = 'R';
	dma_tx_buff[9] = 'O';
	dma_tx_buff[10] = 'R';
	dma_tx_buff[11] = 10;
	dma_tx_buff[12] = 13;
	enviado_USART1 = FALSE;
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, strlen(dma_tx_buff));
	GPIOB->BSRR=LL_GPIO_PIN_5;
	LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_4);
	while(1) {};											//Something went wrong...
  }

  ////data_SSI_IN_old = 0;

  while (1)			//main loop
  {
	  if(status_tone)
	  {
		  if(bat_estado == 0)
		  {
			  comando_tone();
			  status_tone = FALSE;
		  }
		  else status_tone = FALSE;
	  }
	  if(bat_estado == 1)		//BAT_LOW
	  {
		  tiempo_ms = ticks;

		  for(int temp_i = 255; temp_i >=0; temp_i = temp_i - 8)
		  {
			while( (ticks - tiempo_ms) < 15 ){/*Espera 15ms*/}
			tiempo_ms = ticks;
			F_LED_RGB(0,0,temp_i);
		  }
		  
		  F_LED_RGB(0,0,0);
		  tiempo_ms = ticks;
		  while( (ticks - tiempo_ms) < 1000 ){/*Espera 1000ms*/}

	  }
	  else if(bat_estado == 2)		//BAT_HIGH
	  {
		  tiempo_ms = ticks;

		  for(int temp_i = 255; temp_i >= 0; temp_i = temp_i - 8)
		  {
			while( (ticks - tiempo_ms) < 15 ){/*Espera 15ms*/}
			tiempo_ms = ticks;
			F_LED_RGB(temp_i,temp_i,0);
		  }
		  
		  F_LED_RGB(0,0,0);
		  tiempo_ms = ticks;
		  while( (ticks - tiempo_ms) < 1000 ){/*Espera 1000ms*/}

	  }
	  if ( temp_estado != 0 )		//ANY TEMPERATURE HIGH
	  {
		  tiempo_ms = ticks;

		  for(int temp_i = 255; temp_i >= 0; temp_i = temp_i - 8)
		  {
			while( (ticks - tiempo_ms) < 15 ){/*Espera 15ms*/}
			tiempo_ms = ticks;
			F_LED_RGB(temp_i,0,0);
		  }
		  
		  F_LED_RGB(0,0,0);
		  tiempo_ms = ticks;
		  while( (ticks - tiempo_ms) < 1000 ){/*Espera 1000ms*/}
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_SYSCLK);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PCLK2);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM17_CLKSOURCE_PCLK2);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM16_CLKSOURCE_PCLK2);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM15_CLKSOURCE_PCLK2);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Channel4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* USART1_IRQn interrupt configuration */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(USART1_IRQn);
  /* TIM1_TRG_COM_TIM17_IRQn interrupt configuration */
  NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),7, 0));
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration
  PA6   ------> ADC1_IN10
  PA7   ------> ADC1_IN15
  PB0   ------> ADC1_IN11
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  /* USER CODE BEGIN ADC1_Init 1 */
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)dma_adc1_buff);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 32);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)(&ADC1->DR));

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV1;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_TEMPSENSOR);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_181CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_10);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_10, LL_ADC_SAMPLINGTIME_181CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_10, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_15);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_15, LL_ADC_SAMPLINGTIME_181CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_15, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_11);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_181CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */

  // ADC CAlibration
  LL_ADC_StartCalibration(ADC1,LL_ADC_SINGLE_ENDED);
  // Wait ADC Calibration
  while ( LL_ADC_IsCalibrationOnGoing(ADC1) );
  wait_loop_index = LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES;
  while(wait_loop_index != 0)
  {
	  wait_loop_index--;
  }
  // Enable ADC
  LL_ADC_Enable(ADC1);
  // Wait to Start ADC
  while ( LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0 );

  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  LL_ADC_REG_StartConversion(ADC1);

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 1));
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1800;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 900;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 15;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PA8   ------> TIM1_CH1
  PA9   ------> TIM1_CH2
  PA11   ------> TIM1_CH1N
  PA12   ------> TIM1_CH2N
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_11|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);

  /* TIM15 interrupt Init */
  NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  TIM_InitStruct.Prescaler = 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 9000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM15, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM15);
  LL_TIM_SetClockSource(TIM15, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM15, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM15);
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);

  /* TIM16 interrupt Init */
  NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 1));
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  TIM_InitStruct.Prescaler = 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 36000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM16);
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  TIM_InitStruct.Prescaler = 586;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 255;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV4;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM17, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM17);
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB6   ------> USART1_TX
  PB7   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

  /* USART1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN USART1_Init 1 */

  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)dma_tx_buff);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)(&USART1->TDR));
  LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);

  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)dma_rx_buff);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, sizeof(dma_rx_buff));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&USART1->RDR);
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);

  LL_USART_ClearFlag_IDLE(USART1);
  LL_USART_EnableIT_IDLE(USART1);

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_8;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
  change_USART1_speed();
  LL_USART_ClearFlag_TC(USART1);
  LL_USART_EnableIT_TC(USART1);

  LL_USART_EnableDMAReq_TX(USART1);
  LL_USART_EnableDMAReq_RX(USART1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0|LL_GPIO_PIN_2|LL_GPIO_PIN_3|LL_GPIO_PIN_4
                          |LL_GPIO_PIN_5);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
