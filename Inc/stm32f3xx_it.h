/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : stm32f3xx_it.h
  * @brief          : This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F3xx_IT_H
#define __STM32F3xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
void TIM1_BRK_TIM15_IRQHandler(void);
void TIM1_UP_TIM16_IRQHandler(void);
void TIM1_TRG_COM_TIM17_IRQHandler(void);
void USART1_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F3xx_IT_H */
