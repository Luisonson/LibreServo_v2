/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LS_funciones.h
  * @brief          : Header for LS_funciones.c file.
  *                   This file contains the defines of generic functions
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

#include "stm32f3xx.h"
#include "stm32f3xx_ll_usart.h"

#define TAM_BUFF_SENSORES			26

void init_variables();
void change_USART1_speed();
void init_USART1_rx_as_INPUT();

void Delay_ms(uint32_t);
uint32_t Get_Micros();

void SET_PIN (GPIO_TypeDef *GPIOx, uint32_t PinMask);
void CLEAR_PIN (GPIO_TypeDef *GPIOx, uint32_t PinMask);
void FLIP_PIN (GPIO_TypeDef *GPIOx, uint32_t PinMask);
uint32_t READ_PIN (GPIO_TypeDef *GPIOx, uint32_t PinMask);

void F_LED_RGB(uint8_t Z_RED, uint8_t Z_GREEN, uint8_t Z_BLUE);
void comando_LED_RGB();
void comando_LED_RAINBOW();
void RAINBOW();
void tone(uint16_t hz, uint16_t duracion_ms, uint16_t  amplitud);
void comando_tone();
void comando_FD();
void comando_CE();
void int_to_char(char *str, long my_int);
void comando_get();
void read_bat();
void read_temps();

void byteToSPI(uint8_t byte_in);
uint8_t ReadSSI();

void get_kalman_gains();

void new_temp_task(uint8_t n_comando,long n_param_1,long n_param_2,long n_param_3,long n_param_4,long n_param_5,boolean prioritaria);
void new_task(uint8_t n_comando,long n_param_1,long n_param_2,long n_param_3,long n_param_4,long n_param_5,boolean prioritaria);
void new_temp_task_time(uint8_t n_comando,long n_param_1,long n_param_2,long n_param_3,long n_param_4,long n_param_5,long n_time_comando,boolean prioritaria);
void new_task_time(uint8_t n_comando,long n_param_1,long n_param_2,long n_param_3,long n_param_4,long n_param_5,long n_time_comando,boolean prioritaria);
void copy_temp_to_task();
void new_temp_serial_task(volatile uint16_t n_orden_sensores[TAM_BUFF_SENSORES],uint8_t num_sensores,long n_param_1,long n_param_2, boolean prioritaria, boolean cabecera);
void copy_temp_to_task_serial();

void comando_set_value();

void init_crcccitt_tab();
uint16_t update_crc_ccitt( uint16_t crc, unsigned char c );
