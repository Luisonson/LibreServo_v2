/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LS_flash.h
  * @brief          : Header for LS_flash.c file
  * @author			: Luis Picó Chausson
  * @version		: 0.1
  * Created on: 30 mar. 2022
  ******************************************************************************
  * @attention
  *
  * LibreServo by Luis Picó is licensed under a Creative Commons 
  * Attribution-ShareAlike 4.0 International License
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "stm32f3xx.h"

void save_new_values_flash();
void reset_flash();
void reset_value_flash();
uint16_t write_init_flash();
uint16_t read_init_flash();
void read_init_value_flash();

uint16_t write_32_flash(uint16_t VirtAddress,uint32_t variable);
uint16_t read_32_flash(uint16_t VirtAddress,uint32_t* variable);
void add_new_values_flash();
