/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LS_boolean.h
  * @brief          : Boolean variable type definition.
  * @author			: Luis Picó Chausson
  * @version		: 0.1
  * Created on: 11 Abr. 2022
  ******************************************************************************
  * @attention
  *
  * LibreServo by Luis Picó is licensed under a Creative Commons 
  * Attribution-ShareAlike 4.0 International License
  *
  ******************************************************************************
  */
/* USER CODE END Header */

typedef enum
{
  FALSE = 0,
  TRUE = !FALSE
} boolean;
