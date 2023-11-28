/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Brad Kauffman.
  *   This Program is free software; you can redistribute it and/or
  *   modify it under the terms of the GNU General Public
  *   License as published by the Free Software Foundation; either
  *   version 3 of the License, or (at your option) any later version.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RudderPosition_Pin GPIO_PIN_0
#define RudderPosition_GPIO_Port GPIOA
#define Current_Pin GPIO_PIN_1
#define Current_GPIO_Port GPIOA
#define Voltage_Pin GPIO_PIN_2
#define Voltage_GPIO_Port GPIOA
#define TempMotor_Pin GPIO_PIN_3
#define TempMotor_GPIO_Port GPIOA
#define TempDriver_Pin GPIO_PIN_4
#define TempDriver_GPIO_Port GPIOA
#define EnableR_Pin GPIO_PIN_12
#define EnableR_GPIO_Port GPIOB
#define EnableL_Pin GPIO_PIN_13
#define EnableL_GPIO_Port GPIOB
#define LPWM_Pin GPIO_PIN_15
#define LPWM_GPIO_Port GPIOA
#define RPWM_Pin GPIO_PIN_3
#define RPWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LPWMset TIM2->CCR1
#define RPWMset TIM2->CCR2
#define EnableL_High HAL_GPIO_WritePin(EnableL_GPIO_Port,EnableL_Pin,GPIO_PIN_SET)
#define EnableR_High HAL_GPIO_WritePin(EnableR_GPIO_Port,EnableR_Pin,GPIO_PIN_SET)
#define EnableR_Low HAL_GPIO_WritePin(EnableR_GPIO_Port,EnableR_Pin,GPIO_PIN_RESET)
#define EnableL_Low HAL_GPIO_WritePin(EnableL_GPIO_Port,EnableL_Pin,GPIO_PIN_RESET)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
