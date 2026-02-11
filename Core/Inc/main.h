/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32h7xx_hal.h"

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
#define VCC4_Pin GPIO_PIN_2
#define VCC4_GPIO_Port GPIOC
#define GND4_Pin GPIO_PIN_3
#define GND4_GPIO_Port GPIOC
#define DPS310_NCS_Pin GPIO_PIN_4
#define DPS310_NCS_GPIO_Port GPIOA
#define PMW3901_NCS_Pin GPIO_PIN_12
#define PMW3901_NCS_GPIO_Port GPIOB
#define GND2_Pin GPIO_PIN_6
#define GND2_GPIO_Port GPIOC
#define VCC2_Pin GPIO_PIN_7
#define VCC2_GPIO_Port GPIOC
#define ICM42688INT_Pin GPIO_PIN_8
#define ICM42688INT_GPIO_Port GPIOC
#define ICM42688INT_EXTI_IRQn EXTI9_5_IRQn
#define GND3_Pin GPIO_PIN_9
#define GND3_GPIO_Port GPIOA
#define VCC3_Pin GPIO_PIN_10
#define VCC3_GPIO_Port GPIOA
#define VCC1_Pin GPIO_PIN_11
#define VCC1_GPIO_Port GPIOA
#define GND1_Pin GPIO_PIN_12
#define GND1_GPIO_Port GPIOA
#define ICM42688_NCS_Pin GPIO_PIN_2
#define ICM42688_NCS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
