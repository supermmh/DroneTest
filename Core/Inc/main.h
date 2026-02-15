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
extern volatile float Mag_Gauss_x;
extern volatile float Mag_Gauss_y;
extern volatile float Mag_Gauss_z;
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
#define ICM42688GND_Pin GPIO_PIN_3
#define ICM42688GND_GPIO_Port GPIOE
#define ICM42688VCC_Pin GPIO_PIN_4
#define ICM42688VCC_GPIO_Port GPIOE
#define DPS310VCC_Pin GPIO_PIN_2
#define DPS310VCC_GPIO_Port GPIOA
#define DPS310GND_Pin GPIO_PIN_3
#define DPS310GND_GPIO_Port GPIOA
#define DPS310_NCS_Pin GPIO_PIN_4
#define DPS310_NCS_GPIO_Port GPIOA
#define DPS310_Pin GPIO_PIN_5
#define DPS310_GPIO_Port GPIOA
#define PMW3901_NCS_Pin GPIO_PIN_12
#define PMW3901_NCS_GPIO_Port GPIOB
#define PMW3901_Pin GPIO_PIN_13
#define PMW3901_GPIO_Port GPIOB
#define PMW3901VCC_Pin GPIO_PIN_14
#define PMW3901VCC_GPIO_Port GPIOD
#define ICM42688INT_Pin GPIO_PIN_8
#define ICM42688INT_GPIO_Port GPIOC
#define ICM42688INT_EXTI_IRQn EXTI9_5_IRQn
#define PMW3901GND_Pin GPIO_PIN_15
#define PMW3901GND_GPIO_Port GPIOA
#define ICM42688_Pin GPIO_PIN_10
#define ICM42688_GPIO_Port GPIOC
#define ICM42688_NCS_Pin GPIO_PIN_2
#define ICM42688_NCS_GPIO_Port GPIOD
#define MMC5983VCC_Pin GPIO_PIN_4
#define MMC5983VCC_GPIO_Port GPIOB
#define MMC5983GND_Pin GPIO_PIN_5
#define MMC5983GND_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
