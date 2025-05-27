/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define POWER_OUT2_EN_Pin GPIO_PIN_13
#define POWER_OUT2_EN_GPIO_Port GPIOC
#define POWER_OUT1_EN_Pin GPIO_PIN_14
#define POWER_OUT1_EN_GPIO_Port GPIOC
#define POWER_EN_5V_Pin GPIO_PIN_15
#define POWER_EN_5V_GPIO_Port GPIOC
#define ENCODER_A1_Pin GPIO_PIN_0
#define ENCODER_A1_GPIO_Port GPIOA
#define ENCODER_A1_EXTI_IRQn EXTI0_IRQn
#define ENCODER_B1_Pin GPIO_PIN_2
#define ENCODER_B1_GPIO_Port GPIOA
#define ENCODER_B1_EXTI_IRQn EXTI2_IRQn
#define ENCODER_B2_Pin GPIO_PIN_9
#define ENCODER_B2_GPIO_Port GPIOE
#define ENCODER_B2_EXTI_IRQn EXTI9_5_IRQn
#define ENCODER_A2_Pin GPIO_PIN_13
#define ENCODER_A2_GPIO_Port GPIOE
#define ENCODER_A2_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_IN_B2_Pin GPIO_PIN_11
#define MOTOR_IN_B2_GPIO_Port GPIOC
#define MOTOR_IN_A2_Pin GPIO_PIN_12
#define MOTOR_IN_A2_GPIO_Port GPIOC
#define MOTOR_IN_B1_Pin GPIO_PIN_9
#define MOTOR_IN_B1_GPIO_Port GPIOB
#define MOTOR_IN_A1_Pin GPIO_PIN_1
#define MOTOR_IN_A1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
