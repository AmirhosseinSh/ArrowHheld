/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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
#define LEDR_ON()		HAL_GPIO_WritePin(LEDR_GPIO_Port,LEDR_Pin,GPIO_PIN_SET);
#define LEDR_OFF()	HAL_GPIO_WritePin(LEDR_GPIO_Port,LEDR_Pin,GPIO_PIN_RESET);
#define LEDG_ON()		HAL_GPIO_WritePin(LEDG_GPIO_Port,LEDG_Pin,GPIO_PIN_SET);
#define LEDG_OFF()	HAL_GPIO_WritePin(LEDG_GPIO_Port,LEDG_Pin,GPIO_PIN_RESET);
#define BUCK_ON()		HAL_GPIO_WritePin(EN_BUCK_GPIO_Port,EN_BUCK_Pin,GPIO_PIN_SET);
#define BUCK_OFF()	HAL_GPIO_WritePin(EN_BUCK_GPIO_Port,EN_BUCK_Pin,GPIO_PIN_RESET);
#define LASER_ON()	HAL_GPIO_WritePin(LASER_EN_GPIO_Port,LASER_EN_Pin,GPIO_PIN_SET);
#define LASER_OFF()	HAL_GPIO_WritePin(LASER_EN_GPIO_Port,LASER_EN_Pin,GPIO_PIN_RESET);

#define BOOT0_OFF()	HAL_GPIO_WritePin(BOOT0_CTRL_GPIO_Port,BOOT0_CTRL_Pin,GPIO_PIN_RESET);
#define BOOT0_ON()	HAL_GPIO_WritePin(BOOT0_CTRL_GPIO_Port,BOOT0_CTRL_Pin,GPIO_PIN_SET);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BOOT0_CTRL_Pin GPIO_PIN_14
#define BOOT0_CTRL_GPIO_Port GPIOC
#define BTN_Pin GPIO_PIN_0
#define BTN_GPIO_Port GPIOA
#define BTN_EXTI_IRQn EXTI0_1_IRQn
#define LASER_EN_Pin GPIO_PIN_4
#define LASER_EN_GPIO_Port GPIOA
#define EN_BUCK_Pin GPIO_PIN_5
#define EN_BUCK_GPIO_Port GPIOA
#define LEDG_Pin GPIO_PIN_6
#define LEDG_GPIO_Port GPIOA
#define LEDR_Pin GPIO_PIN_7
#define LEDR_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
