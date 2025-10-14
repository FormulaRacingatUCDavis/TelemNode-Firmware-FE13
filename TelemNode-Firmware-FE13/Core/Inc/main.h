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
#define EXTRA_SENSOR_Pin GPIO_PIN_0
#define EXTRA_SENSOR_GPIO_Port GPIOA
#define STRAIN_GAUGE_CONTROL_Pin GPIO_PIN_1
#define STRAIN_GAUGE_CONTROL_GPIO_Port GPIOA
#define SHOCK_ANGLE_Pin GPIO_PIN_2
#define SHOCK_ANGLE_GPIO_Port GPIOA
#define STRAIN_GAUGE_UF_Pin GPIO_PIN_3
#define STRAIN_GAUGE_UF_GPIO_Port GPIOA
#define STRAIN_GAUGE_UB_Pin GPIO_PIN_4
#define STRAIN_GAUGE_UB_GPIO_Port GPIOA
#define STRAIN_GAUGE_LF_Pin GPIO_PIN_5
#define STRAIN_GAUGE_LF_GPIO_Port GPIOA
#define STRAIN_GAUGE_LB_Pin GPIO_PIN_6
#define STRAIN_GAUGE_LB_GPIO_Port GPIOA
#define IR_BRAKE_TEMP_Pin GPIO_PIN_7
#define IR_BRAKE_TEMP_GPIO_Port GPIOA
#define K_BRAKE_TEMP_Pin GPIO_PIN_0
#define K_BRAKE_TEMP_GPIO_Port GPIOB
#define STRAIN_GAUGE_PUSH_Pin GPIO_PIN_1
#define STRAIN_GAUGE_PUSH_GPIO_Port GPIOB
#define HEARTBEAT_Pin GPIO_PIN_12
#define HEARTBEAT_GPIO_Port GPIOB
#define WHEEL_SPEED_Pin GPIO_PIN_8
#define WHEEL_SPEED_GPIO_Port GPIOA
#define WHEEL_1_Pin GPIO_PIN_4
#define WHEEL_1_GPIO_Port GPIOB
#define WHEEL_0_Pin GPIO_PIN_6
#define WHEEL_0_GPIO_Port GPIOB
#define WHEEL_0_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
