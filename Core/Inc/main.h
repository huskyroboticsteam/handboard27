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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Libraries/CAN26/CAN26.h"
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
#define H1_NSLEEP_Pin GPIO_PIN_13
#define H1_NSLEEP_GPIO_Port GPIOC
#define H1_DISABLE_Pin GPIO_PIN_0
#define H1_DISABLE_GPIO_Port GPIOC
#define H1_PH_Pin GPIO_PIN_1
#define H1_PH_GPIO_Port GPIOC
#define H1_EN_Pin GPIO_PIN_2
#define H1_EN_GPIO_Port GPIOC
#define H1_NFAULT_Pin GPIO_PIN_3
#define H1_NFAULT_GPIO_Port GPIOC
#define H2_NSLEEP_Pin GPIO_PIN_0
#define H2_NSLEEP_GPIO_Port GPIOA
#define H2_DISABLE_Pin GPIO_PIN_1
#define H2_DISABLE_GPIO_Port GPIOA
#define H2_PH_Pin GPIO_PIN_2
#define H2_PH_GPIO_Port GPIOA
#define H2_EN_Pin GPIO_PIN_3
#define H2_EN_GPIO_Port GPIOA
#define H2_NFAULT_Pin GPIO_PIN_4
#define H2_NFAULT_GPIO_Port GPIOA
#define STEPPER_SLEEP_Pin GPIO_PIN_5
#define STEPPER_SLEEP_GPIO_Port GPIOA
#define STEPPER_EN_Pin GPIO_PIN_6
#define STEPPER_EN_GPIO_Port GPIOA
#define STEPPER_DIR_Pin GPIO_PIN_7
#define STEPPER_DIR_GPIO_Port GPIOA
#define STEPPER_STEP_Pin GPIO_PIN_4
#define STEPPER_STEP_GPIO_Port GPIOC
#define STEPPER_M1_Pin GPIO_PIN_5
#define STEPPER_M1_GPIO_Port GPIOC
#define STEPPER_M0_Pin GPIO_PIN_1
#define STEPPER_M0_GPIO_Port GPIOB
#define STEPPER_FAULT_Pin GPIO_PIN_2
#define STEPPER_FAULT_GPIO_Port GPIOB
#define CAN_LED_Pin GPIO_PIN_11
#define CAN_LED_GPIO_Port GPIOB
#define FMOSFET_Pin GPIO_PIN_8
#define FMOSFET_GPIO_Port GPIOA
#define LMOSFET_Pin GPIO_PIN_9
#define LMOSFET_GPIO_Port GPIOA
#define LIMIT_SWITCH_Pin GPIO_PIN_12
#define LIMIT_SWITCH_GPIO_Port GPIOA
#define DBG_LED_Pin GPIO_PIN_4
#define DBG_LED_GPIO_Port GPIOB
#define ERR_LED_Pin GPIO_PIN_5
#define ERR_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
