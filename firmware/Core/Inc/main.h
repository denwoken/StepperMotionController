/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"

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
#define DRV1_DIR_Pin LL_GPIO_PIN_0
#define DRV1_DIR_GPIO_Port GPIOC
#define DRV1_NEN_Pin LL_GPIO_PIN_2
#define DRV1_NEN_GPIO_Port GPIOC
#define DRV5_NEN_Pin LL_GPIO_PIN_3
#define DRV5_NEN_GPIO_Port GPIOC
#define DRV5_DIR_Pin LL_GPIO_PIN_1
#define DRV5_DIR_GPIO_Port GPIOA
#define DRV2_DIR_Pin LL_GPIO_PIN_5
#define DRV2_DIR_GPIO_Port GPIOA
#define DRV2_NEN_Pin LL_GPIO_PIN_7
#define DRV2_NEN_GPIO_Port GPIOA
#define DRV4_NEN_Pin LL_GPIO_PIN_15
#define DRV4_NEN_GPIO_Port GPIOB
#define DRV4_DIR_Pin LL_GPIO_PIN_9
#define DRV4_DIR_GPIO_Port GPIOA
#define ENC_BTN_Pin LL_GPIO_PIN_8
#define ENC_BTN_GPIO_Port GPIOD
#define DRV3_NEN_Pin LL_GPIO_PIN_0
#define DRV3_NEN_GPIO_Port GPIOD
#define DRV3_DIR_Pin LL_GPIO_PIN_2
#define DRV3_DIR_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
