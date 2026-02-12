/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ModbusSlaveRTU.h"
#include "StepperMotorController.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static uint32_t tim7_autoreloadTimer = 0;
uint32_t getTim7Time()
{
    return tim7_autoreloadTimer +  LL_TIM_GetCounter(TIM7);
    // uint32_t arr = LL_TIM_GetAutoReload(TIM7);
    // uint32_t cnt = LL_TIM_GetCounter(TIM7);
    // *time_us = (tim7_autoreloadTimer + cnt) * 1000 / (SystemCoreClock / (LL_TIM_GetPrescaler(TIM7) + 1)) ;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	ModbusRTU_dmaTxCpltCallback(ModbusRTUInstance(), &xHigherPriorityTaskWoken);
	
  /* USER CODE END DMA1_Channel2_3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles TIM6, DAC1 and LPTIM1 interrupts (LPTIM1 interrupt through EXTI line 29).
  */
void TIM6_DAC_LPTIM1_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_LPTIM1_IRQn 0 */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  StepperMotorController* controller = getStepperMotorController();  
  //controller->updateMotors(controller);
  controller->notifyTaskISR(controller, &xHigherPriorityTaskWoken);

  /* USER CODE END TIM6_DAC_LPTIM1_IRQn 0 */

  /* USER CODE BEGIN TIM6_DAC_LPTIM1_IRQn 1 */

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  /* USER CODE END TIM6_DAC_LPTIM1_IRQn 1 */
}

/**
  * @brief This function handles TIM7 and LPTIM2 interrupts (LPTIM2 interrupt through EXTI line 30).
  */
void TIM7_LPTIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_LPTIM2_IRQn 0 */
  if(LL_TIM_IsActiveFlag_UPDATE(TIM7)){
    LL_TIM_ClearFlag_UPDATE(TIM7);

    tim7_autoreloadTimer += LL_TIM_GetAutoReload(TIM7);
  }
  
  /* USER CODE END TIM7_LPTIM2_IRQn 0 */

  /* USER CODE BEGIN TIM7_LPTIM2_IRQn 1 */

  /* USER CODE END TIM7_LPTIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	ModbusRTU_IDLECallback(ModbusRTUInstance(), &xHigherPriorityTaskWoken);
    
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
