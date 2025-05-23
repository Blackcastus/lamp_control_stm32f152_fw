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
#include "stm32l1xx_hal.h"

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
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define CHARGING_Pin GPIO_PIN_5
#define CHARGING_GPIO_Port GPIOC
#define LED_SOS_Pin GPIO_PIN_6
#define LED_SOS_GPIO_Port GPIOC
#define COI_Pin GPIO_PIN_8
#define COI_GPIO_Port GPIOC
#define RDM6300_Pin GPIO_PIN_9
#define RDM6300_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_10
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define B2_Pin GPIO_PIN_12
#define B2_GPIO_Port GPIOC
#define B2_EXTI_IRQn EXTI15_10_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define INPUT_WIRELESS_Pin GPIO_PIN_8
#define INPUT_WIRELESS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define BLINK_LED_TIMES 100
#define TURN_ON GPIO_PIN_SET
#define TURN_OFF GPIO_PIN_RESET
#define BUZZER_CONTROL(x) 	HAL_GPIO_WritePin(COI_GPIO_Port, COI_Pin, x)
#define RFID_CONTROL(x) 	HAL_GPIO_WritePin(RDM6300_GPIO_Port, RDM6300_Pin, x)
#define LED_SOS_CONTROL(x)	HAL_GPIO_WritePin(LED_SOS_GPIO_Port, LED_SOS_Pin, x)
#define CHARGE_CONTROL(x)   HAL_GPIO_WritePin(CHARGING_GPIO_Port, CHARGING_Pin, x)
#define IS_IRQ_ENABLED(IRQn)  ((NVIC->ISER[(IRQn) >> 5] & (1 << ((IRQn) & 0x1F))) != 0)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
