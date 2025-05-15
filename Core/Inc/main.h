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
#include "stm32f0xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Y_2_Pin GPIO_PIN_0
#define Y_2_GPIO_Port GPIOC
#define Y_1_Pin GPIO_PIN_1
#define Y_1_GPIO_Port GPIOC
#define Z_1_Pin GPIO_PIN_3
#define Z_1_GPIO_Port GPIOC
#define X1_1_Pin GPIO_PIN_0
#define X1_1_GPIO_Port GPIOA
#define X1_2_Pin GPIO_PIN_1
#define X1_2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define X2_1_Pin GPIO_PIN_4
#define X2_1_GPIO_Port GPIOA
#define X2_2_Pin GPIO_PIN_0
#define X2_2_GPIO_Port GPIOB
#define pulX1_Pin GPIO_PIN_10
#define pulX1_GPIO_Port GPIOB
#define SPI_OUT_Pin GPIO_PIN_12
#define SPI_OUT_GPIO_Port GPIOB
#define pulX2_Pin GPIO_PIN_7
#define pulX2_GPIO_Port GPIOC
#define dirX1_Pin GPIO_PIN_8
#define dirX1_GPIO_Port GPIOA
#define dirX2_Pin GPIO_PIN_9
#define dirX2_GPIO_Port GPIOA
#define dirZ_Pin GPIO_PIN_10
#define dirZ_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define pulZ_Pin GPIO_PIN_3
#define pulZ_GPIO_Port GPIOB
#define pulY_Pin GPIO_PIN_4
#define pulY_GPIO_Port GPIOB
#define dirY_Pin GPIO_PIN_5
#define dirY_GPIO_Port GPIOB
#define SPI_IN_Pin GPIO_PIN_6
#define SPI_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
