/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define HX710_DT_Pin GPIO_PIN_4
#define HX710_DT_GPIO_Port GPIOC
#define HX710_CK_Pin GPIO_PIN_5
#define HX710_CK_GPIO_Port GPIOC
#define BUZZ_Pin GPIO_PIN_2
#define BUZZ_GPIO_Port GPIOB
#define SPI2_SS_Pin GPIO_PIN_12
#define SPI2_SS_GPIO_Port GPIOB
#define OUT6_Pin GPIO_PIN_8
#define OUT6_GPIO_Port GPIOA
#define OUT5_Pin GPIO_PIN_11
#define OUT5_GPIO_Port GPIOA
#define OUT4_Pin GPIO_PIN_12
#define OUT4_GPIO_Port GPIOA
#define OUT3_Pin GPIO_PIN_15
#define OUT3_GPIO_Port GPIOA
#define OUT2_Pin GPIO_PIN_10
#define OUT2_GPIO_Port GPIOC
#define OUT1_Pin GPIO_PIN_11
#define OUT1_GPIO_Port GPIOC
#define IN7_Pin GPIO_PIN_12
#define IN7_GPIO_Port GPIOC
#define IN6_Pin GPIO_PIN_2
#define IN6_GPIO_Port GPIOD
#define IN5_Pin GPIO_PIN_3
#define IN5_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_4
#define IN4_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_5
#define IN3_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_6
#define IN2_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_7
#define IN1_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_8
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_9
#define I2C_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define YComUart huart1
#define YComUartInstance USART1
#define PCUartInstance USART3
#define PCUart huart3
#define debug
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
