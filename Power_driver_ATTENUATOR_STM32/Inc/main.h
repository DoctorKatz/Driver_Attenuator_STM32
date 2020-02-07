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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drivers.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
		uint8_t command;
		uint8_t channel;
		uint8_t data;
} hostDataTypeDef;
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
#define SPI3_SLAVE_SCK_Pin GPIO_PIN_1
#define SPI3_SLAVE_SCK_GPIO_Port GPIOA
#define SPI3_SLAVE_MISO_Pin GPIO_PIN_2
#define SPI3_SLAVE_MISO_GPIO_Port GPIOA
#define SPI3_SLAVE_MOSI_Pin GPIO_PIN_3
#define SPI3_SLAVE_MOSI_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_1
#define RED_LED_GPIO_Port GPIOB
#define SPI2_MASTER_MOSI_Pin GPIO_PIN_15
#define SPI2_MASTER_MOSI_GPIO_Port GPIOB
#define SPI2_MASTER_SCK_Pin GPIO_PIN_8
#define SPI2_MASTER_SCK_GPIO_Port GPIOB
#define SPI2_MASTER_CS_Pin GPIO_PIN_9
#define SPI2_MASTER_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

//#define _DEBUG

#define CHECK_CONNECTION				0x01
#define RECEIVE_SERIAL_NUM		0x02
#define TRANSMIT_TO_DAC					0x03

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
