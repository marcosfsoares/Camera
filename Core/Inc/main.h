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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OV7670_HREF_Pin GPIO_PIN_13
#define OV7670_HREF_GPIO_Port GPIOC
#define OV7670_PCLK_Pin GPIO_PIN_0
#define OV7670_PCLK_GPIO_Port GPIOC
#define OV7670_VSYNC_Pin GPIO_PIN_2
#define OV7670_VSYNC_GPIO_Port GPIOC
#define OV7670_D0_Pin GPIO_PIN_3
#define OV7670_D0_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define OV7670_D1_Pin GPIO_PIN_4
#define OV7670_D1_GPIO_Port GPIOC
#define OV7670_D2_Pin GPIO_PIN_5
#define OV7670_D2_GPIO_Port GPIOC
#define OV7670_XCLK_Pin GPIO_PIN_6
#define OV7670_XCLK_GPIO_Port GPIOC
#define OV7670_D3_Pin GPIO_PIN_8
#define OV7670_D3_GPIO_Port GPIOC
#define OV7670_D4_Pin GPIO_PIN_9
#define OV7670_D4_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define OV7670_D5_Pin GPIO_PIN_10
#define OV7670_D5_GPIO_Port GPIOC
#define OV7670_D6_Pin GPIO_PIN_11
#define OV7670_D6_GPIO_Port GPIOC
#define OV7670_D7_Pin GPIO_PIN_12
#define OV7670_D7_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_6
#define SD_CS_GPIO_Port GPIOB
#define MCU_6050_SCL__OV7670_SIOC_Pin GPIO_PIN_8
#define MCU_6050_SCL__OV7670_SIOC_GPIO_Port GPIOB
#define MCU_6050_SDA__OV7670_SIOD_Pin GPIO_PIN_9
#define MCU_6050_SDA__OV7670_SIOD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
