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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

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
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define A111_SPI_SCK_Pin GPIO_PIN_5
#define A111_SPI_SCK_GPIO_Port GPIOA
#define A111_SPI_MISO_Pin GPIO_PIN_6
#define A111_SPI_MISO_GPIO_Port GPIOA
#define A111_SPI_MOSI_Pin GPIO_PIN_7
#define A111_SPI_MOSI_GPIO_Port GPIOA
#define XC111_SPI_SS1_Pin GPIO_PIN_10
#define XC111_SPI_SS1_GPIO_Port GPIOB
#define A111_SENSOR_INTERRUPT_Pin GPIO_PIN_8
#define A111_SENSOR_INTERRUPT_GPIO_Port GPIOA
#define A111_SENSOR_INTERRUPT_EXTI_IRQn EXTI9_5_IRQn
#define A111_RESET_N_Pin GPIO_PIN_9
#define A111_RESET_N_GPIO_Port GPIOA
#define XC111_PMU_ENABLE_Pin GPIO_PIN_10
#define XC111_PMU_ENABLE_GPIO_Port GPIOA
#define DEBUG_USART_TX_Pin GPIO_PIN_10
#define DEBUG_USART_TX_GPIO_Port GPIOC
#define DEBUG_USART_RX_Pin GPIO_PIN_11
#define DEBUG_USART_RX_GPIO_Port GPIOC
#define XC111_SPI_SS0_Pin GPIO_PIN_4
#define XC111_SPI_SS0_GPIO_Port GPIOB
#define A111_ENABLE_Pin GPIO_PIN_5
#define A111_ENABLE_GPIO_Port GPIOB
#define A111_CS_N_Pin GPIO_PIN_6
#define A111_CS_N_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#ifndef EXAMPLE_SERVICE_ENVELOPE_H_
#define EXAMPLE_SERVICE_ENVELOPE_H_
#endif

#define A111_SPI_HANDLE   hspi1
#define MS_UART_HANDLE    huart2
#define DEBUG_UART_HANDLE huart3
#define A111_USE_SPI_DMA
#define STM32CUBE_MOP_BUFFER_SIZE 32000

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/