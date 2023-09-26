/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include <stdarg.h>
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
#define PB_PC13_Pin GPIO_PIN_13
#define PB_PC13_GPIO_Port GPIOC
#define PB_PC13_EXTI_IRQn EXTI15_10_IRQn
#define PB_PH0_Pin GPIO_PIN_0
#define PB_PH0_GPIO_Port GPIOH
#define PB_PH0_EXTI_IRQn EXTI0_IRQn
#define PB_PH1_Pin GPIO_PIN_1
#define PB_PH1_GPIO_Port GPIOH
#define PB_PH1_EXTI_IRQn EXTI1_IRQn
#define ADC_RED_Pin GPIO_PIN_0
#define ADC_RED_GPIO_Port GPIOC
#define ADC_WHITE_Pin GPIO_PIN_1
#define ADC_WHITE_GPIO_Port GPIOC
#define PB_PC2_Pin GPIO_PIN_2
#define PB_PC2_GPIO_Port GPIOC
#define PB_PC2_EXTI_IRQn EXTI2_IRQn
#define PB_PC3_Pin GPIO_PIN_3
#define PB_PC3_GPIO_Port GPIOC
#define PB_PC3_EXTI_IRQn EXTI3_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define USER_LED_4_Pin GPIO_PIN_6
#define USER_LED_4_GPIO_Port GPIOC
#define USER_LED_3_Pin GPIO_PIN_7
#define USER_LED_3_GPIO_Port GPIOC
#define USER_LED_2_Pin GPIO_PIN_8
#define USER_LED_2_GPIO_Port GPIOC
#define USER_LED_1_Pin GPIO_PIN_9
#define USER_LED_1_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define PB_PC10_Pin GPIO_PIN_10
#define PB_PC10_GPIO_Port GPIOC
#define PB_PC10_EXTI_IRQn EXTI15_10_IRQn
#define PB_PC11_Pin GPIO_PIN_11
#define PB_PC11_GPIO_Port GPIOC
#define PB_PC11_EXTI_IRQn EXTI15_10_IRQn
#define PB_PC12_Pin GPIO_PIN_12
#define PB_PC12_GPIO_Port GPIOC
#define PB_PC12_EXTI_IRQn EXTI15_10_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void hex_dump(uint8_t *buffer, uint8_t len);
void uart_printf(const char *fmt, ...);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
