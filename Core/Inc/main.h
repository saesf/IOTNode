/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "queue.h"
#include "magnetometer.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern int wifiState;
extern int wifiReady;
extern uint8_t magnetometerPayload[14];
extern char *payload;
extern uint8_t magFlag;
extern uint8_t magDMA;
extern uint8_t memTomemTransferCompleteFlag;
extern uint8_t lsdm6dslDmaCpltFlg;
extern uint8_t lsdm6dslDataReadyFlg;


extern uint32_t sensorEvents;
#define MIC_0_DMA_HALF_COMPLETE (uint32_t)(1<<0)
#define MIC_0_DMA_FULL_COMPLETE (uint32_t)(1<<1)
#define MIC_1_DMA_HALF_COMPLETE (uint32_t)(1<<2)
#define MIC_1_DMA_FULL_COMPLETE (uint32_t)(1<<3)


extern uint8_t *payloadBuffers[BUFFER_Q_SIZE];
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
#define XNUCLEO_INT1_Pin GPIO_PIN_3
#define XNUCLEO_INT1_GPIO_Port GPIOA
#define XNUCLEO_INT1_EXTI_IRQn EXTI3_IRQn
#define WIFI_RESET_Pin GPIO_PIN_8
#define WIFI_RESET_GPIO_Port GPIOE
#define WIFI_BOOT_Pin GPIO_PIN_12
#define WIFI_BOOT_GPIO_Port GPIOB
#define LSDM6DSL_INT1_EXTI11_Pin GPIO_PIN_11
#define LSDM6DSL_INT1_EXTI11_GPIO_Port GPIOD
#define LSDM6DSL_INT1_EXTI11_EXTI_IRQn EXTI15_10_IRQn
#define TOF_RESET_Pin GPIO_PIN_6
#define TOF_RESET_GPIO_Port GPIOC
#define LIS3MDL_DRDY_EXTI8_Pin GPIO_PIN_8
#define LIS3MDL_DRDY_EXTI8_GPIO_Port GPIOC
#define LIS3MDL_DRDY_EXTI8_EXTI_IRQn EXTI9_5_IRQn
#define LED_BLUE_Pin GPIO_PIN_9
#define LED_BLUE_GPIO_Port GPIOC
#define WIFI_CS_Pin GPIO_PIN_0
#define WIFI_CS_GPIO_Port GPIOE
#define WIFI_INT_Pin GPIO_PIN_1
#define WIFI_INT_GPIO_Port GPIOE
#define WIFI_INT_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
