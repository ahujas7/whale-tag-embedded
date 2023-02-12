/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define EOC_Pin GPIO_PIN_2
#define EOC_GPIO_Port GPIOC
#define IMU_NRST_Pin GPIO_PIN_0
#define IMU_NRST_GPIO_Port GPIOA
#define VHF_AF_OUT_Pin GPIO_PIN_3
#define VHF_AF_OUT_GPIO_Port GPIOA
#define VHF_PD_Pin GPIO_PIN_5
#define VHF_PD_GPIO_Port GPIOA
#define VHF_SQ_Pin GPIO_PIN_6
#define VHF_SQ_GPIO_Port GPIOA
#define VHF_PTT_Pin GPIO_PIN_7
#define VHF_PTT_GPIO_Port GPIOA
#define VHF_H_L_Pin GPIO_PIN_0
#define VHF_H_L_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_6
#define IMU_INT_GPIO_Port GPIOC
#define GPS_SAFEBOOT_Pin GPIO_PIN_7
#define GPS_SAFEBOOT_GPIO_Port GPIOC
#define GPS_EXTINT_Pin GPIO_PIN_8
#define GPS_EXTINT_GPIO_Port GPIOC
#define GPS_nRESET_Pin GPIO_PIN_9
#define GPS_nRESET_GPIO_Port GPIOC
#define GPS_LNA_EN_Pin GPIO_PIN_8
#define GPS_LNA_EN_GPIO_Port GPIOA
#define Flash_CS_Pin GPIO_PIN_15
#define Flash_CS_GPIO_Port GPIOA
#define FPGA_CS_Pin GPIO_PIN_2
#define FPGA_CS_GPIO_Port GPIOD
#define AudioSD_CS_Pin GPIO_PIN_4
#define AudioSD_CS_GPIO_Port GPIOB
#define SensorsSD_CS_Pin GPIO_PIN_5
#define SensorsSD_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
