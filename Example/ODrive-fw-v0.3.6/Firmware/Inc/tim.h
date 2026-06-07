/*
 * ============================================================================
 * 文件名: tim.h
 *
 * 文件用途:
 *   本文件定义STM32F4定时器驱动的函数接口和句柄变量。
 *   定时器用于PWM输出、编码器接口和ADC触发功能，是电机控制系统的核心。
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM8_Init(void);
                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                                

/* USER CODE BEGIN Prototypes */

void OC4_PWM_Override(TIM_HandleTypeDef* htim);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */


