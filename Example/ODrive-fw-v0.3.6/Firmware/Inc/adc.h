/*
 * ============================================================================
 * 文件名: adc.h
 *
 * 文件用途:
 *   本文件定义STM32F4 ADC外设驱动层的函数接口和句柄变量。
 *   ADC用于电机电流采样、直流母线电压采样和辅助信号采集。
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __adc_H
#define __adc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

float read_ADC_volts(ADC_HandleTypeDef* hadc, uint8_t injected_rank);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/**
  * @}
  */

/**
  * @}
  */


