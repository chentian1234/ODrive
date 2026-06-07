/*
 * ============================================================================
 * 文件名: gpio.h
 *
 * 文件用途:
 *   本文件定义STM32F4 GPIO引脚驱动的函数接口，包括输入/输出/中断引脚的初始化，
 *   以及GPIO引脚功能的动态切换（UART模式与Step/Dir模式）。
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

void SetGPIO12toUART();
void SetGPIO12toStepDir();
void SetupENCIndexGPIO();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */


