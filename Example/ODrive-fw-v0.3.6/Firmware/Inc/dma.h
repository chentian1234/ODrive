/*
 * ============================================================================
 * 文件名: dma.h
 *
 * 文件用途:
 *   本文件定义STM32F4 DMA控制器的函数接口，用于外设与内存之间的高速数据传输。
 *   DMA允许外设直接与内存交换数据，无需CPU干预，降低系统负载。
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __dma_H
#define __dma_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/
extern void _Error_Handler(char*, int);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_DMA_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __dma_H */

/**
  * @}
  */


