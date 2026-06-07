/*
 * ============================================================================
 * 文件名: usart.h
 *
 * 文件用途:
 *   本文件定义STM32F4 UART4串口通信驱动的函数接口和句柄变量。
 *   UART用于与上位机进行命令行通信、调试输出和数据采集。
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_UART4_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */


