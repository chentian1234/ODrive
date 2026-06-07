/*
 * ============================================================================
 * 文件名: stm32f4xx_it.h
 *
 * 文件用途:
 *   本文件定义所有中断服务函数的声明，包括Cortex-M4内核异常处理和STM32F4外设中断。
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void SysTick_Handler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);
void ADC_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void UART4_IRQHandler(void);
void OTG_FS_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */


