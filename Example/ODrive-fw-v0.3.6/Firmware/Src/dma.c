/*
 * ============================================================================
 * 文件名: dma.c
 *
 * 文件用途:
 *   本文件实现STM32F4 DMA控制器的配置，用于外设与内存之间的高速数据传输。
 *   DMA允许外设（如UART）直接与内存交换数据，无需CPU干预，降低系统负载。
 *
 * 主要功能模块：
 *   1. DMA1控制器初始化
 *   2. DMA1_Stream2配置：UART4接收（外设→内存，循环模式）
 *   3. DMA1_Stream4配置：UART4发送（内存→外设，正常模式）
 *   4. DMA中断优先级配置（优先级5）
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */
/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* 配置DMA控制器                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** 
 * @brief DMA控制器初始化函数
 * 
 * 功能说明：
 * 初始化DMA1控制器，配置DMA通道用于外设与内存之间的高速数据传输。
 * DMA允许外设（如UART、ADC、SPI等）直接与内存交换数据，无需CPU干预，
 * 从而大大降低CPU负载，提高系统性能。
 * 
 * 配置的DMA流：
 * - DMA1_Stream2: 用于UART4接收（外设到内存方向）
 *   - 中断优先级: 5
 * - DMA1_Stream4: 用于UART4发送（内存到外设方向）
 *   - 中断优先级: 5
 * 
 * 注意：具体的DMA通道配置（如数据宽度、传输模式等）在各外设的初始化函数中完成。
 */
void MX_DMA_Init(void) 
{
  /* 使能DMA1控制器时钟 */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA中断初始化 */
  /* DMA1_Stream2_IRQn中断配置 - 用于UART4接收 */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn中断配置 - 用于UART4发送 */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

