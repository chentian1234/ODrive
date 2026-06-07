/*
 * ============================================================================
 * 文件名: stm32f4xx_hal_msp.c
 *
 * 文件用途:
 *   本文件实现HAL库的MSP（MCU Support Package）初始化，配置全局中断优先级分组
 *   和系统异常的优先级。HAL_Init()会自动调用HAL_MspInit()。
 *
 * 主要功能模块：
 *   1. HAL_MspInit()：全局MSP初始化
 *      - 设置中断优先级分组为NVIC_PRIORITYGROUP_4（4位抢占优先级）
 *      - 配置系统异常优先级（MemoryManagement/BusFault/UsageFault = 0）
 *      - PendSV/SysTick设为最低优先级（15），用于FreeRTOS任务切换
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

extern void _Error_Handler(char *, int);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * @brief 全局MSP初始化函数
  * 
  * 功能说明：
  * 此函数由HAL_Init()自动调用，用于配置全局的硬件相关初始化。
  * 主要设置中断优先级分组和各系统异常的优先级。
  * 
  * 中断优先级配置：
  * - NVIC_PRIORITYGROUP_4: 4位抢占优先级，0位子优先级
  * - MemoryManagement/BusFault/UsageFault: 优先级0（最高）
  * - SVCall/DebugMonitor: 优先级0
  * - PendSV: 优先级15（最低），用于FreeRTOS任务切换
  * - SysTick: 优先级15（最低），用于系统时基
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  /* 设置中断优先级分组为4位抢占优先级 */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* 系统异常中断优先级配置 */
  /* 内存管理异常 - 最高优先级 */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* 总线错误异常 - 最高优先级 */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* 用法错误异常 - 最高优先级 */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall异常 - 最高优先级 */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* 调试监视器异常 - 最高优先级 */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV异常 - 最低优先级，用于FreeRTOS任务切换 */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
  /* SysTick异常 - 最低优先级，用于系统时基 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

