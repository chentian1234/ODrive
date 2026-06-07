/*
 * ============================================================================
 * 文件名: stm32f4xx_hal_timebase_TIM.c
 *
 * 文件用途:
 *   本文件使用TIM14定时器替代默认的SysTick作为HAL库的时基源，产生1ms周期中断。
 *   使用定时器作为时基可以在低功耗模式下继续运行（SysTick在睡眠模式下会停止）。
 *
 * 主要功能模块：
 *   1. HAL_InitTick()：配置TIM14为1ms时基，计算预分频和周期值
 *   2. HAL_SuspendTick()：暂停tick递增（禁用TIM14更新中断）
 *   3. HAL_ResumeTick()：恢复tick递增（使能TIM14更新中断）
 *
 * 时基计算:
 *   TIM14时钟 = APB1 × 2 = 84MHz
 *   预分频 = 84-1 → 1MHz计数器时钟
 *   周期 = 1000-1 = 999 → 1ms中断周期
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup HAL_TimeBase
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim14; 
uint32_t                 uwIncrementState = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief 初始化TIM14作为HAL库时基源
  * 
  * 功能说明：
  * 配置TIM14定时器产生1ms周期的中断，作为HAL库的时间基准。
  * 使用TIM14代替默认的Systick作为时基，可以在低功耗模式下继续运行。
  * 
  * 配置参数：
  * - TIM14时钟：APB1时钟的2倍（约84MHz）
  * - 预分频器：84-1 = 83，得到1MHz计数器时钟
  * - 周期：(1MHz/1000)-1 = 999，得到1ms中断周期
  * - 中断优先级：由TickPriority参数指定
  * 
  * @param  TickPriority: 时基中断优先级
  * @retval HAL状态
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;
  
  /* 配置TIM14中断优先级 */
  HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, TickPriority ,0); 
  
  /* 使能TIM14全局中断 */
  HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn); 
  
  /* 使能TIM14时钟 */
  __HAL_RCC_TIM14_CLK_ENABLE();
  
  /* 获取时钟配置 */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  
  /* 计算TIM14时钟频率（APB1时钟的2倍） */
  uwTimclock = 2*HAL_RCC_GetPCLK1Freq();
   
  /* 计算预分频值，使TIM14计数器时钟为1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000) - 1);
  
  /* 初始化TIM14 */
  htim14.Instance = TIM14;
  
  /* 初始化TIM14外设：
  + Period = [(TIM14CLK/1000) - 1]，产生1ms时间基准
  + Prescaler = (uwTimclock/1000000 - 1)，得到1MHz计数器时钟
  + ClockDivision = 0
  + Counter direction = Up（向上计数）
  */
  htim14.Init.Period = (1000000 / 1000) - 1;
  htim14.Init.Prescaler = uwPrescalerValue;
  htim14.Init.ClockDivision = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htim14) == HAL_OK)
  {
    /* 启动TIM14中断模式 */
    return HAL_TIM_Base_Start_IT(&htim14);
  }
  
  /* 返回函数状态 */
  return HAL_ERROR;
}

/**
  * @brief 暂停Tick递增
  * @note   通过禁用TIM14更新中断来暂停tick递增
  * @param  无
  * @retval 无
  */
void HAL_SuspendTick(void)
{
  /* 禁用TIM14更新中断 */
  __HAL_TIM_DISABLE_IT(&htim14, TIM_IT_UPDATE);                                                  
}

/**
  * @brief 恢复Tick递增
  * @note   通过使能TIM14更新中断来恢复tick递增
  * @param  无
  * @retval 无
  */
void HAL_ResumeTick(void)
{
  /* 使能TIM14更新中断 */
  __HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);
}

/**
  * @}
  */ 

/**
  * @}
  */ 


