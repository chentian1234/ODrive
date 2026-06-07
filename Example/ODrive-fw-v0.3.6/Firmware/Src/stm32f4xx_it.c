/*
 * ============================================================================
 * 文件名: stm32f4xx_it.c
 *
 * 文件用途:
 *   本文件包含所有中断服务函数（ISR），包括Cortex-M4内核异常处理和STM32F4外设中断。
 *   采用自定义ADC中断分发机制和USB中断延迟处理，优化电机控制的实时性能。
 *
 * 主要功能模块：
 *   1. Cortex-M4异常处理：NMI、HardFault、MemManage、BusFault、UsageFault、SysTick
 *   2. DMA中断：DMA1_Stream2(UART4 RX)、DMA1_Stream4(UART4 TX)
 *   3. ADC中断：自定义分发器ADC_IRQ_Dispatch，绕过HAL库直接回调
 *   4. TIM中断：TIM8_TRG_COM_TIM14中断处理
 *   5. UART中断：UART4中断处理
 *   6. USB中断：OTG_FS中断，采用信号量延迟处理机制
 *   7. EXTI中断：EXTI0/2/4外部中断处理（步进信号）
 *
 * 性能优化:
 *   - ADC中断: 直接检查标志位并调用回调，绕过HAL库减少中断延迟
 *   - USB中断: 立即屏蔽中断并通过信号量唤醒线程，在用户空间处理
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include "freertos_vars.h"
#include "low_level.h"

typedef void (*ADC_handler_t)(ADC_HandleTypeDef* hadc, bool injected);
void ADC_IRQ_Dispatch(ADC_HandleTypeDef* hadc, ADC_handler_t callback);

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern UART_HandleTypeDef huart4;

extern TIM_HandleTypeDef htim14;

/******************************************************************************/
/*            Cortex-M4处理器中断和异常处理函数         */ 
/******************************************************************************/

/**
* @brief 不可屏蔽中断处理函数
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief 硬件错误中断处理函数
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief 内存管理错误中断处理函数
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief 总线错误中断处理函数
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief 非法指令或非法状态中断处理函数
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief 调试监视器中断处理函数
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief 系统滴答定时器中断处理函数
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx外设中断处理函数                                    */
/* 在此添加所使用外设的中断处理函数。                  */
/* 可用的外设中断处理函数名称，                      */
/* 请参考启动文件(startup_stm32f4xx.s)。                    */
/******************************************************************************/

/**
* @brief DMA1 Stream2全局中断处理函数
*/
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
* @brief DMA1 Stream4全局中断处理函数
*/
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
* @brief ADC1, ADC2和ADC3全局中断处理函数
* @brief ADC中断服务函数（被ADC_IRQHandler调用）
* 
* 功能说明：
* 处理ADC1/ADC2/ADC3的中断请求。为了减少HAL库的处理开销，
* 此函数绕过了标准的HAL_ADC_IRQHandler()，直接使用自定义的
* ADC_IRQ_Dispatch()函数来分发中断。
* 
* 中断分发逻辑：
* - hadc1: 调用vbus_sense_adc_cb()处理母线电压采样中断
* - hadc2: 调用pwm_trig_adc_cb()处理PWM触发的ADC采样中断
* - hadc3: 调用pwm_trig_adc_cb()处理PWM触发的ADC采样中断
* 
* 性能优化：
* HAL库的ADC中断处理会增加大量时钟周期，影响电机控制的实时性。
* 自定义处理只检查必要的标志位并立即调用回调，大幅减少中断延迟。
*/
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  // 绕过HAL库的ADC处理，直接使用自定义分发器
  //@TODO 在此添加adc1的vbus测量
  ADC_IRQ_Dispatch(&hadc1, &vbus_sense_adc_cb);
  ADC_IRQ_Dispatch(&hadc2, &pwm_trig_adc_cb);
  ADC_IRQ_Dispatch(&hadc3, &pwm_trig_adc_cb);

  // 绕过HAL库的标准处理流程
  return;

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
* @brief TIM8触发和换相中断以及TIM14全局中断处理函数
*/
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
* @brief UART4全局中断处理函数
*/
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
* @brief USB OTG FS全局中断处理函数
*/
/**
 * @brief USB中断服务函数
 * 
 * 功能说明：
 * 处理USB OTG FS的中断请求。采用延迟处理机制：
 * 1. 立即屏蔽USB中断，防止中断嵌套
 * 2. 释放sem_usb_irq信号量，唤醒usb_update_thread线程
 * 3. 线程会在用户空间处理所有待处理的USB中断
 * 4. 处理完成后，线程会重新使能USB中断
 * 
 * 优势：
 * - 减少中断处理时间，提高系统响应速度
 * - 所有USB处理在同一个线程中完成，避免并发问题
 * - 中断屏蔽防止了中断风暴
 */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  // 屏蔽中断，并通过信号量通知usb_cmd_thread处理
  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  osSemaphoreRelease(sem_usb_irq);
  // 绕过标准中断处理流程
  return;

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void ADC_IRQ_Dispatch(ADC_HandleTypeDef* hadc, ADC_handler_t callback) {

  // 注入通道测量
  uint32_t JEOC = __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOC);
  uint32_t JEOC_IT_EN = __HAL_ADC_GET_IT_SOURCE(hadc, ADC_IT_JEOC);
  if (JEOC && JEOC_IT_EN) {
    callback(hadc, true);
    __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_JSTRT | ADC_FLAG_JEOC));
  }
  // 常规通道测量
  uint32_t EOC = __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC);
  uint32_t EOC_IT_EN = __HAL_ADC_GET_IT_SOURCE(hadc, ADC_IT_EOC);
  if (EOC && EOC_IT_EN) {
    callback(hadc, false);
    __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_STRT | ADC_FLAG_EOC));
  }
}


/**
* @brief EXTI line0外部中断处理函数
*/
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
* @brief EXTI line2外部中断处理函数
*/
void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/**
* @brief EXTI line4外部中断处理函数
*/
void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/* USER CODE END 1 */

