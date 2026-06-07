/*
 * ============================================================================
 * 文件名: stm32f4xx_it.c
 * 
 * 文件用途:
 *   本文件实现了STM32F4系列微控制器的中断服务例程(ISR)。主要处理Cortex-M4内核异常和STM32F4外设中断。
 *   包括ADC中断分发、USB中断延迟处理等功能。
 *
 * 中断服务分组:
 *   1. Cortex-M4内核异常: NMI、HardFault、MemManage、BusFault、UsageFault、SysTick
 *   2. DMA中断: DMA1_Stream2(UART4 RX)、DMA1_Stream4(UART4 TX)
 *   3. ADC中断: 通过ADC_IRQ_Dispatch分发到HAL回调函数
 *   4. TIM中断: TIM8_TRG_COM_TIM14中断处理
 *   5. UART中断: UART4中断处理
 *   6. USB中断: OTG_FS中断处理(延迟到线程处理)
 *   7. EXTI中断: EXTI0/2/4外部中断处理
 *
 * 特殊处理:
 *   - ADC中断: 不直接调用HAL处理函数，而是通过自定义分发器处理
 *   - USB中断: 使用信号量延迟到线程处理，避免在中断中执行耗时操作
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "freertos_vars.h"  /* FreeRTOS 信号量句柄声明 */
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
/*            Cortex-M4内核异常中断服务程序                                    */ 
/******************************************************************************/

/**
 * @brief 不可屏蔽中断(NMI)服务程序
 */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief 硬件错误(HardFault)中断服务程序
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
 * @brief 内存管理(MemManage)错误中断服务程序
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
 * @brief 总线错误(BusFault)中断服务程序
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
 * @brief 未定义指令/状态(UsageFault)中断服务程序
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
 * @brief 调试监视器(DebugMonitor)中断服务程序
 */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief 系统滴答(SysTick)定时器中断服务程序
 */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  xPortSysTickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx外设中断服务程序                                                  */
/* 以下函数是STM32F4xx外设的中断服务程序                                      */
/* 具体中断向量名称请参考启动文件(startup_stm32f4xx.s)                        */
/******************************************************************************/

/**
 * @brief DMA1 Stream2全局中断服务程序(UART4 RX DMA)
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
 * @brief DMA1 Stream4全局中断服务程序(UART4 TX DMA)
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
 * @brief ADC1、ADC2和ADC3全局中断服务程序
 * @brief ADC中断分发函数 - ADC_IRQHandler实现
 * 
 * 函数说明:
 * 由于ADC1/ADC2/ADC3共享同一个中断向量，本函数通过自定义分发器处理中断，
 * 而不是直接调用HAL_ADC_IRQHandler()。这样可以更灵活地控制中断处理流程。
 * ADC_IRQ_Dispatch()函数负责将中断分发到对应的回调函数。
 * 
 * 中断处理说明:
 * - hadc1: 用于vbus_sense_adc_cb()回调，处理总线电压采样中断
 * - hadc2: 用于pwm_trig_adc_cb()回调，处理PWM触发的电机电流采样中断
 * - hadc3: 用于pwm_trig_adc_cb()回调，处理PWM触发的电机电流采样中断
 * 
 * 返回值说明:
 * 直接返回而不调用HAL处理函数，因为ADC_IRQ_Dispatch已经处理了所有中断标志。
 * HAL库的ADC中断处理函数会执行额外的状态检查和回调调用，但在此应用中不需要这些功能。
 */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  // 使用自定义分发器处理ADC中断，而不是直接调用HAL处理函数
  //@TODO 简化处理adc1总线电压采样
  ADC_IRQ_Dispatch(&hadc1, &vbus_sense_adc_cb);
  ADC_IRQ_Dispatch(&hadc2, &pwm_trig_adc_cb);
  ADC_IRQ_Dispatch(&hadc3, &pwm_trig_adc_cb);

  // 直接返回而不调用HAL处理函数
  return;

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
 * @brief TIM8触发/换相/更新中断和TIM14全局中断服务程序
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
 * @brief UART4全局中断服务程序
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
 * @brief USB OTG FS全局中断服务程序
 */
/**
 * @brief USB中断处理函数
 * 
 * 函数说明:
 * 当USB OTG FS中断触发时，执行以下操作:
 * 1. 立即禁用USB中断，防止中断嵌套
 * 2. 释放sem_usb_irq信号量，通知usb_update_thread线程处理
 * 3. 中断处理完成后立即返回，实际的USB事件处理在线程中完成
 * 4. usb_update_thread线程处理完成后会重新使能USB中断
 * 
 * 设计原因:
 * - USB中断处理较为耗时，不适合在中断上下文中执行
 * - 使用信号量+线程的方式可以提高系统实时性
 * - 中断处理完成后需要手动重新使能中断，否则会丢失后续中断
 */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  // 禁用USB中断，等待usb_update_thread线程处理完成后再使能
  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  xSemaphoreGiveFromISR(sem_usb_irq, NULL);
  // 立即返回，实际的USB处理在线程中完成
  return;

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
 * @brief ADC中断分发函数
 * 
 * 函数功能:
 * 检查ADC的中断标志位，如果中断已使能且标志置位，则调用对应的回调函数。
 * 支持注入转换(Injected)和规则转换(Regular)两种模式的中断处理。
 */
void ADC_IRQ_Dispatch(ADC_HandleTypeDef* hadc, ADC_handler_t callback) {

  // 检查注入转换结束标志
  uint32_t JEOC = __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOC);
  uint32_t JEOC_IT_EN = __HAL_ADC_GET_IT_SOURCE(hadc, ADC_IT_JEOC);
  if (JEOC && JEOC_IT_EN) {
    callback(hadc, true);
    __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_JSTRT | ADC_FLAG_JEOC));
  }
  // 检查规则转换结束标志
  uint32_t EOC = __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC);
  uint32_t EOC_IT_EN = __HAL_ADC_GET_IT_SOURCE(hadc, ADC_IT_EOC);
  if (EOC && EOC_IT_EN) {
    callback(hadc, false);
    __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_STRT | ADC_FLAG_EOC));
  }
}


/**
 * @brief EXTI line0外部中断服务程序
 */
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
 * @brief EXTI line2外部中断服务程序
 */
void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/**
 * @brief EXTI line4外部中断服务程序
 */
void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/* USER CODE END 1 */
