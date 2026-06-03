/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* 
     * HAL初始化：重置所有外设、初始化Flash接口和Systick定时器。
     * 这是STM32 HAL库的标准入口，必须首先调用。
     * 主要工作包括：
     *   - 配置Flash延迟（等待状态）
     *   - 初始化Systick为1ms中断（后续会被SystemClock_Config重新配置）
     *   - 设置中断优先级分组
     *   - 初始化HAL库内部状态
     */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* 
     * 系统时钟配置：
     * 配置STM32F4的时钟树，使用外部高速晶振（HSE）作为时钟源，
     * 通过PLL倍频至168MHz系统时钟，并配置AHB/APB总线分频器。
     * 详见SystemClock_Config函数。
     */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* 
     * 按顺序初始化所有配置的外设：
     * 
     * 1. MX_GPIO_Init()    - 初始化所有GPIO引脚（输入/输出模式、上下拉、速度等）
     * 2. MX_DMA_Init()     - 初始化DMA控制器（用于ADC、SPI等外设的高速数据传输）
     * 3. MX_ADC1_Init()    - 初始化ADC1（用于电机电流采样）
     * 4. MX_ADC2_Init()    - 初始化ADC2（用于电机电流采样）
     * 5. MX_CAN1_Init()    - 初始化CAN总线（用于与上位机/其他设备通信）
     * 6. MX_TIM1_Init()    - 初始化TIM1（高级定时器，用于PWM输出驱动电机）
     * 7. MX_TIM8_Init()    - 初始化TIM8（高级定时器，用于PWM输出驱动电机）
     * 8. MX_TIM3_Init()    - 初始化TIM3（通用定时器，用于编码器接口等）
     * 9. MX_TIM4_Init()    - 初始化TIM4（通用定时器，用于其他定时功能）
     * 10. MX_SPI3_Init()   - 初始化SPI3（用于外部通信，如SPI编码器）
     * 11. MX_ADC3_Init()   - 初始化ADC3（用于直流母线电压采样等）
     * 12. MX_TIM2_Init()   - 初始化TIM2（通用定时器，用于其他定时功能）
     * 13. MX_UART4_Init()  - 初始化UART4（用于串口通信/调试）
     * 
     * 注意：USB设备初始化未在CubeMX中配置，由代码手动初始化
     */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_CAN1_Init();
    MX_TIM1_Init();
    MX_TIM8_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_SPI3_Init();
    MX_ADC3_Init();
    MX_TIM2_Init();
    MX_UART4_Init();

    /* USER CODE BEGIN 2 */

    /*
     * OC4_PWM_Override 配置：
     * 用于配置TIM1和TIM8的通道4（OC4）输出，以实现ADC触发功能。
     * 在电机控制中，ADC需要在PWM周期的特定时刻采样电流信号，
     * 通过定时器比较输出（Compare Output）可以精确控制ADC的触发时机，
     * 确保在PWM有效期间进行电流采样，避免开关噪声干扰。
     * 
     * htim1: 控制电机轴0的PWM生成和ADC触发
     * htim8: 控制电机轴1的PWM生成和ADC触发
     */
    OC4_PWM_Override(&htim1);
    OC4_PWM_Override(&htim8);

    /* USER CODE END 2 */

    /* 
     * FreeRTOS初始化与启动流程：
     * 
     * 1. MX_FREERTOS_Init()：创建FreeRTOS任务、信号量、队列、互斥锁等内核对象
     *    - 创建电机控制任务（高优先级，实时控制）
     *    - 创建通信任务（CAN/UART/USB数据处理）
     *    - 创建系统监控任务等
     * 
     * 2. osKernelStart()：启动FreeRTOS调度器
     *    - 此后CPU控制权交给RTOS调度器
     *    - 根据任务优先级和调度策略运行各个任务
     *    - main函数不会再执行到后续的while(1)循环
     * 
     * 注意：如果osKernelStart()返回（理论上不应该），下面的while(1)是安全保护
     */
    MX_FREERTOS_Init();

    /* 启动FreeRTOS调度器，此后系统进入多任务调度状态 */
    osKernelStart();

    /* 正常情况下永远不会到达这里，因为控制权已交给FreeRTOS调度器 */

    /* 安全保护循环，防止调度器异常返回 */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */

}

/**
  * 系统时钟配置函数
  * 
  * 时钟树配置说明（基于STM32F405/407）：
  * 
  * 外部晶振（HSE）：8MHz（典型值，由硬件晶振决定）
  * 
  * PLL配置：
  *   - PLL_M = 4：   HSE 8MHz / 4 = 2MHz（VCO输入频率）
  *   - PLL_N = 168：  2MHz × 168 = 336MHz（VCO输出频率）
  *   - PLL_P = 2：    336MHz / 2 = 168MHz（系统时钟SYSCLK）
  *   - PLL_Q = 7：    336MHz / 7 = 48MHz（USB OTG FS时钟，必须为48MHz）
  * 
  * 总线时钟分频：
  *   - SYSCLK = 168MHz（系统主时钟）
  *   - HCLK = SYSCLK / 1 = 168MHz（AHB总线时钟，用于CPU、DMA、Flash等）
  *   - PCLK1 = HCLK / 4 = 42MHz（APB1总线时钟，用于TIM2-7/12-14、UART、SPI2/3等）
  *   - PCLK2 = HCLK / 2 = 84MHz（APB2总线时钟，用于TIM1/8、ADC、SPI1、USART1/6等）
  * 
  * Flash等待状态：FLASH_LATENCY_5（5个等待状态），确保CPU在168MHz下正确读取Flash
  * 
  * Systick配置：使用HCLK作为时钟源，中断频率 = 168MHz / 168000 = 1kHz（1ms周期）
  */
void SystemClock_Config(void) {

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /* 
     * 使能电源控制（PWR）时钟
     * PWR模块用于配置电压调节器，必须在配置电压缩放前使能
     */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* 
     * 配置电压调节器输出为Scale 1模式（最高性能模式）
     * STM32F4有三种电压调节模式：
     *   - Scale 1：最高性能，支持168MHz系统时钟
     *   - Scale 2：中等性能，支持144MHz系统时钟
     *   - Scale 3：低功耗模式，支持120MHz系统时钟
     * 由于我们需要168MHz，必须使用Scale 1
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* 
     * 配置振荡器（Oscillator）
     * 选择HSE（外部高速晶振）作为PLL时钟源，并启用PLL
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;   /* 选择HSE振荡器 */
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;                     /* 使能HSE */
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                 /* 使能PLL */
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;         /* PLL源选择HSE */
    RCC_OscInitStruct.PLL.PLLM = 4;                              /* PLL输入分频系数 */
    RCC_OscInitStruct.PLL.PLLN = 168;                            /* PLL倍频系数 */
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                  /* 系统时钟分频 */
    RCC_OscInitStruct.PLL.PLLQ = 7;                              /* USB时钟分频 */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);   /* 时钟配置失败，进入错误处理 */
    }

    /* 
     * 配置系统时钟总线分频器
     * 设置SYSCLK、HCLK、PCLK1、PCLK2的分频关系
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;    /* 选择PLL输出作为SYSCLK */
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;           /* AHB时钟 = SYSCLK / 1 = 168MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;            /* APB1时钟 = HCLK / 4 = 42MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;            /* APB2时钟 = HCLK / 2 = 84MHz */

    /* 
     * 应用时钟配置，FLASH_LATENCY_5表示Flash访问需要5个等待状态
     * 168MHz时需要5个等待状态（WS=5），以确保CPU正确读取Flash指令
     */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /* 
     * 配置Systick定时器中断周期
     * Systick频率 = HCLK / 1000 = 168MHz / 1000 = 168kHz
     * 这意味着Systick每秒中断1000次，即每1ms中断一次
     * 这是HAL库的时基，用于HAL_Delay()等延时函数
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /* 
     * 配置Systick时钟源为HCLK（168MHz）
     * 如果选择HCLK/8，则计数频率会降低8倍
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* 
     * 设置Systick中断优先级为15（最低优先级）
     * STM32F4使用4位优先级，范围0-15，15为最低
     * Systick通常设置为最低优先级，以免影响关键任务的实时响应
     */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * HAL定时器周期Elapsed回调函数
  * 
  * 作用说明：
  * 此函数是HAL库的弱定义回调函数，当任何定时器的更新事件（周期结束）发生时，
  * HAL_TIM_IRQHandler()内部会调用此回调函数。
  * 
  * SysTick时基更新机制：
  * 在本项目中，使用TIM14作为HAL库的时基定时器（而非默认的Systick），
  * 当TIM14中断触发时，调用HAL_IncTick()递增全局变量uwTick。
  * uwTick是HAL库的时间基准，HAL_Delay()等延时函数都依赖于此变量。
  * 
  * 注意：虽然SystemClock_Config中配置了Systick，但实际的HAL时基可能由TIM14提供，
  * 这是为了在低功耗模式下仍能保持时基运行（Systick在睡眠模式下会停止）。
  * 
  * @param  htim : 触发回调的定时器句柄
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM14) {
        HAL_IncTick();   /* TIM14中断触发，递增HAL时基计数器uwTick */
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
  * HAL错误处理函数
  * 
  * 作用说明：
  * 当HAL库函数返回错误状态（HAL_ERROR、HAL_TIMEOUT等）时，会调用此函数。
  * 这是STM32 HAL库的标准错误处理机制。
  * 
  * 当前实现：
  * 进入无限死循环，系统停止运行。这是一种"fail-stop"策略，
  * 防止在错误状态下继续执行导致更严重的问题（如电机失控）。
  * 
  * 实际应用中，可以在此添加：
  *   - 错误日志记录（通过串口输出错误信息）
  *   - 故障指示灯闪烁（LED）
  *   - 安全停机处理（关闭PWM输出、切断电机驱动）
  *   - 看门狗复位
  * 
  * @param  file : 发生错误的源文件名
  * @param  line : 发生错误的行号
  * @retval None
  */
void _Error_Handler(char * file, int line) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* 用户可以在此添加自定义的HAL错误处理逻辑 */
    while (1) {
        /* 死循环，系统停止运行，等待看门狗复位或手动复位 */
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
  * 断言失败处理函数
  * 
  * 作用说明：
  * 当USE_FULL_ASSERT宏定义启用时，HAL库和标准库中的assert_param()宏
  * 会在参数检查失败时调用此函数。这是一种调试辅助机制。
  * 
  * 与_Error_Handler的区别：
  *   - _Error_Handler：HAL库函数返回错误时调用（运行时错误）
  *   - assert_failed：参数验证失败时调用（编程错误/逻辑错误）
  * 
  * 使用场景：
  * 在开发阶段，可以通过此函数定位参数传递错误，例如：
  *   - 传入无效的GPIO引脚
  *   - 超出范围的定时器通道
  *   - 不合法的分频系数等
  * 
  * 示例实现（取消注释即可使用）：
  *   printf("断言失败: 文件 %s, 行号 %d\r\n", file, line);
  *   while(1);
  * 
  * 注意：在生产版本中通常不启用USE_FULL_ASSERT，以避免额外的代码开销
  * 
  * @param  file : 发生断言失败的源文件名
  * @param  line : 发生断言失败的行号
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* 用户可以在此添加自定义实现来报告文件名和行号，
       例如: printf("错误的参数值: 文件 %s, 行号 %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
