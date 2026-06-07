/*
 * ============================================================================
 * 文件名: adc.c
 *
 * 文件用途:
 *   本文件实现STM32F4 ADC外设的驱动层，用于电机电流采样、直流母线电压采样
 *   和辅助信号采集。支持常规转换和注入转换两种模式。
 *
 * 主要功能模块：
 *   1. ADC1配置：M1电机电流采样（常规通道）+ M0电机电流采样（注入通道）
 *   2. ADC2配置：M1电机电流采样（常规通道）+ M0电机电流采样（注入通道）
 *   3. ADC3配置：直流母线电压采样（常规通道）+ 电机电流采样（注入通道）
 *   4. MSP层初始化/去初始化：GPIO模拟输入配置、ADC中断配置
 *   5. read_ADC_volts()：读取ADC原始值并转换为实际电压值
 *
 * ADC触发机制:
 *   - TIM1 TRGO触发注入转换（用于M0电机FOC电流采样）
 *   - TIM8 TRGO触发常规转换（用于M1电机FOC电流采样）
 *   - 在PWM周期中心点对齐时刻采样，避开开关噪声
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 1 \
||  HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 2
#include "prev_board_ver/adc_V3_2.c"
#else
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

/* ADC1 初始化函数
 * 用途: 配置ADC1用于电机电流采样，支持常规转换和注入转换两种模式
 * - 常规转换: 由T8定时器触发，用于采样M1电机相关信号
 * - 注入转换: 由T1定时器触发，用于采样M0电机电流信号
 */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

  /* 配置ADC全局参数（时钟、分辨率、数据对齐和转换次数） */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* 配置ADC1常规通道及其在序列器中的 rank 和采样时间 */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* 配置ADC1注入通道及其在序列器中的 rank 和采样时间 */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* ADC2 初始化函数
 * 用途: 配置ADC2用于电机电流采样
 * - 常规转换: 由T8定时器触发，采样M1电机电流（通道13）
 * - 注入转换: 由T1定时器触发，采样M0电机电流（通道10）
 */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

  /* 配置ADC全局参数（时钟、分辨率、数据对齐和转换次数） */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* 配置ADC2常规通道及其在序列器中的 rank 和采样时间 */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* 配置ADC2注入通道及其在序列器中的 rank 和采样时间 */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* ADC3 初始化函数
 * 用途: 配置ADC3用于直流母线电压采样和其他模拟信号采集
 * - 常规转换: 由T8定时器触发，采样母线电压相关信号（通道12）
 * - 注入转换: 由T1定时器触发，采样电机电流信号（通道11）
 */
void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

  /* 配置ADC全局参数（时钟、分辨率、数据对齐和转换次数） */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* 配置ADC3常规通道及其在序列器中的 rank 和采样时间 */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* 配置ADC3注入通道及其在序列器中的 rank 和采样时间 */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
 * @brief ADC外设底层初始化回调函数（由HAL库自动调用）
 * 
 * 功能说明：
 * 当调用HAL_ADC_Init()时，HAL库会自动调用此函数来配置ADC的底层硬件资源。
 * 根据不同的ADC实例（ADC1/ADC2/ADC3），分别配置对应的GPIO引脚和中断。
 * 
 * GPIO配置说明：
 * - PC0-PC5: 配置为模拟输入模式，用于采集电机电流信号（M0_IB, M0_IC, M1_IC, M1_IB）
 *            以及辅助驱动器温度（AUX_TEMP）和M0电机温度（M0_TEMP）
 * - PA4-PA6: 配置为模拟输入模式，用于采集M1电机温度（M1_TEMP）、辅助驱动电流（AUX_I）
 *            和直流母线电压（VBUS_S）
 * 
 * 中断配置：
 * - ADC_IRQn: 优先级设为5，用于ADC转换完成中断
 * 
 * @param adcHandle: ADC句柄指针，指向要初始化的ADC实例
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* 使能ADC1时钟 */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO配置    
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PC4     ------> ADC1_IN14
    PC5     ------> ADC1_IN15 
    */
    GPIO_InitStruct.Pin = M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin 
                          |AUX_TEMP_Pin|M0_TEMP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_TEMP_Pin|AUX_I_Pin|VBUS_S_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1中断初始化 */
    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* 使能ADC2时钟 */
    __HAL_RCC_ADC2_CLK_ENABLE();
  
    /**ADC2 GPIO配置    
    PC0     ------> ADC2_IN10
    PC1     ------> ADC2_IN11
    PC2     ------> ADC2_IN12
    PC3     ------> ADC2_IN13
    PA4     ------> ADC2_IN4
    PA5     ------> ADC2_IN5
    PA6     ------> ADC2_IN6
    PC4     ------> ADC2_IN14
    PC5     ------> ADC2_IN15 
    */
    GPIO_InitStruct.Pin = M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin 
                          |AUX_TEMP_Pin|M0_TEMP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_TEMP_Pin|AUX_I_Pin|VBUS_S_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2中断初始化 */
    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* 使能ADC3时钟 */
    __HAL_RCC_ADC3_CLK_ENABLE();
  
    /**ADC3 GPIO配置    
    PC0     ------> ADC3_IN10
    PC1     ------> ADC3_IN11
    PC2     ------> ADC3_IN12
    PC3     ------> ADC3_IN13 
    */
    GPIO_InitStruct.Pin = M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC3中断初始化 */
    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

/**
 * @brief ADC外设底层去初始化回调函数（由HAL库自动调用）
 * 
 * 功能说明：
 * 当调用HAL_ADC_DeInit()时，HAL库会自动调用此函数来释放ADC的底层硬件资源。
 * 包括禁用ADC时钟、复位GPIO引脚配置、禁用中断等。
 * 
 * @param adcHandle: ADC句柄指针，指向要去初始化的ADC实例
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* 禁用ADC1外设时钟 */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO配置    
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PC4     ------> ADC1_IN14
    PC5     ------> ADC1_IN15 
    */
    HAL_GPIO_DeInit(GPIOC, M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin 
                          |AUX_TEMP_Pin|M0_TEMP_Pin);

    HAL_GPIO_DeInit(GPIOA, M1_TEMP_Pin|AUX_I_Pin|VBUS_S_Pin);

    /* ADC1中断去初始化 */
  /* USER CODE BEGIN ADC1:ADC_IRQn disable */
    /**
    * 取消注释以下行可禁用"ADC_IRQn"中断
    * 注意：禁用共享中断可能影响其他外设
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC1:ADC_IRQn disable */

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* 禁用ADC2外设时钟 */
    __HAL_RCC_ADC2_CLK_DISABLE();
  
    /**ADC2 GPIO配置    
    PC0     ------> ADC2_IN10
    PC1     ------> ADC2_IN11
    PC2     ------> ADC2_IN12
    PC3     ------> ADC2_IN13
    PA4     ------> ADC2_IN4
    PA5     ------> ADC2_IN5
    PA6     ------> ADC2_IN6
    PC4     ------> ADC2_IN14
    PC5     ------> ADC2_IN15 
    */
    HAL_GPIO_DeInit(GPIOC, M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin 
                          |AUX_TEMP_Pin|M0_TEMP_Pin);

    HAL_GPIO_DeInit(GPIOA, M1_TEMP_Pin|AUX_I_Pin|VBUS_S_Pin);

    /* ADC2中断去初始化 */
  /* USER CODE BEGIN ADC2:ADC_IRQn disable */
    /**
    * 取消注释以下行可禁用"ADC_IRQn"中断
    * 注意：禁用共享中断可能影响其他外设
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC2:ADC_IRQn disable */

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* 禁用ADC3外设时钟 */
    __HAL_RCC_ADC3_CLK_DISABLE();
  
    /**ADC3 GPIO配置    
    PC0     ------> ADC3_IN10
    PC1     ------> ADC3_IN11
    PC2     ------> ADC3_IN12
    PC3     ------> ADC3_IN13 
    */
    HAL_GPIO_DeInit(GPIOC, M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin);

    /* ADC3中断去初始化 */
  /* USER CODE BEGIN ADC3:ADC_IRQn disable */
    /**
    * 取消注释以下行可禁用"ADC_IRQn"中断
    * 注意：禁用共享中断可能影响其他外设
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC3:ADC_IRQn disable */

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
#endif  /* ADC包含结束 */

/**
 * @brief 读取ADC转换结果并转换为电压值
 * 
 * 功能说明：
 * 根据ADC句柄和转换类型（常规或注入），读取ADC转换结果并将其转换为实际电压值。
 * 转换公式: voltage = (3.3V / 4096) * ADC_Value
 * 其中3.3V是ADC参考电压，4096是12位ADC的最大值(2^12)
 * 
 * @param hadc: ADC句柄指针，指向要读取的ADC实例
 * @param injected_rank: 注入通道序号，如果为0则读取常规转换结果
 * @return float: 转换后的电压值（单位：伏特）
 */
float read_ADC_volts(ADC_HandleTypeDef* hadc, uint8_t injected_rank) {
    uint32_t ADCValue;
    if(injected_rank) {
        /* 读取注入转换结果 */
        ADCValue = HAL_ADCEx_InjectedGetValue(hadc, injected_rank);
    } else {
        /* 读取常规转换结果 */
        ADCValue = HAL_ADC_GetValue(hadc);
    }
    /* 将ADC原始值转换为电压值: 3.3V参考电压 / 4096(12位分辨率) */
    return (3.3f/((float)(1<<12))) * ADCValue;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

