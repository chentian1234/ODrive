/*
 * ============================================================================
 * 文件名: tim.c
 *
 * 文件用途:
 *   本文件实现STM32F4定时器外设的配置，涵盖PWM输出、编码器接口和ADC触发功能。
 *   定时器是ODrive电机控制系统的核心，负责生成PWM波形驱动MOSFET桥臂，
 *   同时精确控制ADC采样时机以获取电机电流。
 *
 * 主要功能模块：
 *   1. TIM1：M0电机PWM输出（高级定时器，三相+死区+ADC触发）
 *   2. TIM8：M1电机PWM输出（高级定时器，三相+死区+ADC触发）
 *   3. TIM2：辅助驱动器PWM输出（低/高端驱动）
 *   4. TIM3：M0电机编码器接口（4倍频，4级滤波）
 *   5. TIM4：M1电机编码器接口（4倍频，4级滤波）
 *   6. OC4_PWM_Override()：配置OC4通道为PWM模式用于ADC触发
 *   7. MSP层初始化/去初始化：PWM输出引脚配置、编码器引脚配置
 *
 * 定时器分工:
 *   - TIM1/TIM8：中央对齐PWM模式，产生FOC所需的对称PWM波形
 *   - TIM1 TRGO：触发ADC注入转换（M0电流采样）
 *   - TIM8 TRGO：触发ADC常规转换（M1电流采样）
 *   - TIM3/TIM4：硬件编码器模式，自动计算脉冲数和方向
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/**
 * @brief OC4通道PWM输出配置函数
 * 
 * 功能说明：
 * 配置定时器的通道4(OC4)为PWM2模式，用于触发ADC采样。
 * 
 * 为什么需要这个函数：
 * 在电机控制中，ADC需要在PWM周期的特定时刻采样电流信号。
 * 通过定时器比较输出(OC4 PWM)可以精确控制ADC的触发时机。
 * 
 * 注意：CubeMX不允许在没有输出引脚的情况下配置PWM模式，
 * 所以此函数绕过CubeMX限制，直接配置OC4为PWM模式。
 * 比较寄存器设为1(不能为0，否则无法触发)。
 * 
 * @param htim: 定时器句柄指针
 */
void OC4_PWM_Override(TIM_HandleTypeDef* htim) {

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_OC_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_4);
}

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

/**
 * @brief TIM1 初始化函数
 * 
 * 功能说明：
 * 配置TIM1高级定时器，用于M0电机的PWM输出和ADC触发。
 * 
 * 主要配置：
 * - 计数器模式: 中央对齐模式3 (CENTERALIGNED3)
 * - 周期: TIM_1_8_PERIOD_CLOCKS (由电机控制频率决定)
 * - PWM通道: CH1/CH2/CH3 用于驱动M0电机三相
 * - CH4: 配置为PWM模式用于ADC触发
 * - 主输出触发: TIM_TRGO_UPDATE (更新事件触发)
 * - 死区时间: TIM_1_8_DEADTIME_CLOCKS
 * - 刹车: 禁用
 */
void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = TIM_1_8_PERIOD_CLOCKS;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = TIM_1_8_DEADTIME_CLOCKS;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}
/**
 * @brief TIM2 初始化函数
 * 
 * 功能说明：
 * 配置TIM2通用定时器，用于辅助驱动器的PWM输出。
 * 
 * 主要配置：
 * - 计数器模式: 中央对齐模式3 (CENTERALIGNED3)
 * - 周期: TIM_APB1_PERIOD_CLOCKS (APB1频率相关)
 * - CH3: PWM输出用于辅助驱动器低侧
 * - CH4: PWM输出用于辅助驱动器高侧 (特殊脉宽控制)
 */
void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim2.Init.Period = TIM_APB1_PERIOD_CLOCKS;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = TIM_APB1_PERIOD_CLOCKS+1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}
/**
 * @brief TIM3 初始化函数
 * 
 * 功能说明：
 * 配置TIM3通用定时器为编码器模式，用于读取M0电机的编码器信号。
 * 
 * 主要配置：
 * - 编码器模式: TIM_ENCODERMODE_TI12 (TI1和TI2都计数，4倍频)
 * - IC1: 通道1输入捕获，上升沿触发，直接TI输入，4级滤波
 * - IC2: 通道2输入捕获，上升沿触发，直接TI输入，4级滤波
 * - 周期: 0xFFFF (16位最大值)
 * - 滤波器: 4 (抑制编码器信号噪声)
 * 
 * 注意：编码器模式自动计算脉冲数和方向，无需软件干预
 */
void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/**
 * @brief TIM4 初始化函数
 * 
 * 功能说明：
 * 配置TIM4通用定时器为编码器模式，用于读取M1电机的编码器信号。
 * 配置与TIM3相同，用于M1电机的编码器接口。
 * 
 * 主要配置：
 * - 编码器模式: TIM_ENCODERMODE_TI12 (TI1和TI2都计数，4倍频)
 * - IC1/IC2: 输入捕获，上升沿触发，4级滤波
 * - 周期: 0xFFFF (16位最大值)
 */
void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/**
 * @brief TIM8 初始化函数
 * 
 * 功能说明：
 * 配置TIM8高级定时器，用于M1电机的PWM输出。
 * 与TIM1功能类似，但用于第二个电机轴。
 * 
 * 主要配置：
 * - 计数器模式: 中央对齐模式3 (CENTERALIGNED3)
 * - 周期: TIM_1_8_PERIOD_CLOCKS (与TIM1相同)
 * - PWM通道: CH1/CH2/CH3 用于驱动M1电机三相
 * - 主输出触发: TIM_TRGO_UPDATE (用于触发ADC采样)
 * - 死区时间: TIM_1_8_DEADTIME_CLOCKS
 * - 中断: 使能TIM8_TRG_COM_TIM14_IRQn (优先级0)
 * 
 * 注意：TIM8的中断用于ADC采样触发和时基更新
 */
void MX_TIM8_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim8.Init.Period = TIM_1_8_PERIOD_CLOCKS;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = TIM_1_8_DEADTIME_CLOCKS;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief TIM基础定时器底层初始化回调函数
 * 
 * 功能说明：
 * 当调用HAL_TIM_Base_Init()时，HAL库会自动调用此函数来配置TIM1的底层硬件资源。
 * 主要使能TIM1时钟，用于电机M0的PWM输出控制。
 * 
 * @param tim_baseHandle: 定时器句柄指针
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* 使能TIM1时钟 */
    __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}

/**
 * @brief TIM PWM底层初始化回调函数
 * 
 * 功能说明：
 * 当调用HAL_TIM_PWM_Init()时，HAL库会自动调用此函数来配置TIM2/TIM8的底层硬件资源。
 * - TIM2: 使能时钟，用于辅助驱动器PWM输出
 * - TIM8: 使能时钟并配置中断(优先级0)，用于电机M1的PWM输出
 * 
 * @param tim_pwmHandle: 定时器句柄指针
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* 使能TIM2时钟 */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_pwmHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspInit 0 */

  /* USER CODE END TIM8_MspInit 0 */
    /* 使能TIM8时钟 */
    __HAL_RCC_TIM8_CLK_ENABLE();

    /* 使能TIM8中断 - 优先级0(最高) */
    HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
  /* USER CODE BEGIN TIM8_MspInit 1 */

  /* USER CODE END TIM8_MspInit 1 */
  }
}

/**
 * @brief TIM编码器模式底层初始化回调函数
 * 
 * 功能说明：
 * 当调用HAL_TIM_Encoder_Init()时，HAL库会自动调用此函数来配置TIM3/TIM4的底层硬件资源。
 * - TIM3: 配置PB4/PB5为复用功能，连接M0编码器的A/B相信号
 * - TIM4: 配置PB6/PB7为复用功能，连接M1编码器的A/B相信号
 * 
 * @param tim_encoderHandle: 定时器句柄指针
 */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(tim_encoderHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* 使能TIM3时钟 */
    __HAL_RCC_TIM3_CLK_ENABLE();
  
    /**TIM3 GPIO配置    
    PB4     ------> TIM3_CH1 (M0编码器A相)
    PB5     ------> TIM3_CH2 (M0编码器B相)
    */
    GPIO_InitStruct.Pin = M0_ENC_A_Pin|M0_ENC_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(tim_encoderHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* 使能TIM4时钟 */
    __HAL_RCC_TIM4_CLK_ENABLE();
  
    /**TIM4 GPIO配置    
    PB6     ------> TIM4_CH1 (M1编码器A相)
    PB7     ------> TIM4_CH2 (M1编码器B相)
    */
    GPIO_InitStruct.Pin = M1_ENC_A_Pin|M1_ENC_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
}

/**
 * @brief TIM GPIO后置初始化回调函数
 * 
 * 功能说明：
 * 在定时器初始化完成后调用，用于配置各定时器的PWM输出引脚。
 * - TIM1: 配置M0电机的三相高/低端PWM输出(PA8-10, PB13-15)
 * - TIM2: 配置辅助驱动器的低/高端PWM输出(PB10-11)
 * - TIM8: 配置M1电机的三相高/低端PWM输出(PC6-8, PA7, PB0-1)
 * 
 * @param timHandle: 定时器句柄指针
 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
    /**TIM1 GPIO配置    
    PB13     ------> TIM1_CH1N (M0 A相低端)
    PB14     ------> TIM1_CH2N (M0 B相低端)
    PB15     ------> TIM1_CH3N (M0 C相低端)
    PA8     ------> TIM1_CH1  (M0 A相高端)
    PA9     ------> TIM1_CH2  (M0 B相高端)
    PA10     ------> TIM1_CH3 (M0 C相高端)
    */
    GPIO_InitStruct.Pin = M0_AL_Pin|M0_BL_Pin|M0_CL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M0_AH_Pin|M0_BH_Pin|M0_CH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */
  
    /**TIM2 GPIO配置    
    PB10     ------> TIM2_CH3 (辅助低端)
    PB11     ------> TIM2_CH4 (辅助高端)
    */
    GPIO_InitStruct.Pin = AUX_L_Pin|AUX_H_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspPostInit 0 */

  /* USER CODE END TIM8_MspPostInit 0 */
  
    /**TIM8 GPIO配置    
    PA7     ------> TIM8_CH1N (M1 A相低端)
    PB0     ------> TIM8_CH2N (M1 B相低端)
    PB1     ------> TIM8_CH3N (M1 C相低端)
    PC6     ------> TIM8_CH1  (M1 A相高端)
    PC7     ------> TIM8_CH2  (M1 B相高端)
    PC8     ------> TIM8_CH3  (M1 C相高端)
    */
    GPIO_InitStruct.Pin = M1_AL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(M1_AL_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_BL_Pin|M1_CL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_AH_Pin|M1_BH_Pin|M1_CH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM8_MspPostInit 1 */

  /* USER CODE END TIM8_MspPostInit 1 */
  }

}

/**
 * @brief TIM基础定时器底层去初始化回调函数
 * 
 * 功能说明：
 * 当调用HAL_TIM_Base_DeInit()时，HAL库会自动调用此函数来释放TIM1的底层硬件资源。
 * 
 * @param tim_baseHandle: 定时器句柄指针
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* 禁用外设时钟 */
    __HAL_RCC_TIM1_CLK_DISABLE();
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}

/**
 * @brief TIM PWM底层去初始化回调函数
 * 
 * 功能说明：
 * 当调用HAL_TIM_PWM_DeInit()时，HAL库会自动调用此函数来释放TIM2/TIM8的底层硬件资源。
 * - TIM2: 禁用时钟
 * - TIM8: 禁用时钟并关闭中断
 * 
 * @param tim_pwmHandle: 定时器句柄指针
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* 禁用外设时钟 */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_pwmHandle->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspDeInit 0 */

  /* USER CODE END TIM8_MspDeInit 0 */
    /* 禁用外设时钟 */
    __HAL_RCC_TIM8_CLK_DISABLE();

    /* 关闭TIM8中断 */
    HAL_NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn);
  /* USER CODE BEGIN TIM8_MspDeInit 1 */

  /* USER CODE END TIM8_MspDeInit 1 */
  }
}

/**
 * @brief TIM编码器模式底层去初始化回调函数
 * 
 * 功能说明：
 * 当调用HAL_TIM_Encoder_DeInit()时，HAL库会自动调用此函数来释放TIM3/TIM4的底层硬件资源。
 * 包括禁用时钟和复位GPIO引脚配置。
 * 
 * @param tim_encoderHandle: 定时器句柄指针
 */
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* tim_encoderHandle)
{

  if(tim_encoderHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* 禁用外设时钟 */
    __HAL_RCC_TIM3_CLK_DISABLE();
  
    /**TIM3 GPIO配置    
    PB4     ------> TIM3_CH1 (M0编码器A相)
    PB5     ------> TIM3_CH2 (M0编码器B相)
    */
    HAL_GPIO_DeInit(GPIOB, M0_ENC_A_Pin|M0_ENC_B_Pin);

  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(tim_encoderHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* 禁用外设时钟 */
    __HAL_RCC_TIM4_CLK_DISABLE();
  
    /**TIM4 GPIO配置    
    PB6     ------> TIM4_CH1 (M1编码器A相)
    PB7     ------> TIM4_CH2 (M1编码器B相)
    */
    HAL_GPIO_DeInit(GPIOB, M1_ENC_A_Pin|M1_ENC_B_Pin);

  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
}  

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

