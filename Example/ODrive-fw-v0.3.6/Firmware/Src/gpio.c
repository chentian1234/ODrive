/*
 * ============================================================================
 * 文件名: gpio.c
 *
 * 文件用途:
 *   本文件实现STM32F4 GPIO引脚的配置驱动，包括输入/输出/中断引脚的初始化，
 *   以及GPIO引脚功能的动态切换（UART模式与Step/Dir模式）。
 *
 * 主要功能模块：
 *   1. GPIO初始化：输出引脚（SPI片选、电流校准、栅极使能）
 *   2. 输入引脚配置（编码器Z相、故障检测、步进脉冲）
 *   3. 外部中断配置（步进信号、编码器索引）
 *   4. SetGPIO12toUART()：动态切换GPIO_1/2为UART功能
 *   5. SetGPIO12toStepDir()：动态切换GPIO_1/2为步进/方向功能
 *   6. SetupENCIndexGPIO()：配置编码器Z相中断
 *   7. HAL_GPIO_EXTI_Callback()：外部中断回调分发
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
#include "low_level.h"

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 1 \
||  HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 2
#include "prev_board_ver/gpio_V3_2.c"
#else
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* 配置GPIO */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** 
 * @brief GPIO初始化函数
 * 
 * 功能说明：
 * 配置所有GPIO引脚的工作模式，包括输入、输出、模拟、中断等。
 * 
 * 主要配置内容：
 * 1. 使能所有GPIO端口时钟（GPIOA/B/C/D/H）
 * 
 * 2. 输出引脚配置：
 *    - M0_nCS, M1_nCS: SPI片选信号，初始高电平（未选中）
 *    - M1_DC_CAL, M0_DC_CAL: 电流校准控制，初始低电平
 *    - EN_GATE: 栅极驱动器使能，初始低电平（禁用）
 *    - 模式：推挽输出，无上下拉，低速
 * 
 * 3. 输入引脚配置：
 *    - GPIO_3: 外部中断上升沿触发，下拉（用于步进脉冲输入）
 *    - GPIO_4, M0_ENC_Z: 浮空输入（M0编码器Z相信号）
 *    - GPIO_5, M1_ENC_Z: 浮空输入（M1编码器Z相信号）
 *    - nFAULT: 故障检测引脚，上拉（低电平表示故障）
 * 
 * 4. 外部中断配置：
 *    - EXTI2_IRQn: 优先级0（最高），用于GPIO_3步进信号
 * 
 * 注意：GPIO_1和GPIO_2可通过SetGPIO12toUART()/SetGPIO12toStepDir()动态切换功能
 */
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* 使能GPIO端口时钟 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* 配置GPIO输出引脚初始电平 */
  HAL_GPIO_WritePin(GPIOC, M0_nCS_Pin|M1_nCS_Pin, GPIO_PIN_SET);

  /* 配置GPIO输出引脚初始电平 */
  HAL_GPIO_WritePin(GPIOC, M1_DC_CAL_Pin|M0_DC_CAL_Pin, GPIO_PIN_RESET);

  /* 配置GPIO输出引脚初始电平 */
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);

  /* 配置GPIO输出引脚: PCPin PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = M0_nCS_Pin|M1_nCS_Pin|M1_DC_CAL_Pin|M0_DC_CAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 配置GPIO引脚: PtPin */
  GPIO_InitStruct.Pin = GPIO_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIO_3_GPIO_Port, &GPIO_InitStruct);

  /* 配置GPIO引脚: PAPin PAPin */
  GPIO_InitStruct.Pin = GPIO_4_Pin|M0_ENC_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 配置GPIO引脚: PBPin PBPin */
  GPIO_InitStruct.Pin = GPIO_5_Pin|M1_ENC_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 配置GPIO引脚: PtPin */
  GPIO_InitStruct.Pin = EN_GATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GATE_GPIO_Port, &GPIO_InitStruct);

  /* 配置GPIO引脚: PtPin */
  GPIO_InitStruct.Pin = nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(nFAULT_GPIO_Port, &GPIO_InitStruct);

  /* 外部中断初始化 */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 2 */
#endif // End GPIO Include

/**
 * @brief 将GPIO_1和GPIO_2切换为UART功能
 * 
 * 功能说明：
 * 将GPIO_1和GPIO_2配置为UART4的TX和RX引脚，用于串口通信。
 * 此函数通常在系统运行时动态切换GPIO功能时调用。
 * 
 * 配置内容：
 * - GPIO_1: UART4_TX (发送)，复用推挽输出，下拉
 * - GPIO_2: UART4_RX (接收)，复用推挽输出，无上下拉
 * - 复用功能: GPIO_AF8_UART4
 * - 速度: GPIO_SPEED_FREQ_VERY_HIGH
 * 
 * 注意：调用此函数前会先禁用EXTI0中断，以避免引脚模式切换时的误触发
 */
void SetGPIO12toUART() {
  GPIO_InitTypeDef GPIO_InitStruct;

  HAL_NVIC_DisableIRQ(EXTI0_IRQn);

  GPIO_InitStruct.Pin = GPIO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIO_1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIO_2_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief 将GPIO_1和GPIO_2切换为步进/方向控制功能
 * 
 * 功能说明：
 * 将GPIO_1和GPIO_2配置为步进电机的STEP和DIR信号输入。
 * - GPIO_1: STEP信号，外部中断上升沿触发，下拉
 * - GPIO_2: DIR信号，浮空输入
 * 
 * 中断配置：
 * - EXTI0_IRQn: 优先级0（最高），用于步进脉冲检测
 * 
 * 注意：此函数与SetGPIO12toUART()配合使用，实现引脚功能动态切换
 */
void SetGPIO12toStepDir() {
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = GPIO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIO_1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_2_GPIO_Port, &GPIO_InitStruct);

  // TODO: 硬编码的EXTI线路不可移植。应通过CubeMX设置EXTI默认值获取映射
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
 * @brief 配置编码器索引信号(Z相)的外部中断
 * 
 * 功能说明：
 * 配置M0和M1电机编码器的Z相信号（索引信号）为外部中断触发模式。
 * 编码器Z相在电机旋转一圈时触发一次，用于确定电机的绝对位置参考点。
 * 
 * 配置内容：
 * - M0_ENC_Z: 外部中断上升沿触发，无上下拉
 *   - 中断线: EXTI15_10_IRQn，优先级0
 * - M1_ENC_Z: 外部中断上升沿触发，无上下拉
 *   - 中断线: EXTI3_IRQn，优先级0
 * 
 * 注意：中断优先级设为0（最高），确保编码器索引信号能被及时处理
 */
void SetupENCIndexGPIO(){
  GPIO_InitTypeDef GPIO_InitStruct;

  /* 配置GPIO引脚: PAPin */
  GPIO_InitStruct.Pin = M0_ENC_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M0_ENC_Z_GPIO_Port, &GPIO_InitStruct);

  // TODO: 硬编码的EXTI线路不可移植。应通过CubeMX设置EXTI默认值获取映射
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* 配置GPIO引脚: PBPin */
  GPIO_InitStruct.Pin = M1_ENC_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M1_ENC_Z_GPIO_Port, &GPIO_InitStruct);

  // TODO: 硬编码的EXTI线路不可移植。应通过CubeMX设置EXTI默认值获取映射
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}


/**
 * @brief GPIO外部中断回调函数（由HAL库自动调用）
 * 
 * 功能说明：
 * 当GPIO引脚外部中断触发时，HAL库会调用此回调函数进行中断分发处理。
 * 根据不同的中断引脚，调用相应的处理函数：
 * 
 * 1. GPIO_1或GPIO_3中断：调用step_cb()处理步进脉冲信号
 *    - GPIO_1: M0电机STEP信号
 *    - GPIO_3: M1电机STEP信号
 * 
 * 2. M0_ENC_Z中断：调用enc_index_cb()处理M0编码器索引信号
 * 
 * 3. M1_ENC_Z中断：调用enc_index_cb()处理M1编码器索引信号
 * 
 * @param GPIO_Pin: 触发中断的GPIO引脚号
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // 步进信号M0和M1
  if (GPIO_Pin & GPIO_1_Pin || GPIO_Pin & GPIO_3_Pin) {
    step_cb(GPIO_Pin);
  } else if(GPIO_Pin & M0_ENC_Z_Pin){
    enc_index_cb(GPIO_Pin, 0);
  } else if(GPIO_Pin & M1_ENC_Z_Pin){
    enc_index_cb(GPIO_Pin, 1);
  }
}

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

