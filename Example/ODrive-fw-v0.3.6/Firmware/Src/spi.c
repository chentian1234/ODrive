/*
 * ============================================================================
 * 文件名: spi.c
 *
 * 文件用途:
 *   本文件实现STM32F4 SPI3外设的驱动配置，用于与SPI编码器通信。
 *
 * 主要功能模块：
 *   1. SPI3初始化：主模式、16位数据宽度、时钟极性/相位配置
 *   2. MSP层初始化/去初始化：PC10(SCK)/PC11(MISO)/PC12(MOSI)引脚配置
 *
 * 通信参数:
 *   - 工作模式: SPI主模式
 *   - 数据宽度: 16位（适用于SPI编码器）
 *   - 时钟极性: 空闲低电平
 *   - 时钟相位: 第二个时钟沿采样
 *   - 波特率预分频: 16分频
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi3;

/**
 * @brief SPI3 初始化函数
 * 
 * 功能说明：
 * 配置SPI3外设用于与外部设备通信（如SPI编码器等）。
 * 
 * 主要配置参数：
 * - 工作模式: SPI_MODE_MASTER (主模式)
 * - 数据线: SPI_DIRECTION_2LINES (双线全双工)
 * - 数据宽度: SPI_DATASIZE_16BIT (16位数据)
 * - 时钟极性: SPI_POLARITY_LOW (空闲时低电平)
 * - 时钟相位: SPI_PHASE_2EDGE (第二个时钟沿采样)
 * - NSS: SPI_NSS_SOFT (软件管理片选)
 * - 波特率预分频: SPI_BAUDRATEPRESCALER_16 (16分频)
 * - 数据传输顺序: SPI_FIRSTBIT_MSB (高位在前)
 * 
 * 注意：使用16位数据宽度，适用于某些需要16位数据格式的SPI编码器
 */
void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
 * @brief SPI外设底层初始化回调函数（由HAL库自动调用）
 * 
 * 功能说明：
 * 当调用HAL_SPI_Init()时，HAL库会自动调用此函数来配置SPI的底层硬件资源。
 * 
 * GPIO配置：
 * - PC10: SPI3_SCK (时钟信号)
 * - PC11: SPI3_MISO (主入从出)
 * - PC12: SPI3_MOSI (主出从入)
 * - 复用功能: GPIO_AF6_SPI3
 * - 速度: GPIO_SPEED_FREQ_VERY_HIGH (高速)
 * 
 * @param spiHandle: SPI句柄指针，指向要初始化的SPI实例
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* 使能SPI3时钟 */
    __HAL_RCC_SPI3_CLK_ENABLE();
  
    /**SPI3 GPIO配置    
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

/**
 * @brief SPI外设底层去初始化回调函数（由HAL库自动调用）
 * 
 * 功能说明：
 * 当调用HAL_SPI_DeInit()时，HAL库会自动调用此函数来释放SPI的底层硬件资源。
 * 包括禁用SPI时钟、复位GPIO引脚配置等。
 * 
 * @param spiHandle: SPI句柄指针，指向要去初始化的SPI实例
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* 禁用SPI3外设时钟 */
    __HAL_RCC_SPI3_CLK_DISABLE();
  
    /**SPI3 GPIO配置    
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
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

