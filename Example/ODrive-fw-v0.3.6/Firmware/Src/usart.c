/*
 * ============================================================================
 * 文件名: usart.c
 *
 * 文件用途:
 *   本文件实现STM32F4 UART4串口通信外设的驱动配置，支持DMA传输以降低CPU负载。
 *   UART用于与上位机进行命令行通信、调试输出和数据采集。
 *
 * 主要功能模块：
 *   1. UART4初始化：115200波特率、8N1格式、发送+接收模式
 *   2. DMA接收配置：DMA1_Stream2，外设→内存，循环模式
 *   3. DMA发送配置：DMA1_Stream4，内存→外设，正常模式
 *   4. MSP层初始化/去初始化：GPIO引脚配置、DMA配置、中断配置
 *
 * 通信参数:
 *   - 波特率: 115200
 *   - 数据位: 8位
 *   - 停止位: 1位
 *   - 校验位: 无
 *   - 过采样: 16倍
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/**
 * @brief UART4 初始化函数
 * 
 * 功能说明：
 * 配置UART4用于串口通信，支持DMA传输以降低CPU负载。
 * 
 * 串口参数：
 * - 波特率: 115200
 * - 数据位: 8位
 * - 停止位: 1位
 * - 校验位: 无
 * - 工作模式: 发送+接收
 * - 过采样: 16倍过采样
 * 
 * DMA配置：
 * - UART4_RX: DMA1_Stream2，通道4，外设到内存，循环模式
 * - UART4_TX: DMA1_Stream4，通道4，内存到外设，正常模式
 * 
 * 注意：UART4的TX/RX引脚可通过SetGPIO12toUART()动态切换
 */
void MX_UART4_Init(void) {

    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle) {

    GPIO_InitTypeDef GPIO_InitStruct;
    if (uartHandle->Instance==UART4) {
        /* USER CODE BEGIN UART4_MspInit 0 */

        /* USER CODE END UART4_MspInit 0 */
        /* 使能UART4时钟 */
        __HAL_RCC_UART4_CLK_ENABLE();

        /**UART4 GPIO配置
        PA0-WKUP     ------> UART4_TX
        PA1     ------> UART4_RX
        */
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

        /* UART4 DMA初始化 */
        /* UART4接收DMA初始化 */
        hdma_uart4_rx.Instance = DMA1_Stream2;
        hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
        hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

        /* UART4发送DMA初始化 */
        hdma_uart4_tx.Instance = DMA1_Stream4;
        hdma_uart4_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart4_tx.Init.Mode = DMA_NORMAL;
        hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_uart4_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart4_tx);

        /* UART4中断初始化 */
        HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
        /* USER CODE BEGIN UART4_MspInit 1 */

        /* USER CODE END UART4_MspInit 1 */
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle) {

    if (uartHandle->Instance==UART4) {
        /* USER CODE BEGIN UART4_MspDeInit 0 */

        /* USER CODE END UART4_MspDeInit 0 */
        /* 禁用UART4外设时钟 */
        __HAL_RCC_UART4_CLK_DISABLE();

        /**UART4 GPIO配置
        PA0-WKUP     ------> UART4_TX
        PA1     ------> UART4_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_1_Pin|GPIO_2_Pin);

        /* UART4 DMA去初始化 */
        HAL_DMA_DeInit(uartHandle->hdmarx);
        HAL_DMA_DeInit(uartHandle->hdmatx);

        /* UART4中断去初始化 */
        HAL_NVIC_DisableIRQ(UART4_IRQn);
        /* USER CODE BEGIN UART4_MspDeInit 1 */

        /* USER CODE END UART4_MspDeInit 1 */
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

