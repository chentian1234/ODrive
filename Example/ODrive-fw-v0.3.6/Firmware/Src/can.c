/*
 * ============================================================================
 * 文件名: can.c
 *
 * 文件用途:
 *   本文件实现STM32F4 CAN通信外设的驱动层，用于电机控制器与上位机或其他
 *   CAN节点之间的数据交换。
 *
 * 主要功能模块：
 *   1. CAN1初始化：波特率配置、工作模式设置（CAN_NORMAL正常模式）
 *   2. MSP层初始化/去初始化：PB8(RX)/PB9(TX)引脚复用配置
 *
 * 通信参数:
 *   - 波特率预分频器: 16
 *   - 总线时序: SJW=1TQ, BS1=1TQ, BS2=1TQ
 *   - 工作模式: 正常模式（支持收发）
 *   - ABOM(自动总线离线管理): 禁用，需软件手动恢复
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/**
 * @brief CAN1 初始化函数
 * 
 * 功能说明：
 * 配置CAN1外设用于电机控制器与上位机或其他设备的通信。
 * 波特率配置：
 *   - 时钟预分频器: 16
 *   - SJW(同步跳跃宽度): 1TQ
 *   - BS1(时间段1): 1TQ
 *   - BS2(时间段2): 1TQ
 *   - 工作模式: CAN_NORMAL(正常模式，支持发送和接收)
 * 
 * 注意：当前配置中ABOM(自动总线离线管理)被禁用，
 * 这意味着如果CAN总线出现错误，需要软件手动恢复。
 */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_1TQ;
  hcan1.Init.BS2 = CAN_BS2_1TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
 * @brief CAN外设底层初始化回调函数（由HAL库自动调用）
 * 
 * 功能说明：
 * 当调用HAL_CAN_Init()时，HAL库会自动调用此函数来配置CAN的底层硬件资源。
 * 
 * GPIO配置：
 * - PB8: CAN1_RX (接收引脚)
 * - PB9: CAN1_TX (发送引脚)
 * - 复用功能: GPIO_AF9_CAN1
 * - 速度: GPIO_SPEED_FREQ_VERY_HIGH (高速，适用于CAN通信)
 * 
 * @param canHandle: CAN句柄指针，指向要初始化的CAN实例
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* 使能CAN1时钟 */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO配置    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

/**
 * @brief CAN外设底层去初始化回调函数（由HAL库自动调用）
 * 
 * 功能说明：
 * 当调用HAL_CAN_DeInit()时，HAL库会自动调用此函数来释放CAN的底层硬件资源。
 * 包括禁用CAN时钟、复位GPIO引脚配置等。
 * 
 * @param canHandle: CAN句柄指针，指向要去初始化的CAN实例
 */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* 禁用CAN1外设时钟 */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO配置    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
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

