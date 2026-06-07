/*
 * ============================================================================
 * 文件名: usbd_cdc_if.c
 *
 * 文件用途:
 *   本文件实现USB CDC（Communication Device Class）接口的具体操作，包括
 *   数据收发、控制命令处理和接口初始化。USB CDC使设备在主机端呈现为虚拟串口。
 *
 * 主要功能模块：
 *   1. CDC_Init_FS()：CDC接口初始化，配置发送/接收缓冲区（各64字节）
 *   2. CDC_DeInit_FS()：CDC接口去初始化
 *   3. CDC_Control_FS()：处理CDC控制命令（线路编码、控制线状态等）
 *   4. CDC_Receive_FS()：USB数据接收回调，保存命令并唤醒解析线程
 *   5. CDC_Transmit_FS()：通过USB CDC发送数据到主机
 *
 * 缓冲区配置:
 *   - 接收缓冲区: 64字节
 *   - 发送缓冲区: 64字节
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
/* USER CODE BEGIN INCLUDE */
#include "utils.h"
#include "commands.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <freertos_vars.h>
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC 
  * @brief usbd核心模块
  * @{
  */ 

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Defines
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_DEFINES */
/* 自定义CDC缓冲区大小 */
/* 用户可根据需要重新定义或移除这些定义 */
#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  64
/* USER CODE END PRIVATE_DEFINES */
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_Variables
  * @{
  */
/* 创建接收和发送缓冲区 */
/* 通过USB接收的数据存储在此缓冲区 */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* 通过USB发送的数据存储在此缓冲区 */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */
static int8_t CDC_Init_FS     (void);
static int8_t CDC_DeInit_FS   (void);
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */ 
  
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = 
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,  
  CDC_Receive_FS
};

/**
 * @brief CDC接口初始化函数
 * 
 * 功能说明：
 * 初始化CDC通信接口的底层USB传输缓冲区。
 * 设置发送和接收缓冲区的地址和大小。
 * 
 * @return USBD_OK: 初始化成功
 */
static int8_t CDC_Init_FS(void)
{ 
  /* USER CODE BEGIN 3 */ 
  /* 设置应用程序缓冲区 */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */ 
}

/**
 * @brief CDC接口去初始化函数
 * 
 * 功能说明：
 * 释放CDC通信接口的底层USB传输资源。
 * 当前实现为空，因为USB设备库会自动管理资源。
 * 
 * @return USBD_OK: 去初始化成功
 */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */ 
  return (USBD_OK);
  /* USER CODE END 4 */ 
}

/**
 * @brief CDC控制命令处理函数
 * 
 * 功能说明：
 * 处理CDC类的各种控制请求。CDC类定义了一组标准命令，用于配置虚拟串口参数。
 * 大多数命令在此项目中不需要特殊处理，因为USB通信参数由固件固定。
 * 
 * 支持的命令：
 * - CDC_SEND_ENCAPSULATED_COMMAND: 发送封装命令（未使用）
 * - CDC_GET_ENCAPSULATED_RESPONSE: 获取封装响应（未使用）
 * - CDC_SET_COMM_FEATURE/CLEAR_COMM_FEATURE/GET_COMM_FEATURE: 通信特性管理（未使用）
 * - CDC_SET_LINE_CODING: 设置线路编码（波特率、停止位、校验位、数据位）
 * - CDC_GET_LINE_CODING: 获取线路编码
 * - CDC_SET_CONTROL_LINE_STATE: 设置控制线状态（DTR/RTS）
 * - CDC_SEND_BREAK: 发送中断信号
 * 
 * 注意：线路编码配置在此项目中被忽略，因为USB CDC是虚拟串口，
 * 实际通信速率由USB协议决定，与线路编码参数无关。
 * 
 * @param cmd: 命令代码
 * @param pbuf: 包含命令数据的缓冲区
 * @param length: 数据长度
 * @return USBD_OK: 处理成功
 */
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  /* USER CODE BEGIN 5 */
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
 
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
 
    break;

  case CDC_SET_COMM_FEATURE:
 
    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* 线路编码结构体结构                                                           */
  /*-----------------------------------------------------------------------------*/
  /* 偏移量 | 字段        | 大小 | 值类型 | 描述                                 */
  /* 0      | dwDTERate   |   4  | 数字   | 数据终端速率，比特/秒                 */
  /* 4      | bCharFormat |   1  | 数字   | 停止位                               */
  /*                                        0 - 1个停止位                        */
  /*                                        1 - 1.5个停止位                      */
  /*                                        2 - 2个停止位                        */
  /* 5      | bParityType |  1   | 数字   | 校验位                               */
  /*                                        0 - 无校验                           */
  /*                                        1 - 奇校验                           */ 
  /*                                        2 - 偶校验                           */
  /*                                        3 - 标记校验                         */
  /*                                        4 - 空格校验                         */
  /* 6      | bDataBits  |   1   | 数字   | 数据位 (5, 6, 7, 8 或 16位)           */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:   
	
    break;

  case CDC_GET_LINE_CODING:     

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:
 
    break;    
    
  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
 * @brief CDC数据接收函数
 * 
 * 功能说明：
 * 当USB OUT端点接收到数据时，USB设备库会调用此函数。
 * 接收到的数据通过此函数传递给上层应用处理。
 * 
 * 处理流程：
 * 1. 在数据末尾添加null终止符，确保数据可以作为字符串处理
 * 2. 调用set_cmd_buffer()将数据保存到命令缓冲区
 * 3. 释放sem_usb_rx信号量，唤醒命令解析线程处理接收到的数据
 * 
 * 注意：此函数在中断上下文中调用，应尽量简短快速。
 * 实际的数据处理在usb_update_thread线程中完成。
 * 
 * @param Buf: 接收到的数据缓冲区指针
 * @param Len: 接收到的数据长度（字节）
 * @return USBD_OK: 处理成功
 */
static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  // 在字符串末尾添加null终止符
  int modified_len = MACRO_MIN(*Len+1, APP_RX_DATA_SIZE);
  Buf[modified_len-1] = 0;

  // 将数据保存到命令缓冲区
  set_cmd_buffer(Buf, modified_len);
  // 释放接收信号量，唤醒命令解析线程
  xSemaphoreGiveFromISR(sem_usb_rx, NULL);

  return (USBD_OK);
  /* USER CODE END 6 */ 
}

/**
 * @brief USB数据发送函数
 * 
 * 功能说明：
 * 通过USB CDC接口发送数据到上位机。
 * 
 * 处理流程：
 * 1. 检查数据长度是否超过发送缓冲区大小(64字节)
 * 2. 检查是否有正在进行的传输（通过TxState标志）
 * 3. 将数据复制到发送缓冲区
 * 4. 更新发送缓冲区指针和长度
 * 5. 调用USBD_CDC_TransmitPacket()启动实际的数据传输
 * 
 * 并发控制：
 * 通过检查hcdc->TxState标志确保同一时间只有一个传输在进行。
 * 如果TxState != 0，表示上一次传输尚未完成，返回USBD_BUSY。
 * 
 * 调用方式：
 * 此函数由syscalls.c中的_write()函数调用，实现printf通过USB输出。
 * 调用前需等待sem_usb_tx信号量，确保USB接口可用。
 * 
 * @param Buf: 要发送的数据缓冲区指针
 * @param Len: 要发送的数据长度（字节）
 * @return USBD_OK: 发送成功启动
 *         USBD_FAIL: 数据长度超过缓冲区大小
 *         USBD_BUSY: 上一次传输尚未完成
 */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */ 
  
  // 检查数据长度是否超过发送缓冲区
  if (Len > APP_TX_DATA_SIZE)
    return USBD_FAIL;
  // 检查是否有正在进行的传输
  USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0)
    return USBD_BUSY;
  // 复制数据到发送缓冲区
  memcpy(UserTxBufferFS, Buf, Len);
  // 更新发送缓冲区长度
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */ 
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */ 



