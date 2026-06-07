/*
 * ============================================================================
 * 文件名: usb_device.c
 *
 * 文件用途:
 *   本文件实现USB Device设备的初始化配置，将STM32F4配置为CDC类虚拟串口设备，
 *   使电机控制器可以通过USB与上位机通信。
 *
 * 主要功能模块：
 *   1. MX_USB_DEVICE_Init()：USB设备初始化
 *      - USBD_Init(): 初始化USB设备库，加载设备描述符
 *      - USBD_RegisterClass(): 注册CDC类
 *      - USBD_CDC_RegisterInterface(): 注册CDC接口回调函数
 *      - USBD_Start(): 启动USB设备，开始监听主机连接
 *
 * 注意：此函数必须在FreeRTOS初始化之前调用，确保USB枚举过程尽早开始
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USB Device Core handle - USB设备核心句柄 */
USBD_HandleTypeDef hUsbDeviceFS;

/**
 * @brief USB设备初始化函数
 * 
 * 功能说明：
 * 初始化USB OTG FS设备，配置为CDC(Communication Device Class)类，
 * 实现虚拟串口功能，使电机控制器可以通过USB与上位机通信。
 * 
 * 初始化流程：
 * 1. USBD_Init(): 初始化USB设备库，加载设备描述符(FS_Desc)
 * 2. USBD_RegisterClass(): 注册CDC类，使设备支持USB CDC协议
 * 3. USBD_CDC_RegisterInterface(): 注册CDC接口回调函数
 *    (USBD_Interface_fops_FS)，用于处理CDC类的具体操作
 * 4. USBD_Start(): 启动USB设备，开始监听USB主机连接
 * 
 * 注意：此函数必须在FreeRTOS初始化之前调用，确保USB枚举过程尽早开始
 */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */
  
  /* USER CODE END USB_DEVICE_Init_PreTreatment */
  
  /* 初始化设备库，注册支持的类并启动库 */
  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);

  USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);

  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);

  USBD_Start(&hUsbDeviceFS);

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  
  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}
/**
  * @}
  */

/**
  * @}
  */

