/*
 * ============================================================================
 * 文件名: usb_device.h
 *
 * 文件用途:
 *   本文件定义USB Device设备初始化的函数接口，将STM32F4配置为CDC类虚拟串口设备。
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usb_device_H
#define __usb_device_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USB_Device init function */	
void MX_USB_DEVICE_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__usb_device_H */

/**
  * @}
  */

/**
  * @}
  */


