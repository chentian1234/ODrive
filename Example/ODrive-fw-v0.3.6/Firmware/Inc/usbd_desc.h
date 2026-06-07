/*
 * ============================================================================
 * 文件名: usbd_desc.h
 *
 * 文件用途:
 *   本文件定义USB设备的所有描述符接口，包括设备描述符、配置描述符、字符串描述符等。
 *   这些描述符在USB枚举过程中被主机读取，用于识别设备的类型、厂商和功能。
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_DESC__H__
#define __USBD_DESC__H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USB_DESC
  * @brief general defines for the usb device library file
  * @{
  */ 

/** @defgroup USB_DESC_Exported_Defines
  * @{
  */

/**
  * @}
  */ 

/** @defgroup USBD_DESC_Exported_TypesDefinitions
  * @{
  */
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Exported_Variables
  * @{
  */ 
extern USBD_DescriptorsTypeDef FS_Desc;
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Exported_FunctionsPrototype
  * @{
  */ 
  
/**
  * @}
  */ 
#ifdef __cplusplus
}
#endif

#endif /* __USBD_DESC_H */

/**
  * @}
  */ 

/**
* @}
*/ 

