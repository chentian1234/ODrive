/*
 * ============================================================================
 * 文件名: usbd_desc.c
 *
 * 文件用途:
 *   本文件定义USB设备的所有描述符，包括设备描述符、配置描述符、字符串描述符等。
 *   这些描述符在USB枚举过程中被主机读取，用于识别设备的类型、厂商和功能。
 *
 * 主要功能模块：
 *   1. 设备描述符：VID/PID、USB版本、设备类别（CDC类）
 *   2. 字符串描述符：制造商(ODrive)、产品名称(ODrive v3.x)、序列号
 *   3. BOS描述符：LPM电源管理支持
 *   4. 描述符获取函数：按类型返回对应的描述符缓冲区
 *
 * USB描述符参数:
 *   - VID: 0x1209
 *   - PID: 0x0D31
 *   - 制造商: "ODrive"
 *   - 产品名: 根据硬件版本动态显示 (ODrive v3.1~v3.4)
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_DESC 
  * @brief USBD descriptors module
  * @{
  */ 

/** @defgroup USBD_DESC_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Private_Defines
  * @{
  */ 
#define USBD_VID     0x1209
#define USBD_LANGID_STRING     1033
#define USBD_MANUFACTURER_STRING     "ODrive"
#define USBD_PID_FS     0x0D31

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 1
#define USBD_PRODUCT_STRING_FS     "ODrive v3.1"
#elif HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 2
#define USBD_PRODUCT_STRING_FS     "ODrive v3.2"
#elif HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 3
#define USBD_PRODUCT_STRING_FS     "ODrive v3.3"
#else
#define USBD_PRODUCT_STRING_FS     "ODrive v3.4"
#endif

#define USBD_SERIALNUMBER_STRING_FS     "000000000001"
#define USBD_CONFIGURATION_STRING_FS     "CDC Config"
#define USBD_INTERFACE_STRING_FS     "CDC Interface"

#define USB_SIZ_BOS_DESC            0x0C

/* USER CODE BEGIN 0 */

/* USER CODE END 0*/
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Private_Variables
  * @{
  */ 
uint8_t *     USBD_FS_DeviceDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_LangIDStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_ManufacturerStrDescriptor ( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_ProductStrDescriptor ( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_SerialStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_ConfigStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_FS_InterfaceStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);

#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t *     USBD_FS_USRStringDesc (USBD_SpeedTypeDef speed, uint8_t idx , uint16_t *length);  
#endif /* USB_SUPPORT_USER_STRING_DESC */  

#if (USBD_LPM_ENABLED == 1)
uint8_t *USBD_FS_USR_BOSDescriptor(USBD_SpeedTypeDef speed , uint16_t *length);
#endif

USBD_DescriptorsTypeDef FS_Desc =
{
  USBD_FS_DeviceDescriptor,
  USBD_FS_LangIDStrDescriptor, 
  USBD_FS_ManufacturerStrDescriptor,
  USBD_FS_ProductStrDescriptor,
  USBD_FS_SerialStrDescriptor,
  USBD_FS_ConfigStrDescriptor,
  USBD_FS_InterfaceStrDescriptor,
#if (USBD_LPM_ENABLED == 1)  
  USBD_FS_USR_BOSDescriptor,
#endif  
};

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
  {
    0x12,                       /*bLength */
    USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
#if (USBD_LPM_ENABLED == 1)
    0x01,                       /*bcdUSB */ /* changed to USB version 2.01 
                                               in order to support LPM L1 suspend
                                               resume test of USBCV3.0*/
#else  
    0x00,                       /* bcdUSB */
#endif
    0x02,
    0x02,                        /*bDeviceClass*/
    0x02,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    USB_MAX_EP0_SIZE,          /*bMaxPacketSize*/
    LOBYTE(USBD_VID),           /*idVendor*/
    HIBYTE(USBD_VID),           /*idVendor*/
    LOBYTE(USBD_PID_FS),           /*idVendor*/
    HIBYTE(USBD_PID_FS),           /*idVendor*/
    0x00,                       /*bcdDevice rel. 2.00*/
    0x02,
    USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
    USBD_IDX_PRODUCT_STR,       /*Index of product string*/
    USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
    USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
  } ; 
/* USB_DeviceDescriptor */
/* BOS descriptor */
#if (USBD_LPM_ENABLED == 1)
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif
__ALIGN_BEGIN  uint8_t USBD_FS_BOSDesc[USB_SIZ_BOS_DESC] __ALIGN_END =
{
  0x5,
  USB_DESC_TYPE_BOS,
  0xC,
  0x0,
  0x1,  /* 1 device capability */
        /* device capability*/
  0x7,
  USB_DEVICE_CAPABITY_TYPE,
  0x2,
  0x2, /*LPM capability bit set */
  0x0,
  0x0,
  0x0
};
#endif

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif

/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
     USB_LEN_LANGID_STR_DESC,         
     USB_DESC_TYPE_STRING,       
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING), 
};

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4   
#endif
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Private_FunctionPrototypes
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBD_DESC_Private_Functions
  * @{
  */ 

/**
* @brief  返回设备描述符
* @param  speed: 当前设备速度
* @param  length: 指向数据长度变量的指针
* @retval 指向描述符缓冲区的指针
*/
uint8_t *  USBD_FS_DeviceDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

/**
* @brief  返回语言ID字符串描述符
* @param  speed: 当前设备速度
* @param  length: 指向数据长度变量的指针
* @retval 指向描述符缓冲区的指针
*/
uint8_t *  USBD_FS_LangIDStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  *length =  sizeof(USBD_LangIDDesc);  
  return USBD_LangIDDesc;
}

/**
* @brief  返回产品字符串描述符
* @param  speed: 当前设备速度
* @param  length: 指向数据长度变量的指针
* @retval 指向描述符缓冲区的指针
*/
uint8_t *  USBD_FS_ProductStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString ((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString ((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
* @brief  返回制造商字符串描述符
* @param  speed: 当前设备速度
* @param  length: 指向数据长度变量的指针
* @retval 指向描述符缓冲区的指针
*/
uint8_t *  USBD_FS_ManufacturerStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  USBD_GetString ((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
* @brief  返回序列号字符串描述符
* @param  speed: 当前设备速度
* @param  length: 指向数据长度变量的指针
* @retval 指向描述符缓冲区的指针
*/
uint8_t *  USBD_FS_SerialStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  if(speed  == USBD_SPEED_HIGH)
  {
    USBD_GetString ((uint8_t *)USBD_SERIALNUMBER_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString ((uint8_t *)USBD_SERIALNUMBER_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
* @brief  返回配置字符串描述符
* @param  speed: 当前设备速度
* @param  length: 指向数据长度变量的指针
* @retval 指向描述符缓冲区的指针
*/
uint8_t *  USBD_FS_ConfigStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  if(speed  == USBD_SPEED_HIGH)
  {
    USBD_GetString ((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString ((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;  
}

/**
* @brief  返回接口字符串描述符
* @param  speed: 当前设备速度
* @param  length: 指向数据长度变量的指针
* @retval 指向描述符缓冲区的指针
*/
uint8_t *  USBD_FS_InterfaceStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString ((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString ((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;  
}
#if (USBD_LPM_ENABLED == 1)
/**
  * @brief  返回BOS描述符
  * @param  speed: 当前设备速度
  * @param  length: 指向数据长度变量的指针
  * @retval 指向描述符缓冲区的指针
  */
uint8_t *USBD_FS_USR_BOSDescriptor(USBD_SpeedTypeDef speed , uint16_t *length)
{
  *length = sizeof(USBD_FS_BOSDesc);
  return (uint8_t*)USBD_FS_BOSDesc;
}
#endif
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

