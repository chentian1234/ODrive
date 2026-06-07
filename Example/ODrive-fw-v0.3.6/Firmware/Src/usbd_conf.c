/*
 * ============================================================================
 * 文件名: usbd_conf.c
 *
 * 文件用途:
 *   本文件实现USB设备库的底层驱动接口（BSP），负责USB OTG FS外设的硬件配置
 *   和与USB协议栈之间的数据传递。是USB设备库与STM32硬件之间的桥接层。
 *
 * 主要功能模块：
 *   1. HAL_PCD_MspInit/DeInit()：USB OTG FS MSP初始化/去初始化
 *   2. HAL PCD回调函数：SetupStage、DataIn/Out、SOF、Reset、Suspend/Resume等
 *   3. USBD_LL_*接口函数：Init/DeInit/Start/Stop/OpenEP/CloseEP等底层操作
 *   4. USBD_LL_Delay()：USB延时函数（基于HAL_Delay）
 *   5. HAL_PCDEx_LPM_Callback()：LPM电源管理回调
 *
 * USB OTG FS配置:
 *   - 端点数: 4
 *   - 速度: 全速(Full Speed, 12Mbps)
 *   - DMA: 禁用
 *   - 低功耗: 禁用
 *   - VBUS检测: 禁用
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"

PCD_HandleTypeDef hpcd_USB_OTG_FS;
void _Error_Handler(char * file, int line);

/* External functions --------------------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/
/* MSP Init */

void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(pcdHandle->Instance==USB_OTG_FS)
  {
  /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

  /* USER CODE END USB_OTG_FS_MspInit 0 */
  
    /**USB_OTG_FS GPIO配置    
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 使能USB外设时钟 */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    /* 外设中断初始化 */
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

  /* USER CODE END USB_OTG_FS_MspInit 1 */
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* pcdHandle)
{
  if(pcdHandle->Instance==USB_OTG_FS)
  {
  /* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

  /* USER CODE END USB_OTG_FS_MspDeInit 0 */
    /* 禁用USB外设时钟 */
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
  
    /**USB_OTG_FS GPIO配置    
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* 外设中断去初始化 */
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

  /* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

  /* USER CODE END USB_OTG_FS_MspDeInit 1 */
  }
}

/**
  * @brief  建立阶段回调函数
  * @param  hpcd: PCD句柄
  * @retval 无
  */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
}

/**
  * @brief  数据输出阶段回调函数
  * @param  hpcd: PCD句柄
  * @param  epnum: 端点号
  * @retval 无
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

/**
  * @brief  数据输入阶段回调函数
  * @param  hpcd: PCD句柄
  * @param  epnum: 端点号
  * @retval 无
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

/**
  * @brief  SOF(帧起始)回调函数
  * @param  hpcd: PCD句柄
  * @retval 无
  */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
}

/**
  * @brief  复位回调函数
  * @param  hpcd: PCD句柄
  * @retval 无
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{ 
  USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

  /* 设置USB当前速度 */
  switch (hpcd->Init.speed)
  {
  case PCD_SPEED_HIGH:
    speed = USBD_SPEED_HIGH;
    break;
  case PCD_SPEED_FULL:
    speed = USBD_SPEED_FULL;    
    break;
	
  default:
    speed = USBD_SPEED_FULL;    
    break;    
  }
  USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);  
  
  /* 复位设备 */
  USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}

/**
  * @brief  挂起回调函数
  * 启用低功耗模式时，调试功能不可用(IAR, Keil不支持)
  * @param  hpcd: PCD句柄
  * @retval 无
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{  
   /* 通知USB库核心进入挂起模式 */
  USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
  __HAL_PCD_GATE_PHYCLOCK(hpcd);
  /* 进入STOP模式 */
  /* USER CODE BEGIN 2 */
  if (hpcd->Init.low_power_enable)
  {
    /* 设置Cortex系统控制寄存器的SLEEPDEEP位和SleepOnExit位 */
    SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
  }
  /* USER CODE END 2 */
}

/**
  * @brief  恢复回调函数
  * 启用低功耗模式时，调试功能不可用(IAR, Keil不支持)
  * @param  hpcd: PCD句柄
  * @retval 无
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
  USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
  
}

/**
  * @brief  等时输出未完成回调函数
  * @param  hpcd: PCD句柄
  * @param  epnum: 端点号
  * @retval 无
  */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

/**
  * @brief  等时输入未完成回调函数
  * @param  hpcd: PCD句柄
  * @param  epnum: 端点号
  * @retval 无
  */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoINIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

/**
  * @brief  连接回调函数
  * @param  hpcd: PCD句柄
  * @retval 无
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected((USBD_HandleTypeDef*)hpcd->pData);
}

/**
  * @brief  断开连接回调函数
  * @param  hpcd: PCD句柄
  * @retval 无
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected((USBD_HandleTypeDef*)hpcd->pData);
}

/*******************************************************************************
                       底层驱动接口 (USB设备库 --> PCD)
*******************************************************************************/
/**
  * @brief  初始化底层驱动
  * @param  pdev: 设备句柄
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_Init (USBD_HandleTypeDef *pdev)
{ 
  /* 初始化USB_IP */
  if (pdev->id == DEVICE_FS) {
  /* 将驱动链接到协议栈 */	
  hpcd_USB_OTG_FS.pData = pdev;
  pdev->pData = &hpcd_USB_OTG_FS; 
  
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80);
  }
  return USBD_OK;
}

/**
  * @brief  反初始化底层驱动
  * @param  pdev: 设备句柄
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_DeInit (USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
 
  hal_status = HAL_PCD_DeInit(pdev->pData);
     
  switch (hal_status) {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status; 
}

/**
  * @brief  启动底层驱动
  * @param  pdev: 设备句柄
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
 
  hal_status = HAL_PCD_Start(pdev->pData);
     
  switch (hal_status) {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status;  
}

/**
  * @brief  停止底层驱动
  * @param  pdev: 设备句柄
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_Stop (USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
 
  hal_status = HAL_PCD_Stop(pdev->pData);
     
  switch (hal_status) {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status; 
}

/**
  * @brief  打开底层驱动端点
  * @param  pdev: 设备句柄
  * @param  ep_addr: 端点地址
  * @param  ep_type: 端点类型
  * @param  ep_mps: 端点最大包大小                 
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_OpenEP  (USBD_HandleTypeDef *pdev, 
                                      uint8_t  ep_addr,                                      
                                      uint8_t  ep_type,
                                      uint16_t ep_mps)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Open(pdev->pData, 
                               ep_addr, 
                               ep_mps, 
                               ep_type);
  
     
  switch (hal_status) {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status; 
}

/**
  * @brief  关闭底层驱动端点
  * @param  pdev: 设备句柄
  * @param  ep_addr: 端点地址
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_CloseEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  
  hal_status = HAL_PCD_EP_Close(pdev->pData, ep_addr);
      
  switch (hal_status) {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status;  
}

/**
  * @brief  刷新(清除)底层驱动端点
  * @param  pdev: 设备句柄
  * @param  ep_addr: 端点地址
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_FlushEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  
  hal_status = HAL_PCD_EP_Flush(pdev->pData, ep_addr);
      
  switch (hal_status) {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status;  
}

/**
  * @brief  设置端点STALL状态
  * @param  pdev: 设备句柄
  * @param  ep_addr: 端点地址
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_StallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  
  hal_status = HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
      
  switch (hal_status) {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status;  
}

/**
  * @brief  清除端点STALL状态
  * @param  pdev: 设备句柄
  * @param  ep_addr: 端点地址
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_ClearStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  
  hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);  
     
  switch (hal_status) {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status; 
}

/**
  * @brief  返回端点STALL状态
  * @param  pdev: 设备句柄
  * @param  ep_addr: 端点地址
  * @retval STALL状态 (1: 是, 0: 否)
  */
uint8_t USBD_LL_IsStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;
  
  if((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall; 
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall; 
  }
}
/**
  * @brief  为设备分配USB地址
  * @param  pdev: 设备句柄
  * @param  dev_addr: 设备地址
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_SetUSBAddress (USBD_HandleTypeDef *pdev, uint8_t dev_addr)   
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  
  hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);
     
  switch (hal_status) {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status;  
}

/**
  * @brief  通过端点传输数据
  * @param  pdev: 设备句柄
  * @param  ep_addr: 端点地址
  * @param  pbuf: 要发送的数据指针
  * @param  size: 数据大小    
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_Transmit (USBD_HandleTypeDef *pdev, 
                                      uint8_t  ep_addr,                                      
                                      uint8_t  *pbuf,
                                      uint16_t  size)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
     
  switch (hal_status) {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status;    
}

/**
  * @brief  准备端点接收数据
  * @param  pdev: 设备句柄
  * @param  ep_addr: 端点地址
  * @param  pbuf: 要接收的数据指针
  * @param  size: 数据大小
  * @retval USBD状态
  */
USBD_StatusTypeDef  USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, 
                                           uint8_t  ep_addr,                                      
                                           uint8_t  *pbuf,
                                           uint16_t  size)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
     
  switch (hal_status) {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status; 
}

/**
  * @brief  返回最后一次传输的数据包大小
  * @param  pdev: 设备句柄
  * @param  ep_addr: 端点地址
  * @retval 接收到的数据大小
  */
uint32_t USBD_LL_GetRxDataSize  (USBD_HandleTypeDef *pdev, uint8_t  ep_addr)  
{
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) pdev->pData, ep_addr);
}

#if (USBD_LPM_ENABLED == 1)
/**
  * @brief  HAL_PCDEx_LPM回调函数: 发送LPM消息到用户层
  * @param  hpcd: PCD句柄
  * @param  msg: LPM消息
  * @retval HAL状态
  */
void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg)
{
  switch ( msg)
  {
  case PCD_LPM_L0_ACTIVE:
    if (hpcd->Init.low_power_enable)
    {
      SystemClock_Config();
      
      /* 重置Cortex系统控制寄存器的SLEEPDEEP位 */
      SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }
    __HAL_PCD_UNGATE_PHYCLOCK(hpcd);
    USBD_LL_Resume(hpcd->pData);    
    break;
    
  case PCD_LPM_L1_ACTIVE:
    __HAL_PCD_GATE_PHYCLOCK(hpcd);
    USBD_LL_Suspend(hpcd->pData);
    
    /* 进入STOP模式 */
    if (hpcd->Init.low_power_enable)
    {   
      /* 设置Cortex系统控制寄存器的SLEEPDEEP位和SleepOnExit位 */
      SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }     
    break;   
  }
}
#endif
/**
  * @brief  USB设备库延时函数
  * @param  Delay: 延时时间(毫秒)
  * @retval 无
  */
void  USBD_LL_Delay (uint32_t Delay)
{
  HAL_Delay(Delay);  
}

