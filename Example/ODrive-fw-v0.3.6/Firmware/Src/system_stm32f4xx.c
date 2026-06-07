/*
 * ============================================================================
 * 文件名: system_stm32f4xx.c
 *
 * 文件用途:
 *   本文件是CMSIS Cortex-M4系统级源文件，提供SystemInit()和SystemCoreClockUpdate()
 *   两个核心函数。SystemInit()在启动文件（startup_stm32f4xx.s）中调用，
 *   用于初始化FPU设置、重置时钟配置、配置中断向量表位置。
 *
 * 主要功能模块：
 *   1. SystemInit()：系统启动初始化
 *      - FPU使能（CP10/CP11完全访问）
 *      - 重置RCC时钟配置为默认复位状态
 *      - 配置中断向量表位置（Flash或SRAM）
 *      - 可选配置外部存储器控制器（SRAM/SDRAM）
 *   2. SystemCoreClockUpdate()：更新系统核心时钟变量
 *   3. SystemInit_ExtMemCtl()：外部存储器配置（条件编译）
 *
 * 关键变量:
 *   - SystemCoreClock: 当前系统核心时钟频率（HCLK），默认16MHz
 *   - HSE_VALUE: 外部高速晶振频率，默认25MHz
 *   - HSI_VALUE: 内部高速振荡器频率，默认16MHz
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/** @addtogroup CMSIS
  * @{
 */

/** @addtogroup stm32f4xx_system
  * @{
  */  
  
/** @addtogroup STM32F4xx_System_Private_Includes
  * @{
  */


#include "stm32f4xx.h"

#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)25000000) /*!< 外部振荡器默认值，单位Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000) /*!< 内部振荡器值，单位Hz */
#endif /* HSI_VALUE */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to use external SRAM or SDRAM as data memory  */
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)\
 || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx)
/* #define DATA_IN_ExtSRAM */
#endif /* STM32F40xxx || STM32F41xxx || STM32F42xxx || STM32F43xxx || STM32F469xx || STM32F479xx ||\
          STM32F412Zx || STM32F412Vx */
 
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
/* #define DATA_IN_ExtSDRAM */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx ||\
          STM32F479xx */

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /*!< 中断向量表基地址偏移字段。
                                   此值必须是0x200的整数倍。 */
/******************************************************************************/

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */
  /* 此变量通过以下三种方式更新：
      1) 调用CMSIS函数SystemCoreClockUpdate()
      2) 调用HAL API函数HAL_RCC_GetHCLKFreq()
      3) 每次调用HAL_RCC_ClockConfig()配置系统时钟频率时
         注意：如果使用此函数配置系统时钟，则无需调用上述前两个函数，
         因为SystemCoreClock变量会自动更新。
  */
uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_FunctionPrototypes
  * @{
  */

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
  static void SystemInit_ExtMemCtl(void); 
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Functions
  * @{
  */

/**
  * @brief 微控制器系统初始化函数
  *         初始化FPU设置、中断向量表位置和外部存储器配置
  * @param  无
  * @retval 无
  */
void SystemInit(void)
{
  /* FPU设置 ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* 设置CP10和CP11完全访问 */
  #endif
  /* 重置RCC时钟配置为默认复位状态 ------------*/
  /* 设置HSION位 */
  RCC->CR |= (uint32_t)0x00000001;

  /* 重置CFGR寄存器 */
  RCC->CFGR = 0x00000000;

  /* 重置HSEON、CSSON和PLLON位 */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* 重置PLLCFGR寄存器 */
  RCC->PLLCFGR = 0x24003010;

  /* 重置HSEBYP位 */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* 禁用所有中断 */
  RCC->CIR = 0x00000000;

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
  SystemInit_ExtMemCtl(); 
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

  /* 配置中断向量表位置及偏移地址 ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* 中断向量表重定位到内部SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* 中断向量表重定位到内部FLASH */
#endif
}

/**
   * @brief  根据时钟寄存器值更新SystemCoreClock变量
  *         SystemCoreClock变量包含核心时钟(HCLK)，可用于用户应用程序
  *         配置SysTick定时器或其他参数。
  *           
  * @note   每次核心时钟(HCLK)改变时，必须调用此函数更新SystemCoreClock值。
  *         否则，任何基于此变量的配置将不正确。
  *     
  * @note   - 此函数计算的系统频率不是芯片的实际频率，而是基于预定义
  *           常量和所选时钟源计算得出：
  *             
  *           - 如果SYSCLK源是HSI，SystemCoreClock将包含HSI_VALUE(*)
  *                                              
  *           - 如果SYSCLK源是HSE，SystemCoreClock将包含HSE_VALUE(**)
  *                          
  *           - 如果SYSCLK源是PLL，SystemCoreClock将包含HSE_VALUE(**) 
  *             或HSI_VALUE(*)乘以/除以PLL因子
  *         
  *         (*) HSI_VALUE是stm32f4xx_hal_conf.h文件中定义的常量(默认值16 MHz)，
  *             但实际值可能随电压和温度变化而变化
  *    
  *         (**) HSE_VALUE是stm32f4xx_hal_conf.h文件中定义的常量，
  *              用户必须确保HSE_VALUE与实际晶振频率一致，否则此函数可能出错
  *                
  *         - 使用HSE晶体的分数值时，此函数结果可能不正确
  *     
  * @param  无
  * @retval 无
  */
void SystemCoreClockUpdate(void)
{
  uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
  
  /* 获取SYSCLK时钟源 -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case 0x00:  /* HSI用作系统时钟源 */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE用作系统时钟源 */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:  /* PLL用作系统时钟源 */

      /* PLL_VCO = (HSE_VALUE或HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */    
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      
      if (pllsource != 0)
      {
        /* HSE用作PLL时钟源 */
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      else
      {
        /* HSI用作PLL时钟源 */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      SystemCoreClock = pllvco/pllp;
      break;
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* 计算HCLK频率 --------------------------------------------------*/
  /* 获取HCLK预分频器 */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK频率 */
  SystemCoreClock >>= tmp;
}

#if defined (DATA_IN_ExtSRAM) && defined (DATA_IN_ExtSDRAM)
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx)
/**
  * @brief  配置外部存储器控制器。
  *         在startup_stm32f4xx.s中跳转至main()之前调用。
  *         此函数配置外部存储器（SRAM/SDRAM）
  *         该SRAM/SDRAM将用作程序数据存储器（包括堆和栈）。
  * @param  无
  * @retval 无
  */
void SystemInit_ExtMemCtl(void)
{
  __IO uint32_t tmp = 0x00;

  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  register __IO uint32_t index;

  /* 使能GPIOC、GPIOD、GPIOE、GPIOF、GPIOG、GPIOH和GPIOI接口时钟 */
  RCC->AHB1ENR |= 0x000001F8;

  /* RCC外设时钟使能后的延迟 */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
  
  /* 将PDx引脚连接到FMC复用功能 */
  GPIOD->AFR[0]  = 0x00CCC0CC;
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* 配置PDx引脚为复用功能模式 */  
  GPIOD->MODER   = 0xAAAA0A8A;
  /* 配置PDx引脚速度为100 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0FCF;
  /* 配置PDx引脚输出类型为推挽 */  
  GPIOD->OTYPER  = 0x00000000;
  /* PDx引脚无上下拉 */ 
  GPIOD->PUPDR   = 0x00000000;

  /* 将PEx引脚连接到FMC复用功能 */
  GPIOE->AFR[0]  = 0xC00CC0CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* 配置PEx引脚为复用功能模式 */ 
  GPIOE->MODER   = 0xAAAA828A;
  /* 配置PEx引脚速度为100 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* 配置PEx引脚输出类型为推挽 */  
  GPIOE->OTYPER  = 0x00000000;
  /* PEx引脚无上下拉 */ 
  GPIOE->PUPDR   = 0x00000000;
  
  /* 将PFx引脚连接到FMC复用功能 */
  GPIOF->AFR[0]  = 0xCCCCCCCC;
  GPIOF->AFR[1]  = 0xCCCCCCCC;
  /* 配置PFx引脚为复用功能模式 */   
  GPIOF->MODER   = 0xAA800AAA;
  /* 配置PFx引脚速度为50 MHz */ 
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* 配置PFx引脚输出类型为推挽 */  
  GPIOF->OTYPER  = 0x00000000;
  /* PFx引脚无上下拉 */ 
  GPIOF->PUPDR   = 0x00000000;

  /* 将PGx引脚连接到FMC复用功能 */
  GPIOG->AFR[0]  = 0xCCCCCCCC;
  GPIOG->AFR[1]  = 0xCCCCCCCC;
  /* 配置PGx引脚为复用功能模式 */ 
  GPIOG->MODER   = 0xAAAAAAAA;
  /* 配置PGx引脚速度为50 MHz */ 
  GPIOG->OSPEEDR = 0xAAAAAAAA;
  /* 配置PGx引脚输出类型为推挽 */  
  GPIOG->OTYPER  = 0x00000000;
  /* PGx引脚无上下拉 */ 
  GPIOG->PUPDR   = 0x00000000;
  
  /* 将PHx引脚连接到FMC复用功能 */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* 配置PHx引脚为复用功能模式 */ 
  GPIOH->MODER   = 0xAAAA08A0;
  /* 配置PHx引脚速度为50 MHz */ 
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* 配置PHx引脚输出类型为推挽 */  
  GPIOH->OTYPER  = 0x00000000;
  /* PHx引脚无上下拉 */ 
  GPIOH->PUPDR   = 0x00000000;
  
  /* 将PIx引脚连接到FMC复用功能 */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* 配置PIx引脚为复用功能模式 */ 
  GPIOI->MODER   = 0x0028AAAA;
  /* 配置PIx引脚速度为50 MHz */ 
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* 配置PIx引脚输出类型为推挽 */  
  GPIOI->OTYPER  = 0x00000000;
  /* PIx引脚无上下拉 */ 
  GPIOI->PUPDR   = 0x00000000;
  
/*-- FMC配置 -------------------------------------------------------*/
  /* 使能FMC接口时钟 */
  RCC->AHB3ENR |= 0x00000001;
  /* RCC外设时钟使能后的延迟 */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);

  FMC_Bank5_6->SDCR[0] = 0x000019E4;
  FMC_Bank5_6->SDTR[0] = 0x01115351;      
  
  /* SDRAM初始化序列 */
  /* 时钟使能命令 */
  FMC_Bank5_6->SDCMR = 0x00000011; 
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }

  /* 延迟 */
  for (index = 0; index<1000; index++);
  
  /* PALL命令（预充电所有bank） */
  FMC_Bank5_6->SDCMR = 0x00000012;           
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
  
  /* 自动刷新命令 */
  FMC_Bank5_6->SDCMR = 0x00000073;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
 
  /* MRD寄存器编程 */
  FMC_Bank5_6->SDCMR = 0x00046014;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  } 
  
  /* 设置刷新计数 */
  tmpreg = FMC_Bank5_6->SDRTR;
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));
  
  /* 禁用写保护 */
  tmpreg = FMC_Bank5_6->SDCR[0]; 
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  /* 配置并使能Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001011;
  FMC_Bank1->BTCR[3]  = 0x00000201;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */ 
#if defined(STM32F469xx) || defined(STM32F479xx)
  /* 配置并使能Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001091;
  FMC_Bank1->BTCR[3]  = 0x00110212;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F469xx || STM32F479xx */ 

  (void)(tmp); 
}
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
#elif defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
/**
  * @brief  配置外部存储器控制器。
  *         在startup_stm32f4xx.s中跳转至main()之前调用。
  *         此函数配置外部存储器（SRAM/SDRAM）
  *         该SRAM/SDRAM将用作程序数据存储器（包括堆和栈）。
  * @param  无
  * @retval 无
  */
void SystemInit_ExtMemCtl(void)
{
  __IO uint32_t tmp = 0x00;
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
#if defined (DATA_IN_ExtSDRAM)
  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  register __IO uint32_t index;

#if defined(STM32F446xx)
  /* 使能GPIOA、GPIOC、GPIOD、GPIOE、GPIOF、GPIOG接口时钟 */
  RCC->AHB1ENR |= 0x0000007D;
#else
  /* 使能GPIOC、GPIOD、GPIOE、GPIOF、GPIOG、GPIOH和GPIOI接口时钟 */
  RCC->AHB1ENR |= 0x000001F8;
#endif /* STM32F446xx */  
  /* RCC外设时钟使能后的延迟 */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
  
#if defined(STM32F446xx)
  /* 将PAx引脚连接到FMC复用功能 */
  GPIOA->AFR[0]  |= 0xC0000000;
  GPIOA->AFR[1]  |= 0x00000000;
  /* 配置PAx引脚为复用功能模式 */
  GPIOA->MODER   |= 0x00008000;
  /* 配置PAx引脚速度为50 MHz */
  GPIOA->OSPEEDR |= 0x00008000;
  /* 配置PAx引脚输出类型为推挽 */
  GPIOA->OTYPER  |= 0x00000000;
  /* PAx引脚无上下拉 */
  GPIOA->PUPDR   |= 0x00000000;

  /* 将PCx引脚连接到FMC复用功能 */
  GPIOC->AFR[0]  |= 0x00CC0000;
  GPIOC->AFR[1]  |= 0x00000000;
  /* 配置PCx引脚为复用功能模式 */
  GPIOC->MODER   |= 0x00000A00;
  /* 配置PCx引脚速度为50 MHz */
  GPIOC->OSPEEDR |= 0x00000A00;
  /* 配置PCx引脚输出类型为推挽 */
  GPIOC->OTYPER  |= 0x00000000;
  /* PCx引脚无上下拉 */
  GPIOC->PUPDR   |= 0x00000000;
#endif /* STM32F446xx */

  /* 将PDx引脚连接到FMC复用功能 */
  GPIOD->AFR[0]  = 0x000000CC;
  GPIOD->AFR[1]  = 0xCC000CCC;
  /* 配置PDx引脚为复用功能模式 */  
  GPIOD->MODER   = 0xA02A000A;
  /* 配置PDx引脚速度为50 MHz */  
  GPIOD->OSPEEDR = 0xA02A000A;
  /* 配置PDx引脚输出类型为推挽 */  
  GPIOD->OTYPER  = 0x00000000;
  /* PDx引脚无上下拉 */ 
  GPIOD->PUPDR   = 0x00000000;

  /* 将PEx引脚连接到FMC复用功能 */
  GPIOE->AFR[0]  = 0xC00000CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* 配置PEx引脚为复用功能模式 */ 
  GPIOE->MODER   = 0xAAAA800A;
  /* 配置PEx引脚速度为50 MHz */ 
  GPIOE->OSPEEDR = 0xAAAA800A;
  /* 配置PEx引脚输出类型为推挽 */  
  GPIOE->OTYPER  = 0x00000000;
  /* PEx引脚无上下拉 */ 
  GPIOE->PUPDR   = 0x00000000;

  /* 将PFx引脚连接到FMC复用功能 */
  GPIOF->AFR[0]  = 0xCCCCCCCC;
  GPIOF->AFR[1]  = 0xCCCCCCCC;
  /* 配置PFx引脚为复用功能模式 */   
  GPIOF->MODER   = 0xAA800AAA;
  /* 配置PFx引脚速度为50 MHz */ 
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* 配置PFx引脚输出类型为推挽 */  
  GPIOF->OTYPER  = 0x00000000;
  /* PFx引脚无上下拉 */ 
  GPIOF->PUPDR   = 0x00000000;

  /* 将PGx引脚连接到FMC复用功能 */
  GPIOG->AFR[0]  = 0xCCCCCCCC;
  GPIOG->AFR[1]  = 0xCCCCCCCC;
  /* 配置PGx引脚为复用功能模式 */ 
  GPIOG->MODER   = 0xAAAAAAAA;
  /* 配置PGx引脚速度为50 MHz */ 
  GPIOG->OSPEEDR = 0xAAAAAAAA;
  /* 配置PGx引脚输出类型为推挽 */  
  GPIOG->OTYPER  = 0x00000000;
  /* PGx引脚无上下拉 */ 
  GPIOG->PUPDR   = 0x00000000;

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx)  
  /* 将PHx引脚连接到FMC复用功能 */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* 配置PHx引脚为复用功能模式 */ 
  GPIOH->MODER   = 0xAAAA08A0;
  /* 配置PHx引脚速度为50 MHz */ 
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* 配置PHx引脚输出类型为推挽 */  
  GPIOH->OTYPER  = 0x00000000;
  /* PHx引脚无上下拉 */ 
  GPIOH->PUPDR   = 0x00000000;
  
  /* 将PIx引脚连接到FMC复用功能 */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* 配置PIx引脚为复用功能模式 */ 
  GPIOI->MODER   = 0x0028AAAA;
  /* 配置PIx引脚速度为50 MHz */ 
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* 配置PIx引脚输出类型为推挽 */  
  GPIOI->OTYPER  = 0x00000000;
  /* PIx引脚无上下拉 */ 
  GPIOI->PUPDR   = 0x00000000;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
  
/*-- FMC配置 -------------------------------------------------------*/
  /* 使能FMC接口时钟 */
  RCC->AHB3ENR |= 0x00000001;
  /* RCC外设时钟使能后的延迟 */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);

  /* 配置并使能SDRAM bank1 */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCR[0] = 0x00001954;
#else  
  FMC_Bank5_6->SDCR[0] = 0x000019E4;
#endif /* STM32F446xx */
  FMC_Bank5_6->SDTR[0] = 0x01115351;      
  
  /* SDRAM初始化序列 */
  /* 时钟使能命令 */
  FMC_Bank5_6->SDCMR = 0x00000011; 
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }

  /* 延迟 */
  for (index = 0; index<1000; index++);
  
  /* PALL命令（预充电所有bank） */
  FMC_Bank5_6->SDCMR = 0x00000012;           
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
  
  /* 自动刷新命令 */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCMR = 0x000000F3;
#else  
  FMC_Bank5_6->SDCMR = 0x00000073;
#endif /* STM32F446xx */
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
 
  /* MRD寄存器编程 */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCMR = 0x00044014;
#else  
  FMC_Bank5_6->SDCMR = 0x00046014;
#endif /* STM32F446xx */
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  } 
  
  /* 设置刷新计数 */
  tmpreg = FMC_Bank5_6->SDRTR;
#if defined(STM32F446xx)
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000050C<<1));
#else    
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));
#endif /* STM32F446xx */
  
  /* Disable write protection */
  tmpreg = FMC_Bank5_6->SDCR[0]; 
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);
#endif /* DATA_IN_ExtSDRAM */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx || STM32F479xx */

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)\
 || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx)

#if defined(DATA_IN_ExtSRAM)
/*-- GPIOs Configuration -----------------------------------------------------*/
   /* Enable GPIOD, GPIOE, GPIOF and GPIOG interface clock */
  RCC->AHB1ENR   |= 0x00000078;
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
  
  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x00CCC0CC;
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xAAAA0A8A;
  /* Configure PDx pins speed to 100 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0FCF;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00CC0CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA828A;
  /* Configure PEx pins speed to 100 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0x00CCCCCC;
  GPIOF->AFR[1]  = 0xCCCC0000;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA000AAA;
  /* Configure PFx pins speed to 100 MHz */ 
  GPIOF->OSPEEDR = 0xFF000FFF;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0x00CCCCCC;
  GPIOG->AFR[1]  = 0x000000C0;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0x00085AAA;
  /* Configure PGx pins speed to 100 MHz */ 
  GPIOG->OSPEEDR = 0x000CAFFF;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;
  
/*-- FMC/FSMC Configuration --------------------------------------------------*/
  /* Enable the FMC/FSMC interface clock */
  RCC->AHB3ENR         |= 0x00000001;

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001011;
  FMC_Bank1->BTCR[3]  = 0x00000201;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */ 
#if defined(STM32F469xx) || defined(STM32F479xx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001091;
  FMC_Bank1->BTCR[3]  = 0x00110212;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F469xx || STM32F479xx */
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx)|| defined(STM32F417xx)\
   || defined(STM32F412Zx) || defined(STM32F412Vx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FSMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FSMC_Bank1->BTCR[2]  = 0x00001011;
  FSMC_Bank1->BTCR[3]  = 0x00000201;
  FSMC_Bank1E->BWTR[2] = 0x0FFFFFFF;
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F412Zx || STM32F412Vx */

#endif /* DATA_IN_ExtSRAM */
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx || STM32F437xx ||\
          STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx  */ 
  (void)(tmp); 
}
#endif /* DATA_IN_ExtSRAM && DATA_IN_ExtSDRAM */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
