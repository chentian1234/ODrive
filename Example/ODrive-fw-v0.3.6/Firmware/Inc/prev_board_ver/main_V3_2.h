/*
 * ============================================================================
 * 文件名: main_V3_2.h
 *
 * 文件用途:
 *   本文件定义ODrive V3.2版本硬件的所有GPIO引脚宏定义和定时器参数。
 *   包含：
 *     - 定时器时钟频率和周期配置（TIM1/8: 168MHz, TIM_APB1: 84MHz）
 *     - 死区时间配置（TIM1/8: 20时钟周期, TIM_APB1: 40时钟周期）
 *     - 所有GPIO引脚定义（M0/M1三相PWM、电流采样、编码器、SPI片选等）
 *     - 外部中断引脚定义（GPIO_1/2/3的EXTI中断）
 *
 * 注意：此文件适用于V3.2版本硬件，新版本硬件请使用main.h
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Private define ------------------------------------------------------------*/
/* ============================================================================
 * 定时器参数配置
 * ============================================================================ */
#define TIM_1_8_CLOCK_HZ 168000000        /* TIM1/TIM8时钟频率: 168MHz (APB2) */
#define TIM_1_8_PERIOD_CLOCKS 10192       /* TIM1/TIM8周期: 10192 → PWM频率 ≈ 16.48kHz */
#define TIM_1_8_DEADTIME_CLOCKS 20        /* TIM1/TIM8死区时间: 20时钟周期 ≈ 119ns */
#define TIM_APB1_CLOCK_HZ 84000000        /* TIM2/TIM3/TIM4/TIM5时钟频率: 84MHz (APB1) */
#define TIM_APB1_PERIOD_CLOCKS 4096       /* TIM2周期: 4096 → PWM频率 ≈ 20.5kHz */
#define TIM_APB1_DEADTIME_CLOCKS 40       /* TIM2死区时间: 40时钟周期 ≈ 476ns */

/* ============================================================================
 * GPIO引脚定义 - SPI片选和电流校准
 * ============================================================================ */
#define M0_nCS_Pin GPIO_PIN_13            /* M0 DRV8301 SPI片选 */
#define M0_nCS_GPIO_Port GPIOC
#define M1_nCS_Pin GPIO_PIN_14            /* M1 DRV8301 SPI片选 */
#define M1_nCS_GPIO_Port GPIOC
#define M1_DC_CAL_Pin GPIO_PIN_15         /* M1电流采样直流校准ADC输入 */
#define M1_DC_CAL_GPIO_Port GPIOC

/* ============================================================================
 * GPIO引脚定义 - 电流采样 (ADC输入)
 * ============================================================================ */
#define M0_IB_Pin GPIO_PIN_0              /* M0 B相电流采样 */
#define M0_IB_GPIO_Port GPIOC
#define M0_IC_Pin GPIO_PIN_1              /* M0 C相电流采样 */
#define M0_IC_GPIO_Port GPIOC
#define M1_IC_Pin GPIO_PIN_2              /* M1 C相电流采样 */
#define M1_IC_GPIO_Port GPIOC
#define M1_IB_Pin GPIO_PIN_3              /* M1 B相电流采样 */
#define M1_IB_GPIO_Port GPIOC

/* ============================================================================
 * GPIO引脚定义 - 模拟输入 (ADC)
 * ============================================================================ */
#define VBUS_S_Pin GPIO_PIN_0             /* 母线电压采样 */
#define VBUS_S_GPIO_Port GPIOA
#define M1_TEMP_Pin GPIO_PIN_1            /* M1温度采样 */
#define M1_TEMP_GPIO_Port GPIOA
#define AUX_I_Pin GPIO_PIN_2              /* 辅助电流采样 */
#define AUX_I_GPIO_Port GPIOA
#define GPIO_4_Pin GPIO_PIN_3             /* 通用GPIO4 (UART/Step-Dir复用) */
#define GPIO_4_GPIO_Port GPIOA
#define GPIO_3_Pin GPIO_PIN_4             /* 通用GPIO3 (UART/Step-Dir复用) */
#define GPIO_3_GPIO_Port GPIOA
#define GPIO_3_EXTI_IRQn EXTI4_IRQn       /* GPIO3外部中断 */
#define GPIO_2_Pin GPIO_PIN_5             /* 通用GPIO2 (UART/Step-Dir复用) */
#define GPIO_2_GPIO_Port GPIOA
#define AUX_V_Pin GPIO_PIN_6              /* 辅助电压采样 */
#define AUX_V_GPIO_Port GPIOA
#define M1_AL_Pin GPIO_PIN_7              /* M1 A相低边电流采样 */
#define M1_AL_GPIO_Port GPIOA
#define AUX_TEMP_Pin GPIO_PIN_4           /* 辅助温度采样 */
#define AUX_TEMP_GPIO_Port GPIOC
#define M0_TEMP_Pin GPIO_PIN_5            /* M0温度采样 */
#define M0_TEMP_GPIO_Port GPIOC

/* ============================================================================
 * GPIO引脚定义 - M1三相低边PWM
 * ============================================================================ */
#define M1_BL_Pin GPIO_PIN_0              /* M1 B相低边 */
#define M1_BL_GPIO_Port GPIOB
#define M1_CL_Pin GPIO_PIN_1              /* M1 C相低边 */
#define M1_CL_GPIO_Port GPIOB

/* ============================================================================
 * GPIO引脚定义 - 通用GPIO (带外部中断)
 * ============================================================================ */
#define GPIO_1_Pin GPIO_PIN_2             /* 通用GPIO1 (UART/Step-Dir复用) */
#define GPIO_1_GPIO_Port GPIOB
#define GPIO_1_EXTI_IRQn EXTI2_IRQn       /* GPIO1外部中断 */

/* ============================================================================
 * GPIO引脚定义 - 辅助端口PWM输出
 * ============================================================================ */
#define AUX_L_Pin GPIO_PIN_10             /* 辅助端口低边 */
#define AUX_L_GPIO_Port GPIOB
#define AUX_H_Pin GPIO_PIN_11             /* 辅助端口高边 */
#define AUX_H_GPIO_Port GPIOB

/* ============================================================================
 * GPIO引脚定义 - DRV8301控制和M0三相低边PWM
 * ============================================================================ */
#define EN_GATE_Pin GPIO_PIN_12           /* DRV8301栅极驱动使能 */
#define EN_GATE_GPIO_Port GPIOB
#define M0_AL_Pin GPIO_PIN_13             /* M0 A相低边 */
#define M0_AL_GPIO_Port GPIOB
#define M0_BL_Pin GPIO_PIN_14             /* M0 B相低边 */
#define M0_BL_GPIO_Port GPIOB
#define M0_CL_Pin GPIO_PIN_15             /* M0 C相低边 */
#define M0_CL_GPIO_Port GPIOB

/* ============================================================================
 * GPIO引脚定义 - M1三相高边PWM
 * ============================================================================ */
#define M1_AH_Pin GPIO_PIN_6              /* M1 A相高边 */
#define M1_AH_GPIO_Port GPIOC
#define M1_BH_Pin GPIO_PIN_7              /* M1 B相高边 */
#define M1_BH_GPIO_Port GPIOC
#define M1_CH_Pin GPIO_PIN_8              /* M1 C相高边 */
#define M1_CH_GPIO_Port GPIOC

/* ============================================================================
 * GPIO引脚定义 - M0直流校准和三相高边PWM
 * ============================================================================ */
#define M0_DC_CAL_Pin GPIO_PIN_9          /* M0电流采样直流校准ADC输入 */
#define M0_DC_CAL_GPIO_Port GPIOC
#define M0_AH_Pin GPIO_PIN_8              /* M0 A相高边 */
#define M0_AH_GPIO_Port GPIOA
#define M0_BH_Pin GPIO_PIN_9              /* M0 B相高边 */
#define M0_BH_GPIO_Port GPIOA
#define M0_CH_Pin GPIO_PIN_10             /* M0 C相高边 */
#define M0_CH_GPIO_Port GPIOA

/* ============================================================================
 * GPIO引脚定义 - 编码器接口
 * ============================================================================ */
#define M0_ENC_Z_Pin GPIO_PIN_15          /* M0编码器Index(Z相)脉冲 */
#define M0_ENC_Z_GPIO_Port GPIOA
#define nFAULT_Pin GPIO_PIN_2             /* DRV8301故障信号(低电平有效) */
#define nFAULT_GPIO_Port GPIOD
#define M1_ENC_Z_Pin GPIO_PIN_3           /* M1编码器Index(Z相)脉冲 */
#define M1_ENC_Z_GPIO_Port GPIOB
#define M0_ENC_A_Pin GPIO_PIN_4           /* M0编码器A相 */
#define M0_ENC_A_GPIO_Port GPIOB
#define M0_ENC_B_Pin GPIO_PIN_5           /* M0编码器B相 */
#define M0_ENC_B_GPIO_Port GPIOB
#define M1_ENC_A_Pin GPIO_PIN_6           /* M1编码器A相 */
#define M1_ENC_A_GPIO_Port GPIOB
#define M1_ENC_B_Pin GPIO_PIN_7           /* M1编码器B相 */
#define M1_ENC_B_GPIO_Port GPIOB
