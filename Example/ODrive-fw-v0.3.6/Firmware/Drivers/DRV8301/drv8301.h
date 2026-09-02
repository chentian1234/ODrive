// ============================================================================
// 文件说明: drv8301.h
// 芯片型号: TI DRV8301 三相栅极驱动器
// 功能概述: 本头文件定义了 DRV8301 驱动所需的所有公共接口, 包括:
//   - SPI 控制字的位掩码 (地址掩码、数据掩码、读写位)
//   - 状态寄存器 / 控制寄存器每一位的掩码宏
//   - 读写模式、过流模式、过流阈值、PWM 模式、运放增益等枚举
//   - DRV8301_Obj 驱动对象结构体
//   - DRV_SPI_8301_Vars_t 高层配置 / 状态结构体 (便于后台循环周期读写)
//   - 全部驱动 API 的函数声明
// ============================================================================
#ifndef _DRV8301_H_
#define _DRV8301_H_

#include "stdbool.h"
#include "stdint.h"

#include "stm32f4xx_hal.h"

// ============================================================================
// 平台适配层: 把 TI C2000 系列的 HAL 类型映射为 STM32 HAL 类型
//   SPI_Handle      -> SPI_HandleTypeDef*     (HAL 初始化好的 SPI 句柄)
//   GPIO_Handle     -> GPIO_TypeDef*          (如 GPIOA / GPIOB)
//   GPIO_Number_e   -> uint16_t               (如 GPIO_PIN_10)
// ============================================================================
typedef SPI_HandleTypeDef* SPI_Handle;
typedef GPIO_TypeDef* GPIO_Handle;
typedef uint16_t GPIO_Number_e;


// DRV8301 SPI 控制字格式 (16bit):
//   Bit15 : R/W      (0 = 写, 1 = 读)
//   Bit14 : 保留     (写时填 0)
//   Bit13-11 : 寄存器地址
//   Bit10-0  : 数据字段
// 下方三个宏对应这三个区域的位掩码
#define DRV8301_ADDR_MASK               (0x7800)   // 寄存器地址掩码 (Bit13-11, 0x7 = 111, 左移 11 位)

#define DRV8301_DATA_MASK               (0x07FF)   // 数据字段掩码 (Bit10-0, 共 11bit 有效数据)

#define DRV8301_RW_MASK                 (0x8000)   // 读/写控制位掩码 (Bit15, 最高位)

#define DRV8301_FAULT_TYPE_MASK         (0x07FF)   // 故障类型掩码, 与 DATA_MASK 相同,
                                                     // 因为故障信息全部落在数据字段 Bit10-0 内


// ============================================================================
// 状态寄存器 1 (Status Register 1) 位定义
//   读地址 = 0 << 11
//   各位含义:
//     Bit0-5 : 三相 (C/B/A) 的低侧 FET / 高侧 FET 过流标志 (共 6 位)
//     Bit6   : OTW 过温警告
//     Bit7   : OTSD 过温关断 (更严重, 直接关 PWM)
//     Bit8   : PVDD_UV 功率电源欠压
//     Bit9   : GVDD_UV 栅极驱动电源欠压
//     Bit10  : FAULT  总故障标志 (任一保护触发都会置 1)
// ============================================================================
#define DRV8301_STATUS1_FETLC_OC_BITS   (1 << 0)   // C 相低侧 FET 过流
#define DRV8301_STATUS1_FETHC_OC_BITS   (1 << 1)   // C 相高侧 FET 过流
#define DRV8301_STATUS1_FETLB_OC_BITS   (1 << 2)   // B 相低侧 FET 过流
#define DRV8301_STATUS1_FETHB_OC_BITS   (1 << 3)   // B 相高侧 FET 过流
#define DRV8301_STATUS1_FETLA_OC_BITS   (1 << 4)   // A 相低侧 FET 过流
#define DRV8301_STATUS1_FETHA_OC_BITS   (1 << 5)   // A 相高侧 FET 过流
#define DRV8301_STATUS1_OTW_BITS        (1 << 6)   // 过温警告 (Over Temperature Warning), 轻微
#define DRV8301_STATUS1_OTSD_BITS       (1 << 7)   // 过温关断 (Over Temperature Shut Down), 严重
#define DRV8301_STATUS1_PVDD_UV_BITS    (1 << 8)   // PVDD 功率电源欠压 (通常指电机母线供电)
#define DRV8301_STATUS1_GVDD_UV_BITS    (1 << 9)   // GVDD 栅极驱动电源欠压 (自举或 LDO 供电)
#define DRV8301_STATUS1_FAULT_BITS      (1 << 10)  // FAULT 总故障标志, 任何保护触发都会置位


// ============================================================================
// 状态寄存器 2 (Status Register 2) 位定义
//   读地址 = 1 << 11
// ============================================================================
#define DRV8301_STATUS2_ID_BITS        (15 << 0)   // Device ID, 低 4bit, 标识 DRV8301 芯片版本号
#define DRV8301_STATUS2_GVDD_OV_BITS    (1 << 7)   // GVDD 栅极驱动电源过压


// ============================================================================
// 控制寄存器 1 (Control Register 1) 位定义
//   写地址 = 2 << 11
//   各位含义:
//     Bit0-1  : GATE_CURRENT  栅极驱动峰值电流 (0=1.70A, 1=0.70A, 2=0.25A)
//     Bit2    : GATE_RESET    软复位位 (写 1 触发复位)
//     Bit3    : PWM_MODE      PWM 输入模式 (0=六输入, 1=三输入)
//     Bit4-5  : OC_MODE       过流保护模式
//     Bit6-10 : OC_ADJ_SET    过流 Vds 阈值 (32 档, 0.060V ~ 2.400V)
// ============================================================================
#define DRV8301_CTRL1_GATE_CURRENT_BITS  (3 << 0)   // 栅极驱动峰值电流字段 (2bit)
#define DRV8301_CTRL1_GATE_RESET_BITS    (1 << 2)   // 软复位位, 写 1 触发 DRV8301 复位
#define DRV8301_CTRL1_PWM_MODE_BITS      (1 << 3)   // PWM 输入模式字段 (1bit)
#define DRV8301_CTRL1_OC_MODE_BITS       (3 << 4)   // 过流保护响应模式 (2bit)
#define DRV8301_CTRL1_OC_ADJ_SET_BITS   (31 << 6)   // 过流 Vds 阈值 (5bit, 32 档)


// ============================================================================
// 控制寄存器 2 (Control Register 2) 位定义
//   写地址 = 3 << 11
//   各位含义:
//     Bit0-1  : OCTW_SET     过流 + 过温在 /OCTW 引脚上的组合行为
//     Bit2-3  : GAIN         分流放大器增益 (10/20/40/80 V/V)
//     Bit4    : DC_CAL_1     分流放大器 Ch1 校准模式 (0=Load, 1=NoLoad 短路校准)
//     Bit5    : DC_CAL_2     分流放大器 Ch2 校准模式
//     Bit6    : OC_TOFF      过流保护关断时间模式
// ============================================================================
#define DRV8301_CTRL2_OCTW_SET_BITS      (3 << 0)   // OCTW 引脚报告模式 (2bit)
#define DRV8301_CTRL2_GAIN_BITS          (3 << 2)   // 分流放大器增益 (2bit)
#define DRV8301_CTRL2_DC_CAL_1_BITS      (1 << 4)   // Ch1 DC 校准模式 (1bit)
#define DRV8301_CTRL2_DC_CAL_2_BITS      (1 << 5)   // Ch2 DC 校准模式 (1bit)
#define DRV8301_CTRL2_OC_TOFF_BITS       (1 << 6)   // 过流关断时间模式 (1bit)


// ============================================================================
// 枚举定义
// ============================================================================

// DRV8301 SPI 读写控制模式
//   R/W 位在控制字最高位 Bit15
typedef enum
{
  DRV8301_CtrlMode_Read  = 1 << 15,   // 读模式, 控制字 Bit15 = 1
  DRV8301_CtrlMode_Write = 0 << 15    // 写模式, 控制字 Bit15 = 0
} DRV8301_CtrlMode_e;


// 分流放大器 DC 校准模式
//   分流放大器用于放大采样电阻上的小压降. 在"正常 Load 模式"下
//   直接输出采样结果; 在"NoLoad 短路校准模式"下内部短路输入端,
//   用于 ADC 零点校准, 消除运放失调和 PCB 走线偏移.
typedef enum
{
  DRV8301_DcCalMode_Ch1_Load   = (0 << 4),   // Ch1: 正常测量, 接负载
  DRV8301_DcCalMode_Ch1_NoLoad = (1 << 4),   // Ch1: 输入端短路, 用于零点校准
  DRV8301_DcCalMode_Ch2_Load   = (0 << 5),   // Ch2: 正常测量
  DRV8301_DcCalMode_Ch2_NoLoad = (1 << 5)    // Ch2: 短路校准
} DRV8301_DcCalMode_e;


// 故障类型枚举. 每一项直接对应状态寄存器 1/2 的故障位.
//   多个故障可能同时置位, 因为是位标志的组合.
typedef enum
{
  DRV8301_FaultType_NoFault  = (0 << 0),  // 无任何故障
  DRV8301_FaultType_FETLC_OC = (1 << 0),  // C 相低侧 FET 过流
  DRV8301_FaultType_FETHC_OC = (1 << 1),  // C 相高侧 FET 过流
  DRV8301_FaultType_FETLB_OC = (1 << 2),  // B 相低侧 FET 过流
  DRV8301_FaultType_FETHB_OC = (1 << 3),  // B 相高侧 FET 过流
  DRV8301_FaultType_FETLA_OC = (1 << 4),  // A 相低侧 FET 过流
  DRV8301_FaultType_FETHA_OC = (1 << 5),  // A 相高侧 FET 过流
  DRV8301_FaultType_OTW      = (1 << 6),  // 过温警告 (轻微, 仍可运行)
  DRV8301_FaultType_OTSD     = (1 << 7),  // 过温关断 (严重, 强制关 PWM)
  DRV8301_FaultType_PVDD_UV  = (1 << 8),  // PVDD 功率电源欠压
  DRV8301_FaultType_GVDD_UV  = (1 << 9),  // GVDD 栅极驱动电源欠压
  DRV8301_FaultType_GVDD_OV  = (1 << 10)  // GVDD 栅极驱动电源过压 (来自状态寄存器 2)
} DRV8301_FaultType_e;


// 过流保护响应模式
//   CurrentLimit  : 逐周期限流, 过流关断后下一个 PWM 周期自动重启, 适合电机软启动
//   LatchShutDown : 锁存关断, 一旦触发过流就永久关断, 必须软件复位才能恢复, 安全级别最高
//   ReportOnly    : 只报告过流事件, 不做任何硬件保护
//   Disabled      : 完全关闭过流保护
typedef enum
{
  DRV8301_OcMode_CurrentLimit  = 0 << 4,   // 逐周期限流模式, 最常用
  DRV8301_OcMode_LatchShutDown = 1 << 4,   // 锁存关断, 触发后需软件复位
  DRV8301_OcMode_ReportOnly    = 2 << 4,   // 只上报, 不关断
  DRV8301_OcMode_Disabled      = 3 << 4    // 关闭过流保护 (不建议)
} DRV8301_OcMode_e;


// 过流保护关断时间模式
//   Normal  : 正常 CBC (Cycle-By-Cycle) 逐周期工作, 过流后关断时间 = 固有死区时间
//   Ctrl    : 过流期间延长关断时间, 适合大感性负载 / 防饱和
typedef enum
{
  DRV8301_OcOffTimeMode_Normal  = 0 << 6,   // 正常模式, 使用固定死区时间
  DRV8301_OcOffTimeMode_Ctrl    = 1 << 6    // 延长关断时间模式
} DRV8301_OcOffTimeMode_e;


// /OCTW 引脚 (Over Current / Temperature Warning) 的报告组合
//   DRV8301 有一个物理引脚 /OCTW, 用来同时提示过流和过温事件.
//   本枚举决定该引脚在事件发生时的行为:
//     Both    : 过流或过温任一发生都拉低 /OCTW
//     OT_Only : 只在过温时才拉低 /OCTW
//     OC_Only : 只在过流时才拉低 /OCTW
typedef enum
{
  DRV8301_OcTwMode_Both    = 0 << 0,   // 同时报告过流和过温
  DRV8301_OcTwMode_OT_Only = 1 << 0,   // 仅过温时触发
  DRV8301_OcTwMode_OC_Only = 2 << 0    // 仅过流时触发
} DRV8301_OcTwMode_e;


// 栅极驱动峰值电流 (决定 MOSFET 开关速度)
//   电流越大 -> 开关越快 (dt 小), 但 EMI 越大
//   电流越小 -> 开关越慢, 开关损耗越大但 EMI 小
//   选值需要根据 MOSFET 栅极电荷 Qg 和期望的 di/dt 折中
typedef enum
{
  DRV8301_PeakCurrent_1p70_A  = 0 << 0,   // 最大, 适合大栅极电荷 MOSFET
  DRV8301_PeakCurrent_0p70_A  = 1 << 0,   // 中等
  DRV8301_PeakCurrent_0p25_A  = 2 << 0    // 最小, 默认值
} DRV8301_PeakCurrent_e;


// PWM 输入模式
//   Six_Inputs   : 六输入模式, MCU 给 DRV8301 6 路独立 PWM (每相高/低侧各一路)
//                  灵活度最高, 支持不对称死区等高级特性
//   Three_Inputs : 三输入模式, MCU 只给 3 路 PWM, DRV8301 内部自动生成互补 PWM
//                  MCU 端只需要 Timer PWM 输出, 硬件成本低
typedef enum
{
  DRV8301_PwmMode_Six_Inputs   = 0 << 3,   // 六输入独立控制 (灵活度最高)
  DRV8301_PwmMode_Three_Inputs = 1 << 3    // 三输入互补模式 (MCU 端硬件简单)
} DRV8301_PwmMode_e;


// DRV8301 四个寄存器的 SPI 地址编码
//   地址位于 SPI 控制字 Bit13-11, 左移 11 位
typedef enum
{
  DRV8301_RegName_Status_1   = 0 << 11,   // 状态寄存器 1: 各种故障标志
  DRV8301_RegName_Status_2   = 1 << 11,   // 状态寄存器 2: GVDD_OV + DeviceID
  DRV8301_RegName_Control_1  = 2 << 11,   // 控制寄存器 1: 栅极电流/PWM模式/OC配置
  DRV8301_RegName_Control_2  = 3 << 11    // 控制寄存器 2: GAIN/DC校准/OC_TOFF
} DRV8301_RegName_e;


// 软复位位配置
//   Normal = 正常运行
//   All    = 软复位, 写 1 到控制寄存器 1 的 Bit2 即可触发
typedef enum
{
  DRV8301_Reset_Normal = 0 << 2,   // 正常模式
  DRV8301_Reset_All    = 1 << 2    // 触发软复位
} DRV8301_Reset_e;


// 分流放大器增益 (单位: V/V, 每 V 输入产生多少 V 输出)
//   增益越高 -> 电流分辨率越高, 但动态范围越窄
//   选择依据: 分流电阻 R_sense 阻值 × 最大电流 ≤ Vref / Gain
//     其中 Vref 是 MCU ADC 参考电压 (常见 3.3V)
typedef enum
{
  DRV8301_ShuntAmpGain_10VpV = 0 << 2,   // 10 倍, 适合大电流低分辨率场景
  DRV8301_ShuntAmpGain_20VpV = 1 << 2,   // 20 倍
  DRV8301_ShuntAmpGain_40VpV = 2 << 2,   // 40 倍
  DRV8301_ShuntAmpGain_80VpV = 3 << 2    // 80 倍, 小电流高分辨率 (DRV8301 仅部分型号支持)
} DRV8301_ShuntAmpGain_e;


// 分流放大器编号, 用于 DC 校准时指定通道
typedef enum
{
  DRV8301_ShuntAmpNumber_1 = 1,      // Ch1 分流放大器
  DRV8301_ShuntAmpNumber_2 = 2       // Ch2 分流放大器
} DRV8301_ShuntAmpNumber_e;


// 过流保护 Vds 阈值, 共 32 档 (0.060V ~ 2.400V)
//   DRV8301 内部实时检测每个 MOSFET 的漏源电压 Vds
//   当 Vds > 设定阈值时认为该 MOSFET 发生过流, 触发保护
//   Vds_threshold = OC_ADJ_VAL × 0.05V + 0.060V (大致)
//   典型选型: 0.730V 对应的 OC_ADJ_VAL ≈ 21, 是最常用的默认值
typedef enum
{
  DRV8301_VdsLevel_0p060_V =  0 << 6,   // 0.060 V (最灵敏)
  DRV8301_VdsLevel_0p068_V =  1 << 6,
  DRV8301_VdsLevel_0p076_V =  2 << 6,
  DRV8301_VdsLevel_0p086_V =  3 << 6,
  DRV8301_VdsLevel_0p097_V =  4 << 6,
  DRV8301_VdsLevel_0p109_V =  5 << 6,
  DRV8301_VdsLevel_0p123_V =  6 << 6,
  DRV8301_VdsLevel_0p138_V =  7 << 6,
  DRV8301_VdsLevel_0p155_V =  8 << 6,
  DRV8301_VdsLevel_0p175_V =  9 << 6,
  DRV8301_VdsLevel_0p197_V = 10 << 6,
  DRV8301_VdsLevel_0p222_V = 11 << 6,
  DRV8301_VdsLevel_0p250_V = 12 << 6,
  DRV8301_VdsLevel_0p282_V = 13 << 6,
  DRV8301_VdsLevel_0p317_V = 14 << 6,
  DRV8301_VdsLevel_0p358_V = 15 << 6,
  DRV8301_VdsLevel_0p403_V = 16 << 6,
  DRV8301_VdsLevel_0p454_V = 17 << 6,
  DRV8301_VdsLevel_0p511_V = 18 << 6,
  DRV8301_VdsLevel_0p576_V = 19 << 6,
  DRV8301_VdsLevel_0p648_V = 20 << 6,
  DRV8301_VdsLevel_0p730_V = 21 << 6,   // 默认推荐值, 通用场景
  DRV8301_VdsLevel_0p822_V = 22 << 6,
  DRV8301_VdsLevel_0p926_V = 23 << 6,
  DRV8301_VdsLevel_1p043_V = 24 << 6,
  DRV8301_VdsLevel_1p175_V = 25 << 6,
  DRV8301_VdsLevel_1p324_V = 26 << 6,
  DRV8301_VdsLevel_1p491_V = 27 << 6,
  DRV8301_VdsLevel_1p679_V = 28 << 6,
  DRV8301_VdsLevel_1p892_V = 29 << 6,
  DRV8301_VdsLevel_2p131_V = 30 << 6,
  DRV8301_VdsLevel_2p400_V = 31 << 6    // 2.400 V (最不灵敏)
} DRV8301_VdsLevel_e;


// 预留枚举, 原始 TI 驱动保留的占位
typedef enum
{
  DRV8301_GETID=0
} Drv8301SpiOutputDataSelect_e;


// ============================================================================
// 高层配置 / 状态结构体 (用于后台循环周期读写)
//   这些结构体把 DRV8301 四个寄存器的每个 bit 拆分成可读的字段,
//   方便在调试器 Watch 窗口直接观察和修改
// ============================================================================

// 状态寄存器 1 的解析结构体
typedef struct _DRV_SPI_8301_Stat1_t_
{
  bool                  FAULT;       // 总故障标志, 任何保护触发 = true
  bool                  GVDD_UV;     // GVDD 欠压
  bool                  PVDD_UV;     // PVDD 欠压
  bool                  OTSD;        // 过温关断
  bool                  OTW;         // 过温警告
  bool                  FETHA_OC;    // A 相高侧 FET 过流
  bool                  FETLA_OC;    // A 相低侧 FET 过流
  bool                  FETHB_OC;    // B 相高侧 FET 过流
  bool                  FETLB_OC;    // B 相低侧 FET 过流
  bool                  FETHC_OC;    // C 相高侧 FET 过流
  bool                  FETLC_OC;    // C 相低侧 FET 过流
}DRV_SPI_8301_Stat1_t_;


// 状态寄存器 2 的解析结构体
typedef struct _DRV_SPI_8301_Stat2_t_
{
  bool                  GVDD_OV;      // GVDD 过压
  uint16_t              DeviceID;     // 芯片版本号 (低 4bit 有效)
}DRV_SPI_8301_Stat2_t_;


// 控制寄存器 1 的解析结构体
typedef struct _DRV_SPI_8301_CTRL1_t_
{
  DRV8301_PeakCurrent_e    DRV8301_CURRENT;   // 栅极驱动峰值电流
  DRV8301_Reset_e          DRV8301_RESET;      // 软复位位
  DRV8301_PwmMode_e        PWM_MODE;           // PWM 输入模式
  DRV8301_OcMode_e         OC_MODE;            // 过流保护模式
  DRV8301_VdsLevel_e       OC_ADJ_SET;         // 过流 Vds 阈值
}DRV_SPI_8301_CTRL1_t_;


// 控制寄存器 2 的解析结构体
typedef struct _DRV_SPI_8301_CTRL2_t_
{
  DRV8301_OcTwMode_e       OCTW_SET;       // /OCTW 引脚组合报告模式
  DRV8301_ShuntAmpGain_e   GAIN;           // 分流放大器增益
  DRV8301_DcCalMode_e      DC_CAL_CH1p2;   // DC 校准模式 (含 Ch1 和 Ch2 两个通道的位)
  DRV8301_OcOffTimeMode_e  OC_TOFF;        // 过流关断时间模式
}DRV_SPI_8301_CTRL2_t_;


// DRV8301 SPI 变量总结构体, 在后台循环中通过 writeData/readData 周期更新
typedef struct _DRV_SPI_8301_Vars_t_
{
  DRV_SPI_8301_Stat1_t_     Stat_Reg_1;   // 状态寄存器 1 解析结果
  DRV_SPI_8301_Stat2_t_     Stat_Reg_2;   // 状态寄存器 2 解析结果
  DRV_SPI_8301_CTRL1_t_     Ctrl_Reg_1;   // 控制寄存器 1 解析/设置
  DRV_SPI_8301_CTRL2_t_     Ctrl_Reg_2;   // 控制寄存器 2 解析/设置
  bool                      SndCmd;       // 写命令标志, 后台循环检测到 true 时把 Ctrl_Reg_1/2 写入 DRV8301
  bool                      RcvCmd;       // 读命令标志, 后台循环检测到 true 时从 DRV8301 读回所有寄存器
}DRV_SPI_8301_Vars_t;


// ============================================================================
// DRV8301 驱动对象结构体 (底层对象)
//   保存所有与 DRV8301 交互所需的硬件句柄和内部状态标志
//   由 DRV8301_init() 初始化, 之后通过 setSpiHandle / setEnGpioHandle 等
//   函数把 STM32 HAL 句柄关联进来
// ============================================================================
typedef struct _DRV8301_Obj_
{
  SPI_Handle       spiHandle;                  // HAL SPI 句柄, 所有寄存器通信都走这个 SPI
  GPIO_Handle      EngpioHandle;               // EN_GATE 引脚所属 GPIO 端口 (如 GPIOA)
  GPIO_Number_e    EngpioNumber;               // EN_GATE 具体引脚号 (如 GPIO_PIN_10)
  GPIO_Handle      nCSgpioHandle;              // SPI 片选 nCS 引脚所属 GPIO 端口
  GPIO_Number_e    nCSgpioNumber;              // SPI 片选 nCS 具体引脚号
  bool             RxTimeOut;                  // SPI RX 超时标志 (预留, 供异步通信时使用)
  bool             enableTimeOut;              // 使能超时标志 (预留, 用于检测 DRV8301 能否正常启动)
} DRV8301_Obj;


// DRV8301 驱动对象的指针句柄类型, 所有驱动函数都通过 handle 参数访问对象
typedef struct _DRV8301_Obj_ *DRV8301_Handle;


// DRV8301 SPI 单次通信的 16bit 字类型, 等价于 uint16_t
typedef  uint16_t    DRV8301_Word_t;


// ============================================================================
// 函数声明
// ============================================================================

// ----------------------------------------------------------------------------
// 内部辅助函数: 构造 SPI 控制字 (inline, 头文件中实现)
//   将读写模式 + 寄存器地址 + 数据字段 按位或组合成 16bit SPI 控制字
// ----------------------------------------------------------------------------
static inline DRV8301_Word_t DRV8301_buildCtrlWord(const DRV8301_CtrlMode_e ctrlMode,
                                                   const DRV8301_RegName_e regName,
                                                   const uint16_t data)
{
  DRV8301_Word_t ctrlWord = ctrlMode | regName | (data & DRV8301_DATA_MASK);

  return(ctrlWord);
}


// ----------------------------------------------------------------------------
// DC 校准模式相关 API
// ----------------------------------------------------------------------------

// 获取指定分流放大器 (1 或 2) 的 DC 校准模式
extern DRV8301_DcCalMode_e DRV8301_getDcCalMode(DRV8301_Handle handle,
                                                const DRV8301_ShuntAmpNumber_e ampNumber);

// 设置指定分流放大器的 DC 校准模式
extern void DRV8301_setDcCalMode(DRV8301_Handle handle,
                                 const DRV8301_ShuntAmpNumber_e ampNumber,
                                 const DRV8301_DcCalMode_e mode);


// ----------------------------------------------------------------------------
// 使能 / 复位 API
// ----------------------------------------------------------------------------

// 使能 DRV8301: EN_GATE 拉高, 等待启动, 轮询 FAULT 直到清零
extern void DRV8301_enable(DRV8301_Handle handle);

// 触发 DRV8301 软复位: 重置所有寄存器, 清除锁存故障
extern void DRV8301_reset(DRV8301_Handle handle);

// 内部 inline: 清零 enableTimeOut 超时标志
static inline void DRV8301_resetEnableTimeout(DRV8301_Handle handle)
{
  DRV8301_Obj *obj = (DRV8301_Obj *)handle;

  obj->enableTimeOut = false;

  return;
}

// 内部 inline: 清零 RxTimeOut 超时标志
static inline void DRV8301_resetRxTimeout(DRV8301_Handle handle)
{
  DRV8301_Obj *obj = (DRV8301_Obj *)handle;

  obj->RxTimeOut = false;

  return;
}


// ----------------------------------------------------------------------------
// 故障检测 API
// ----------------------------------------------------------------------------

// 判断 DRV8301 是否发生任何故障 (FAULT 总标志位)
extern bool DRV8301_isFault(DRV8301_Handle handle);

// 判断 DRV8301 是否处于软复位状态 (GATE_RESET 位)
extern bool DRV8301_isReset(DRV8301_Handle handle);

// 获取具体故障类型 (可能返回组合值, 多个故障同时置位)
extern DRV8301_FaultType_e DRV8301_getFaultType(DRV8301_Handle handle);


// ----------------------------------------------------------------------------
// 设备信息 API
// ----------------------------------------------------------------------------

// 读取 DRV8301 的 DeviceID (低 4bit, 芯片版本号)
extern uint16_t DRV8301_getId(DRV8301_Handle handle);


// ----------------------------------------------------------------------------
// 配置读取 API (get 系列)
// ----------------------------------------------------------------------------

// 读取当前过流 Vds 阈值
extern DRV8301_VdsLevel_e DRV8301_getOcLevel(DRV8301_Handle handle);

// 读取当前过流保护响应模式 (限流 / 锁存 / 仅上报 / 关闭)
extern DRV8301_OcMode_e DRV8301_getOcMode(DRV8301_Handle handle);

// 读取过流保护关断时间模式
extern DRV8301_OcOffTimeMode_e DRV8301_getOcOffTimeMode(DRV8301_Handle handle);

// 读取 /OCTW 引脚组合报告模式
extern DRV8301_OcTwMode_e DRV8301_getOcTwMode(DRV8301_Handle handle);

// 读取栅极驱动峰值电流设置
extern DRV8301_PeakCurrent_e DRV8301_getPeakCurrent(DRV8301_Handle handle);

// 读取 PWM 输入模式 (六输入 / 三输入)
extern DRV8301_PwmMode_e DRV8301_getPwmMode(DRV8301_Handle handle);

// 读取分流放大器增益 (10/20/40/80 V/V)
extern DRV8301_ShuntAmpGain_e DRV8301_getShuntAmpGain(DRV8301_Handle handle);

// 直接读取状态寄存器 1 的原始 16bit 值
extern uint16_t DRV8301_getStatusRegister1(DRV8301_Handle handle);

// 直接读取状态寄存器 2 的原始 16bit 值
extern uint16_t DRV8301_getStatusRegister2(DRV8301_Handle handle);


// ----------------------------------------------------------------------------
// 初始化 API
// ----------------------------------------------------------------------------

// 构造函数: 在给定内存上创建 DRV8301_Obj 对象, 返回句柄
//   参数 pMemory 指向用户预分配的内存, numBytes 必须 >= sizeof(DRV8301_Obj)
extern DRV8301_Handle DRV8301_init(void *pMemory,const size_t numBytes);


// ----------------------------------------------------------------------------
// 硬件句柄关联 API
// ----------------------------------------------------------------------------

// 设置 EN_GATE 引脚所属的 GPIO 端口 (如 GPIOA)
void DRV8301_setEnGpioHandle(DRV8301_Handle handle,GPIO_Handle gpioHandle);

// 设置 EN_GATE 的具体引脚号 (如 GPIO_PIN_10)
void DRV8301_setEnGpioNumber(DRV8301_Handle handle,GPIO_Number_e gpioNumber);

// 设置 SPI 片选 nCS 引脚所属的 GPIO 端口
void DRV8301_setnCSGpioHandle(DRV8301_Handle handle,GPIO_Handle gpioHandle);

// 设置 SPI 片选 nCS 的具体引脚号
void DRV8301_setnCSGpioNumber(DRV8301_Handle handle,GPIO_Number_e gpioNumber);

// 设置 DRV8301 使用的 SPI HAL 句柄 (如 &hspi1)
void DRV8301_setSpiHandle(DRV8301_Handle handle,SPI_Handle spiHandle);


// ----------------------------------------------------------------------------
// 配置写入 API (set 系列)
// ----------------------------------------------------------------------------

// 设置过流 Vds 阈值 (32 档)
extern void DRV8301_setOcLevel(DRV8301_Handle handle,const DRV8301_VdsLevel_e VdsLevel);

// 设置过流保护响应模式
extern void DRV8301_setOcMode(DRV8301_Handle handle,const DRV8301_OcMode_e mode);

// 设置过流保护关断时间模式
extern void DRV8301_setOcOffTimeMode(DRV8301_Handle handle,const DRV8301_OcOffTimeMode_e mode);

// 设置 /OCTW 引脚报告模式
extern void DRV8301_setOcTwMode(DRV8301_Handle handle,const DRV8301_OcTwMode_e mode);

// 设置栅极驱动峰值电流
extern void DRV8301_setPeakCurrent(DRV8301_Handle handle,const DRV8301_PeakCurrent_e peakCurrent);

// 设置 PWM 输入模式
extern void DRV8301_setPwmMode(DRV8301_Handle handle,const DRV8301_PwmMode_e mode);

// 设置分流放大器增益
extern void DRV8301_setShuntAmpGain(DRV8301_Handle handle,const DRV8301_ShuntAmpGain_e gain);


// ----------------------------------------------------------------------------
// 底层 SPI 读写 API
// ----------------------------------------------------------------------------

// 从 DRV8301 指定寄存器读取数据 (返回低 10bit, 已通过 DATA_MASK 过滤)
//   注意: 内部使用两次 SPI 事务 (先写读请求, 再产生时钟读数据)
extern uint16_t DRV8301_readSpi(DRV8301_Handle handle,const DRV8301_RegName_e regName);

// 向 DRV8301 指定寄存器写入数据 (单次 SPI 事务)
extern void DRV8301_writeSpi(DRV8301_Handle handle,const DRV8301_RegName_e regName,const uint16_t data);


// ----------------------------------------------------------------------------
// 高层批量读写 API (适合后台循环周期调用, 减少 SPI 事务)
// ----------------------------------------------------------------------------

// 批量写: 当 Spi_8301_Vars->SndCmd 为 true 时, 把 Ctrl_Reg_1 和 Ctrl_Reg_2
//         的所有字段一次性写入 DRV8301 的两个控制寄存器, 然后自动清零 SndCmd
extern void DRV8301_writeData(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars);

// 批量读: 当 Spi_8301_Vars->RcvCmd 为 true 时, 依次读取两个状态寄存器
//         和两个控制寄存器, 把每一位解析后填入 Stat_Reg_1/2 和 Ctrl_Reg_1/2,
//         然后自动清零 RcvCmd
extern void DRV8301_readData(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars);

// SPI 接口初始化: 清空 SndCmd / RcvCmd, 延时等待稳定, 主动读取一次所有寄存器
//         填充 Spi_8301_Vars, 为应用层提供一份初始快照
extern void DRV8301_setupSpi(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars);


#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DRV8301_H_