// ============================================================================
// 文件说明: drv8301.c
// 芯片型号: TI DRV8301 三相栅极驱动器 (带有三个电流采样放大器)
// 功能概述:
//   本文件实现了对 DRV8301 栅极驱动芯片的完整操作接口, 包含以下功能模块:
//     1. 初始化模块       - 创建/初始化 DRV8301 驱动对象, 关联 SPI 与 GPIO 句柄
//     2. 底层 SPI 读写    - 通过 SPI 对 DRV8301 的状态寄存器 / 控制寄存器进行读写
//     3. 控制寄存器配置   - 设置 PWM 模式, 过流保护, 栅极驱动电流, 三运放增益等
//     4. 状态寄存器读取   - 获取故障类型, 欠压/过压/过温/过流等保护状态
//     5. 高层接口         - writeData / readData / setupSpi 供后台循环周期调用
//
// 硬件连接要点 (参考 DRV8301 数据手册):
//   - EN_GATE : MCU GPIO -> DRV8301 使能输入 (高电平有效)
//   - nCS     : MCU SPI NSS -> DRV8301 片选 (低有效, SPI 通信必须)
//   - SCLK / MOSI / MISO : MCU SPI 主模式连接到 DRV8301 对应引脚
//   - nFAULT  : (可选) MCU GPIO 输入, DRV8301 故障输出, 低电平表示有故障
//
// SPI 通信协议 (DRV8301):
//   - 每次传输 16bit, 格式:
//       Bit15 : R/W  (0 = 写, 1 = 读)
//       Bit14 : 保留 (写时填 0, 读时忽略)
//       Bit12-11 : 寄存器地址 (R/W / 控制1 / 控制2 / 状态1 / 状态2)
//       Bit9-0  : 数据内容
//   - 写操作: 拉低 nCS -> 发送 16bit 控制字 -> 拉高 nCS, 数据在 nCS 上升沿被锁存
//   - 读操作: 需要分两次 SPI 事务, 详见 DRV8301_readSpi() 内部注释
// ============================================================================


#include "assert.h"
#include <math.h>
#include "freertos_vars.h"

#include "drv8301.h"


// ============================================================================
// 函数: DRV8301_enable
// 功能: 使能 DRV8301 栅极驱动器
// 参数: handle - DRV8301 驱动对象句柄
// 返回: 无
// 说明:
//   1. 将 EN_GATE 引脚拉高, 使驱动器输出 PWM 驱动信号
//   2. 等待 10ms 让 DRV8301 完成上电初始化 (数据手册要求的启动时间)
//   3. 轮询读取状态寄存器 1, 直到 FAULT 位清零, 确保启动过程中没有异常
//   4. 再延时 1ms, 让寄存器状态稳定, 供后续程序读取
// ============================================================================
void DRV8301_enable(DRV8301_Handle handle) {

    // 将 EN_GATE 引脚置高, 使能 DRV8301 栅极驱动输出
    HAL_GPIO_WritePin(handle->EngpioHandle, handle->EngpioNumber, GPIO_PIN_SET);

    // 等待 10ms, 给 DRV8301 足够的上电和内部初始化时间
    vTaskDelay(pdMS_TO_TICKS(10));

    // 循环读取状态寄存器 1 的 FAULT 位, 直到其为 0, 确保启动阶段无故障
    // DRV8301 刚上电时如果电源异常或自保护触发, FAULT 位会被置 1
    while ((DRV8301_readSpi(handle,DRV8301_RegName_Status_1) & DRV8301_STATUS1_FAULT_BITS) != 0);

    // 再延时 1ms, 让 DRV8301 内部寄存器状态更新完毕
    vTaskDelay(pdMS_TO_TICKS(1));

    return;
}


// ============================================================================
// 函数: DRV8301_getDcCalMode
// 功能: 获取指定分流放大器的 DC 校准模式
// 参数:
//   handle    - DRV8301 驱动对象句柄
//   ampNumber - 分流放大器编号 (1 或 2, 对应 Ch1 和 Ch2)
// 返回: DC 校准模式枚举值
// 说明:
//   DRV8301 内含三个电流采样放大器 (对应三相的相电流采样),
//   每个运放都可以独立设置校准模式: Load (正常测量) 或 Cal (短路校准).
//   本函数读取控制寄存器 2, 然后根据 ampNumber 清除其他通道的位,
//   从而得到指定通道的 DC 校准模式。
// ============================================================================
DRV8301_DcCalMode_e DRV8301_getDcCalMode(DRV8301_Handle handle,const DRV8301_ShuntAmpNumber_e ampNumber) {
    uint16_t data;

    // 先通过 SPI 读取控制寄存器 2 的完整内容
    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

    // 根据分流放大器编号, 清除另一个通道的 DC_CAL 位, 只保留目标通道的数据
    if (ampNumber == DRV8301_ShuntAmpNumber_1) {
        data &= (~DRV8301_CTRL2_DC_CAL_1_BITS);

    } else if (ampNumber == DRV8301_ShuntAmpNumber_2) {
        data &= (~DRV8301_CTRL2_DC_CAL_2_BITS);
    }

    // 将剩余的数据强转为枚举类型并返回
    return((DRV8301_DcCalMode_e)data);
}


// ============================================================================
// 函数: DRV8301_getFaultType
// 功能: 获取 DRV8301 最近发生的具体故障类型
// 参数: handle - DRV8301 驱动对象句柄
// 返回: 故障类型枚举 (过流 / 过温 / 欠压 / 过压 / 无故障)
// 说明:
//   DRV8301 的故障信息分布在两个状态寄存器中:
//     - 状态寄存器 1 (Status_1): FAULT 总标志位 + 具体过流 / 过温 / 欠压原因
//     - 状态寄存器 2 (Status_2): GVDD (栅极驱动电源) 过压 GVDD_OV
//   本函数首先检查 Status_1 的 FAULT 位是否为 1, 若为 1 则进一步解析故障源;
//   若 FAULT 位为 1 但 Status_1 中未能匹配到已知故障类型, 则去 Status_2 里
//   检查 GVDD_OV 位, 作为补充判定。
// ============================================================================
DRV8301_FaultType_e DRV8301_getFaultType(DRV8301_Handle handle) {
    DRV8301_Word_t      readWord;
    DRV8301_FaultType_e faultType = DRV8301_FaultType_NoFault;

    // 先读取状态寄存器 1, 查看是否有故障发生
    readWord = DRV8301_readSpi(handle,DRV8301_RegName_Status_1);

    // FAULT 总标志位为 1 说明 DRV8301 检测到了至少一种保护条件
    if (readWord & DRV8301_STATUS1_FAULT_BITS) {
        // 使用 FAULT_TYPE_MASK 掩码取出具体的故障子类型
        faultType = (DRV8301_FaultType_e)(readWord & DRV8301_FAULT_TYPE_MASK);

        // 特殊情况: FAULT=1 但解析出的故障类型却是 NoFault,
        // 说明故障源不在 Status_1 里, 需要进一步查看 Status_2
        if (faultType == DRV8301_FaultType_NoFault) {
            // 读取状态寄存器 2, 检查 GVDD (栅极驱动电源) 是否过压
            readWord = DRV8301_readSpi(handle,DRV8301_RegName_Status_2);

            if (readWord & DRV8301_STATUS2_GVDD_OV_BITS) {
                faultType = DRV8301_FaultType_GVDD_OV;
            }
        }
    }

    return(faultType);
}


// ============================================================================
// 函数: DRV8301_getId
// 功能: 读取 DRV8301 的设备 ID
// 参数: handle - DRV8301 驱动对象句柄
// 返回: 设备 ID 值 (低 4bit 有效, 表示芯片版本号)
// 说明:
//   DRV8301 芯片内部固化了一个版本号, 位于状态寄存器 2 的低 4bit。
//   本函数将其读取出来, 方便上层程序识别芯片型号 / 做兼容处理。
// ============================================================================
uint16_t DRV8301_getId(DRV8301_Handle handle) {
    uint16_t data;

    // 读取状态寄存器 2
    data = DRV8301_readSpi(handle,DRV8301_RegName_Status_2);

    // 用 ID 掩码取出 DeviceID 字段
    data &= DRV8301_STATUS2_ID_BITS;

    return(data);
}


// ============================================================================
// 函数: DRV8301_getOcLevel
// 功能: 获取过流保护阈值 (Vds 电压等级)
// 参数: handle - DRV8301 驱动对象句柄
// 返回: VdsLevel_e 枚举, 表示过流触发对应的 Vds 电压阈值 (如 0.25V / 0.5V ...)
// 说明:
//   DRV8301 通过实时监测每个 MOSFET 的 Vds (漏源电压) 来判断是否过流。
//   当 Vds 超过 OC_ADJ_SET 设定的阈值时, 触发过流保护。
//   本函数从控制寄存器 1 中读取 OC_ADJ_SET 字段。
// ============================================================================
DRV8301_VdsLevel_e DRV8301_getOcLevel(DRV8301_Handle handle) {
    uint16_t data;

    // 读取控制寄存器 1
    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

    // 保留 OC_ADJ_SET 位, 清除其他位, 从而单独取出过流阈值
    data &= (~DRV8301_CTRL1_OC_ADJ_SET_BITS);

    return((DRV8301_VdsLevel_e)data);
}


// ============================================================================
// 函数: DRV8301_getOcMode
// 功能: 获取过流保护的响应模式
// 参数: handle - DRV8301 驱动对象句柄
// 返回: OcMode_e 枚举, 例如
//   - CurrentLimit  : 逐周期限流 (到达阈值后关断该周期, 下周期自动重启)
//   - LatchedFault  : 锁存故障 (触发后保持关断, 需软件复位才能恢复)
// 说明: 本函数读取控制寄存器 1 的 OC_MODE 字段
// ============================================================================
DRV8301_OcMode_e DRV8301_getOcMode(DRV8301_Handle handle) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

    // 清除非 OC_MODE 的其他位, 只保留 OC_MODE 字段
    data &= (~DRV8301_CTRL1_OC_MODE_BITS);

    return((DRV8301_OcMode_e)data);
}


// ============================================================================
// 函数: DRV8301_getOcOffTimeMode
// 功能: 获取过流保护关断后自动重启的延时模式
// 参数: handle - DRV8301 驱动对象句柄
// 返回: OcOffTimeMode_e 枚举, 例如
//   - Normal   : 固定死区时间后重启
//   - Extended : 延长关断时间 (抗饱和 / 大感性负载更安全)
// 说明: 本函数读取控制寄存器 2 的 OC_TOFF 字段
// ============================================================================
DRV8301_OcOffTimeMode_e DRV8301_getOcOffTimeMode(DRV8301_Handle handle) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

    data &= (~DRV8301_CTRL2_OC_TOFF_BITS);

    return((DRV8301_OcOffTimeMode_e)data);
}


// ============================================================================
// 函数: DRV8301_getOcTwMode
// 功能: 获取过流 / 过温警告的组合响应模式
// 参数: handle - DRV8301 驱动对象句柄
// 返回: OcTwMode_e 枚举, 指定 OTSD (过温关断) 和 OTW (过温警告) 的行为
// 说明: 本函数读取控制寄存器 2 的 OCTW_SET 字段
// ============================================================================
DRV8301_OcTwMode_e DRV8301_getOcTwMode(DRV8301_Handle handle) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

    data &= (~DRV8301_CTRL2_OCTW_SET_BITS);

    return((DRV8301_OcTwMode_e)data);
}


// ============================================================================
// 函数: DRV8301_getPeakCurrent
// 功能: 获取 DRV8301 栅极驱动峰值电流设置
// 参数: handle - DRV8301 驱动对象句柄
// 返回: PeakCurrent_e 枚举 (如 0.25A / 0.5A / 1.0A 等)
// 说明:
//   栅极驱动电流决定了 MOSFET 的开关速度 (di/dt),
//   设得越大开关越快, 但 EMI 也更严重; 设得太小会导致开关损耗增大。
//   本函数从控制寄存器 1 的 GATE_CURRENT 字段读取当前配置。
// ============================================================================
DRV8301_PeakCurrent_e DRV8301_getPeakCurrent(DRV8301_Handle handle) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

    data &= (~DRV8301_CTRL1_GATE_CURRENT_BITS);

    return((DRV8301_PeakCurrent_e)data);
}


// ============================================================================
// 函数: DRV8301_getPwmMode
// 功能: 获取 DRV8301 的 PWM 输入模式
// 参数: handle - DRV8301 驱动对象句柄
// 返回: PwmMode_e 枚举, 例如
//   - Six_Inputs     : 六输入模式 (每个半桥的高侧/低侧分别由独立 PWM 控制)
//   - Three_Inputs   : 三输入模式 (MCU 只需输出 3 路 PWM, DRV8301 内部自动互补)
//   - Independent    : 独立模式 (PWMn 直接控制对应 NMOS, nPWMn 直接控制 PMOS)
// 说明: 本函数读取控制寄存器 1 的 PWM_MODE 字段
// ============================================================================
DRV8301_PwmMode_e DRV8301_getPwmMode(DRV8301_Handle handle) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

    data &= (~DRV8301_CTRL1_PWM_MODE_BITS);

    return((DRV8301_PwmMode_e)data);
}


// ============================================================================
// 函数: DRV8301_getShuntAmpGain
// 功能: 获取分流放大器 (电流采样运放) 的增益设置
// 参数: handle - DRV8301 驱动对象句柄
// 返回: ShuntAmpGain_e 枚举 (如 10V/V / 20V/V / 40V/V)
// 说明:
//   DRV8301 内置三个 电流采样放大器, 用于放大分流电阻上的小压降,
//   便于 MCU 的 ADC 采样。增益越高, 能分辨的最小电流越小,
//   但动态范围越窄, 容易饱和。需结合分流电阻阻值和 ADC 参考电压选择。
//   本函数读取控制寄存器 2 的 GAIN 字段。
// ============================================================================
DRV8301_ShuntAmpGain_e DRV8301_getShuntAmpGain(DRV8301_Handle handle) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

    data &= (~DRV8301_CTRL2_GAIN_BITS);

    return((DRV8301_ShuntAmpGain_e)data);
}


// ============================================================================
// 函数: DRV8301_init
// 功能: 初始化 DRV8301 驱动对象
// 参数:
//   pMemory  - 指向为 DRV8301_Obj 结构体预分配的内存区域的指针
//   numBytes - 分配的内存大小 (字节), 必须 >= sizeof(DRV8301_Obj)
// 返回: 成功返回 DRV8301_Handle, 内存不足返回 NULL
// 说明:
//   本函数是面向对象的 DRV8301 驱动模型的"构造函数":
//   - 把一块用户提供的内存强转为 DRV8301_Obj, 作为驱动对象
//   - 调用内部的 resetEnableTimeout / resetRxTimeout 把超时标志清零
//   - 之后还需通过 setSpiHandle / setEnGpioHandle / setnCSGpioHandle
//     等函数把底层硬件 (SPI / GPIO) 关联进来才能正常使用
// ============================================================================
DRV8301_Handle DRV8301_init(void *pMemory,const size_t numBytes) {
    DRV8301_Handle handle;

    // 安全检查: 分配的内存必须能容纳一个 DRV8301_Obj
    if (numBytes < sizeof(DRV8301_Obj))
        return((DRV8301_Handle)NULL);

    // 把用户提供的内存强制转换为驱动对象句柄
    handle = (DRV8301_Handle)pMemory;

    // 清零内部超时标志位
    DRV8301_resetRxTimeout(handle);
    DRV8301_resetEnableTimeout(handle);

    return(handle);
}


// ============================================================================
// 函数: DRV8301_setEnGpioHandle
// 功能: 设置 DRV8301 的 EN_GATE 引脚所使用的 GPIO 句柄
// 参数:
//   handle     - DRV8301 驱动对象句柄
//   gpioHandle - STM32 HAL GPIO 对象句柄 (通常是 GPIOA / GPIOB 等)
// 返回: 无
// 说明:
//   DRV8301 需要一个 GPIO 来控制 EN_GATE (使能栅极驱动),
//   本函数把 HAL_GPIO_Init 返回的 GPIOx 指针保存到驱动对象内部,
//   供后续 enable() / disable() 等函数使用。
// ============================================================================
void DRV8301_setEnGpioHandle(DRV8301_Handle handle,GPIO_Handle gpioHandle) {
    DRV8301_Obj *obj = (DRV8301_Obj *)handle;

    obj->EngpioHandle = gpioHandle;

    return;
}


// ============================================================================
// 函数: DRV8301_setEnGpioNumber
// 功能: 设置 DRV8301 的 EN_GATE 引脚编号
// 参数:
//   handle     - DRV8301 驱动对象句柄
//   gpioNumber - 具体的 GPIO 引脚号 (如 GPIO_PIN_10)
// 返回: 无
// 说明:
//   与 setEnGpioHandle 配套使用, 指定在哪个 GPIOx 的哪根引脚上输出 EN_GATE 信号。
// ============================================================================
void DRV8301_setEnGpioNumber(DRV8301_Handle handle,GPIO_Number_e gpioNumber) {
    DRV8301_Obj *obj = (DRV8301_Obj *)handle;

    obj->EngpioNumber = gpioNumber;

    return;
}


// ============================================================================
// 函数: DRV8301_setnCSGpioHandle
// 功能: 设置 DRV8301 SPI 片选 (nCS) 引脚所使用的 GPIO 句柄
// 参数:
//   handle     - DRV8301 驱动对象句柄
//   gpioHandle - GPIOx 指针 (如 GPIOA)
// 返回: 无
// 说明:
//   DRV8301 的 SPI 片选通常由 MCU 的普通 GPIO 模拟 (而不是 SPI 硬件 NSS),
//   以便支持一个 SPI 总线挂多个从设备。本函数保存 GPIO 端口。
// ============================================================================
void DRV8301_setnCSGpioHandle(DRV8301_Handle handle,GPIO_Handle gpioHandle) {
    DRV8301_Obj *obj = (DRV8301_Obj *)handle;

    obj->nCSgpioHandle = gpioHandle;

    return;
}


// ============================================================================
// 函数: DRV8301_setnCSGpioNumber
// 功能: 设置 DRV8301 SPI 片选 (nCS) 的具体引脚号
// 参数:
//   handle     - DRV8301 驱动对象句柄
//   gpioNumber - 如 GPIO_PIN_4
// 返回: 无
// 说明: 与 setnCSGpioHandle 配套使用
// ============================================================================
void DRV8301_setnCSGpioNumber(DRV8301_Handle handle,GPIO_Number_e gpioNumber) {
    DRV8301_Obj *obj = (DRV8301_Obj *)handle;

    obj->nCSgpioNumber = gpioNumber;

    return;
}


// ============================================================================
// 函数: DRV8301_setSpiHandle
// 功能: 设置 DRV8301 使用的 SPI 硬件句柄
// 参数:
//   handle    - DRV8301 驱动对象句柄
//   spiHandle - STM32 HAL SPI 对象句柄 (如 &hspi1)
// 返回: 无
// 说明:
//   所有与 DRV8301 的寄存器通信都走 SPI, 本函数把 HAL 初始化好的 SPI 句柄
//   关联到驱动对象上, 后续 readSpi / writeSpi 内部直接调用 HAL_SPI_Transmit 等。
// ============================================================================
void DRV8301_setSpiHandle(DRV8301_Handle handle,SPI_Handle spiHandle) {
    DRV8301_Obj *obj = (DRV8301_Obj *)handle;

    obj->spiHandle = spiHandle;

    return;
}


// ============================================================================
// 函数: DRV8301_isFault
// 功能: 判断 DRV8301 当前是否处于故障状态
// 参数: handle - DRV8301 驱动对象句柄
// 返回: true = 有故障, false = 正常
// 说明:
//   只检查状态寄存器 1 的 FAULT 总标志位, 不区分具体是哪种故障。
//   若需知道故障来源, 应进一步调用 DRV8301_getFaultType()。
// ============================================================================
bool DRV8301_isFault(DRV8301_Handle handle) {
    DRV8301_Word_t readWord;
    bool status=false;

    // 读取状态寄存器 1
    readWord = DRV8301_readSpi(handle,DRV8301_RegName_Status_1);

    // 只要 FAULT 位非零, 就认为有故障
    if (readWord & DRV8301_STATUS1_FAULT_BITS) {
        status = true;
    }

    return(status);
}


// ============================================================================
// 函数: DRV8301_isReset
// 功能: 判断 DRV8301 是否处于复位状态
// 参数: handle - DRV8301 驱动对象句柄
// 返回: true = 处于复位, false = 正常工作
// 说明:
//   控制寄存器 1 里的 GATE_RESET 位为 1 时, 栅极驱动输出全关,
//   相当于把 DRV8301 置于"软复位"状态。本函数就是读取这个位。
// ============================================================================
bool DRV8301_isReset(DRV8301_Handle handle) {
    DRV8301_Word_t readWord;
    bool status=false;

    readWord = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

    if (readWord & DRV8301_CTRL1_GATE_RESET_BITS) {
        status = true;
    }

    return(status);
}


// ============================================================================
// 函数: DRV8301_readSpi
// 功能: 通过 SPI 从 DRV8301 的指定寄存器读取数据
// 参数:
//   handle   - DRV8301 驱动对象句柄
//   regName  - 要读取的寄存器名 (Status_1 / Status_2 / Control_1 / Control_2)
// 返回: 读取到的寄存器低 10bit 数据 (已通过 DRV8301_DATA_MASK 过滤)
// 说明:
//   DRV8301 的 SPI 读取比较特殊, 无法像普通 SPI 从设备那样在一次
//   传输中同时发送读命令并在 MISO 上得到返回数据。必须分两次事务:
//
//     第 1 次事务:
//       拉低 nCS -> 发送 16bit 读命令 (R/W=1 + 目标寄存器地址) -> 拉高 nCS
//       这一步只是把"读请求"锁存到 DRV8301 内部
//
//     第 2 次事务:
//       再次拉低 nCS -> 发送任意 16bit (这里发全 0, 目的是产生时钟) ->
//       同时在 MISO 上读取 DRV8301 返回的数据 -> 拉高 nCS
//
//   代码中保留了 vTaskDelay 延时, 这是实测得到的经验: 若 SPI 时钟较快,
//   两次事务之间若没有足够间隔, DRV8301 可能还没把返回数据准备好,
//   导致读不到有效值。assert 语句也用来检测 SPI 通信是否真的返回了数据。
// ============================================================================
uint16_t DRV8301_readSpi(DRV8301_Handle handle, const DRV8301_RegName_e regName) {

    // ---- 第一次 SPI 事务: 发送读命令 ----
    // 拉低 nCS, 选中 DRV8301
    HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 构造读控制字: R/W=1, 寄存器地址=regName, 数据字段填 0
    uint16_t zerobuff = 0;
    uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Read, regName, 0);
    uint16_t recbuff = 0xbeef;  // 预置一个不可能的哨兵值, 用来检测是否真的读到了数据
    HAL_SPI_Transmit(handle->spiHandle, (uint8_t*)(&controlword), 1, 1000);

    // DRV8301 数据手册建议不必在两次传输之间再次拉高拉低 nCS (16 个时钟就能提交)
    // 但实测发现: 必须重新拉高再拉低 nCS, 否则第二次事务读不到正确数据
    HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(1));

    // ---- 第二次 SPI 事务: 产生时钟并读取返回数据 ----
    HAL_SPI_TransmitReceive(handle->spiHandle, (uint8_t*)(&zerobuff), (uint8_t*)(&recbuff), 1, 1000);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 拉高 nCS, 结束本次通信
    HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 如果 recbuff 还是初始哨兵值 0xbeef, 说明 SPI 根本没读到从设备响应, 触发断言
    assert(recbuff != 0xbeef);

    // 只返回数据字段 (低 10bit), 去掉寄存器地址等高位
    return(recbuff & DRV8301_DATA_MASK);
}


// ============================================================================
// 函数: DRV8301_reset
// 功能: 触发 DRV8301 的软复位
// 参数: handle - DRV8301 驱动对象句柄
// 返回: 无
// 说明:
//   通过向控制寄存器 1 的 GATE_RESET 位写 1 来实现。
//   软复位会: 关断所有 PWM 输出, 清除锁存的过流/过温故障,
//   把所有寄存器恢复到默认值, 相当于给 DRV8301 重新上电的效果。
// ============================================================================
void DRV8301_reset(DRV8301_Handle handle) {
    uint16_t data;

    // 先读出控制寄存器 1 的当前值 (目的是"读-改-写", 不影响其他位)
    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

    // 置位 GATE_RESET 位, 触发软复位
    data |= DRV8301_CTRL1_GATE_RESET_BITS;

    // 写回寄存器
    DRV8301_writeSpi(handle,DRV8301_RegName_Control_1,data);

    return;
}


// ============================================================================
// 函数: DRV8301_setDcCalMode
// 功能: 设置指定分流放大器的 DC 校准模式
// 参数:
//   handle    - DRV8301 驱动对象句柄
//   ampNumber - 分流放大器编号 (1 或 2)
//   mode      - 目标校准模式 (Load 正常测量 / Cal 短路校准)
// 返回: 无
// 说明:
//   使用"读-改-写"方式: 先读取控制寄存器 2, 清除目标通道的 DC_CAL 位,
//   再将新的 mode 写入, 不影响其他位。
//   这是典型的 SPI 从设备寄存器修改模式, 避免破坏同寄存器中其他配置。
// ============================================================================
void DRV8301_setDcCalMode(DRV8301_Handle handle,const DRV8301_ShuntAmpNumber_e ampNumber,const DRV8301_DcCalMode_e mode) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

    // 清除对应通道的 DC_CAL 位
    if (ampNumber == DRV8301_ShuntAmpNumber_1) {
        data &= (~DRV8301_CTRL2_DC_CAL_1_BITS);

    } else if (ampNumber == DRV8301_ShuntAmpNumber_2) {
        data &= (~DRV8301_CTRL2_DC_CAL_2_BITS);
    }

    // 写入新的校准模式
    data |= mode;

    DRV8301_writeSpi(handle,DRV8301_RegName_Control_2,data);

    return;
}


// ============================================================================
// 函数: DRV8301_setOcLevel
// 功能: 设置过流保护的 Vds 阈值
// 参数:
//   handle   - DRV8301 驱动对象句柄
//   VdsLevel - 目标过流阈值 (如 DRV8301_VdsLevel_0p730_V)
// 返回: 无
// 说明: 操作控制寄存器 1 的 OC_ADJ_SET 字段 (读-改-写)
// ============================================================================
void DRV8301_setOcLevel(DRV8301_Handle handle,const DRV8301_VdsLevel_e VdsLevel) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

    // 先把 OC_ADJ_SET 字段清零
    data &= (~DRV8301_CTRL1_OC_ADJ_SET_BITS);

    // 填入新的过流阈值
    data |= VdsLevel;

    DRV8301_writeSpi(handle,DRV8301_RegName_Control_1,data);

    return;
}


// ============================================================================
// 函数: DRV8301_setOcMode
// 功能: 设置过流保护的响应模式 (限流 / 锁存故障)
// 参数:
//   handle - DRV8301 驱动对象句柄
//   mode   - 目标过流模式
// 返回: 无
// 说明: 操作控制寄存器 1 的 OC_MODE 字段
// ============================================================================
void DRV8301_setOcMode(DRV8301_Handle handle,const DRV8301_OcMode_e mode) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

    data &= (~DRV8301_CTRL1_OC_MODE_BITS);

    data |= mode;

    DRV8301_writeSpi(handle,DRV8301_RegName_Control_1,data);

    return;
}


// ============================================================================
// 函数: DRV8301_setOcOffTimeMode
// 功能: 设置过流保护后的关断时间模式
// 参数:
//   handle - DRV8301 驱动对象句柄
//   mode   - 目标关断时间模式
// 返回: 无
// 说明: 操作控制寄存器 2 的 OC_TOFF 字段
// ============================================================================
void DRV8301_setOcOffTimeMode(DRV8301_Handle handle,const DRV8301_OcOffTimeMode_e mode) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

    data &= (~DRV8301_CTRL2_OC_TOFF_BITS);

    data |= mode;

    DRV8301_writeSpi(handle,DRV8301_RegName_Control_2,data);

    return;
}


// ============================================================================
// 函数: DRV8301_setOcTwMode
// 功能: 设置过流 / 过温保护的组合行为模式
// 参数:
//   handle - DRV8301 驱动对象句柄
//   mode   - 目标 OCP / OTP 组合模式
// 返回: 无
// 说明: 操作控制寄存器 2 的 OCTW_SET 字段
// ============================================================================
void DRV8301_setOcTwMode(DRV8301_Handle handle,const DRV8301_OcTwMode_e mode) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

    data &= (~DRV8301_CTRL2_OCTW_SET_BITS);

    data |= mode;

    DRV8301_writeSpi(handle,DRV8301_RegName_Control_2,data);

    return;
}


// ============================================================================
// 函数: DRV8301_setPeakCurrent
// 功能: 设置栅极驱动峰值电流
// 参数:
//   handle      - DRV8301 驱动对象句柄
//   peakCurrent - 目标峰值电流 (如 DRV8301_PeakCurrent_1p0_A)
// 返回: 无
// 说明: 操作控制寄存器 1 的 GATE_CURRENT 字段
// ============================================================================
void DRV8301_setPeakCurrent(DRV8301_Handle handle,const DRV8301_PeakCurrent_e peakCurrent) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

    data &= (~DRV8301_CTRL1_GATE_CURRENT_BITS);

    data |= peakCurrent;

    DRV8301_writeSpi(handle,DRV8301_RegName_Control_1,data);

    return;
}


// ============================================================================
// 函数: DRV8301_setPwmMode
// 功能: 设置 DRV8301 的 PWM 输入模式
// 参数:
//   handle - DRV8301 驱动对象句柄
//   mode   - 目标 PWM 模式 (六输入 / 三输入 / 独立等)
// 返回: 无
// 说明: 操作控制寄存器 1 的 PWM_MODE 字段
// ============================================================================
void DRV8301_setPwmMode(DRV8301_Handle handle,const DRV8301_PwmMode_e mode) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_1);

    data &= (~DRV8301_CTRL1_PWM_MODE_BITS);

    data |= mode;

    DRV8301_writeSpi(handle,DRV8301_RegName_Control_1,data);

    return;
}


// ============================================================================
// 函数: DRV8301_setShuntAmpGain
// 功能: 设置分流放大器增益
// 参数:
//   handle - DRV8301 驱动对象句柄
//   gain   - 目标增益 (10V/V / 20V/V / 40V/V)
// 返回: 无
// 说明: 操作控制寄存器 2 的 GAIN 字段
// ============================================================================
void DRV8301_setShuntAmpGain(DRV8301_Handle handle,const DRV8301_ShuntAmpGain_e gain) {
    uint16_t data;

    data = DRV8301_readSpi(handle,DRV8301_RegName_Control_2);

    data &= (~DRV8301_CTRL2_GAIN_BITS);

    data |= gain;

    DRV8301_writeSpi(handle,DRV8301_RegName_Control_2,data);

    return;
}


// ============================================================================
// 函数: DRV8301_writeSpi
// 功能: 通过 SPI 向 DRV8301 的指定寄存器写入数据
// 参数:
//   handle  - DRV8301 驱动对象句柄
//   regName - 目标寄存器名
//   data    - 要写入的数据 (低 10bit 有效)
// 返回: 无
// 说明:
//   写操作比读操作简单: 只需要一次 SPI 事务即可。
//     1. 拉低 nCS 选中从设备
//     2. 通过 SPI 发送 16bit 控制字 (R/W=0 + 寄存器地址 + 数据)
//     3. 拉高 nCS, 在上升沿 DRV8301 将数据锁存到目标寄存器
//   同样, nCS 的翻转之间插入了 1ms 延时, 保证 DRV8301 内部时序稳定。
// ============================================================================
void DRV8301_writeSpi(DRV8301_Handle handle, const DRV8301_RegName_e regName,const uint16_t data) {
    // 拉低 nCS, 选中 DRV8301
    HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 构造写控制字: R/W=0, 寄存器地址=regName, 数据=data
    uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Write, regName, data);
    HAL_SPI_Transmit(handle->spiHandle, (uint8_t*)(&controlword), 1, 1000);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 拉高 nCS, DRV8301 在上升沿锁存数据
    HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(1));

    return;
}


// ============================================================================
// 函数: DRV8301_writeData
// 功能: 一次性把应用层配置写入 DRV8301 的两个控制寄存器
// 参数:
//   handle          - DRV8301 驱动对象句柄
//   Spi_8301_Vars   - 包含控制寄存器字段的高层配置结构体
// 返回: 无
// 说明:
//   这是面向上层应用的"批量写"接口, 设计为在后台循环里周期性调用。
//   当 Spi_8301_Vars->SndCmd == true 时, 把结构体 Ctrl_Reg_1 / Ctrl_Reg_2
//   中的各个字段通过位或拼合起来, 然后分别写入 Control_1 / Control_2。
//   写完后自动把 SndCmd 清零。
//
//   相比逐个调用 setPeakCurrent / setOcMode 等函数, 本函数能减少 SPI
//   事务数量 (一次传完所有字段), 适合需要实时配置的场景。
// ============================================================================
void DRV8301_writeData(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars) {
    DRV8301_RegName_e  drvRegName;
    uint16_t drvDataNew;

    if (Spi_8301_Vars->SndCmd) {
        // ---- 写入控制寄存器 1 ----
        drvRegName = DRV8301_RegName_Control_1;
        // 将控制寄存器 1 的各个字段 (栅极电流 / 复位位 / PWM 模式 / 过流模式 / 过流阈值)
        // 通过位或组合成一个 16bit 值
        drvDataNew = Spi_8301_Vars->Ctrl_Reg_1.DRV8301_CURRENT |  \
                     Spi_8301_Vars->Ctrl_Reg_1.DRV8301_RESET   |  \
                     Spi_8301_Vars->Ctrl_Reg_1.PWM_MODE     |  \
                     Spi_8301_Vars->Ctrl_Reg_1.OC_MODE      |  \
                     Spi_8301_Vars->Ctrl_Reg_1.OC_ADJ_SET;
        DRV8301_writeSpi(handle,drvRegName,drvDataNew);

        // ---- 写入控制寄存器 2 ----
        drvRegName = DRV8301_RegName_Control_2;
        // 将控制寄存器 2 的各个字段 (过流过温行为 / 运放增益 / DC 校准 / 关断时间)
        // 组合成一个 16bit 值
        drvDataNew = Spi_8301_Vars->Ctrl_Reg_2.OCTW_SET      |  \
                     Spi_8301_Vars->Ctrl_Reg_2.GAIN          |  \
                     Spi_8301_Vars->Ctrl_Reg_2.DC_CAL_CH1p2  |  \
                     Spi_8301_Vars->Ctrl_Reg_2.OC_TOFF;
        DRV8301_writeSpi(handle,drvRegName,drvDataNew);

        // 清除发送命令标志, 表示本次批量写已经完成
        Spi_8301_Vars->SndCmd = false;
    }

    return;
}


// ============================================================================
// 函数: DRV8301_readData
// 功能: 一次性读取 DRV8301 的所有状态寄存器和控制寄存器, 并填充到结构体
// 参数:
//   handle          - DRV8301 驱动对象句柄
//   Spi_8301_Vars   - 存放读取结果的高层结构体
// 返回: 无
// 说明:
//   这是面向上层应用的"批量读"接口, 设计为在后台循环里周期性调用。
//   当 Spi_8301_Vars->RcvCmd == true 时, 依次读取:
//     1. 状态寄存器 1 -> Stat_Reg_1 各故障标志位 (FAULT / GVDD_UV / PVDD_UV /
//        OTSD / OTW / 三相 FET 的 OC 标志)
//     2. 状态寄存器 2 -> Stat_Reg_2 (GVDD_OV + DeviceID)
//     3. 控制寄存器 1 -> Ctrl_Reg_1 (栅极电流 / 复位 / PWM 模式 / OC 模式 / 阈值)
//     4. 控制寄存器 2 -> Ctrl_Reg_2 (OC/TW 行为 / 增益 / DC 校准 / OC_TOFF)
//   读取时通过掩码把每一位的布尔值或枚举值拆出来, 写入结构体对应字段,
//   方便上层应用直接使用 (如在调试窗口观察或做保护逻辑判断)。
// ============================================================================
void DRV8301_readData(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars) {
    DRV8301_RegName_e  drvRegName;
    uint16_t drvDataNew;

    if (Spi_8301_Vars->RcvCmd) {
        // ---- 读取状态寄存器 1, 解析各故障标志位 ----
        drvRegName = DRV8301_RegName_Status_1;
        drvDataNew = DRV8301_readSpi(handle,drvRegName);
        // FAULT: 总故障标志
        Spi_8301_Vars->Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
        // GVDD_UV: 栅极驱动电源欠压
        Spi_8301_Vars->Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
        // PVDD_UV: 功率级电源欠压
        Spi_8301_Vars->Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
        // OTSD: 过温关断 (严重)
        Spi_8301_Vars->Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
        // OTW: 过温警告 (轻微)
        Spi_8301_Vars->Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
        // FETHA_OC / FETLA_OC / FETHB_OC / FETLB_OC / FETHC_OC / FETLC_OC :
        //   三相 (A/B/C) 的高侧 FET (H) 低侧 FET (L) 是否发生过流
        Spi_8301_Vars->Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
        Spi_8301_Vars->Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
        Spi_8301_Vars->Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
        Spi_8301_Vars->Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
        Spi_8301_Vars->Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
        Spi_8301_Vars->Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);

        // ---- 读取状态寄存器 2 ----
        drvRegName = DRV8301_RegName_Status_2;
        drvDataNew = DRV8301_readSpi(handle,drvRegName);
        // GVDD_OV: 栅极驱动电源过压
        Spi_8301_Vars->Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
        // DeviceID: 芯片版本号
        Spi_8301_Vars->Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);

        // ---- 读取控制寄存器 1 ----
        drvRegName = DRV8301_RegName_Control_1;
        drvDataNew = DRV8301_readSpi(handle,drvRegName);
        // 栅极驱动峰值电流
        Spi_8301_Vars->Ctrl_Reg_1.DRV8301_CURRENT = (DRV8301_PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
        // 软复位位状态
        Spi_8301_Vars->Ctrl_Reg_1.DRV8301_RESET = (DRV8301_Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
        // PWM 输入模式
        Spi_8301_Vars->Ctrl_Reg_1.PWM_MODE = (DRV8301_PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
        // 过流保护模式
        Spi_8301_Vars->Ctrl_Reg_1.OC_MODE = (DRV8301_OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
        // 过流 Vds 阈值
        Spi_8301_Vars->Ctrl_Reg_1.OC_ADJ_SET = (DRV8301_VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);

        // ---- 读取控制寄存器 2 ----
        drvRegName = DRV8301_RegName_Control_2;
        drvDataNew = DRV8301_readSpi(handle,drvRegName);
        // 过流/过温组合行为
        Spi_8301_Vars->Ctrl_Reg_2.OCTW_SET = (DRV8301_OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
        // 分流放大器增益
        Spi_8301_Vars->Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
        // DC 校准模式 (同时包含 Ch1 和 Ch2 两个通道的位)
        Spi_8301_Vars->Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
        // 过流关断时间模式
        Spi_8301_Vars->Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);

        // 清除接收命令标志
        Spi_8301_Vars->RcvCmd = false;
    }

    return;
}


// ============================================================================
// 函数: DRV8301_setupSpi
// 功能: 初始化 DRV8301 的 SPI 通信接口
// 参数:
//   handle          - DRV8301 驱动对象句柄
//   Spi_8301_Vars   - 将要被填充的高层配置结构体
// 返回: 无
// 说明:
//   这是 DRV8301 SPI 接口的初始化函数, 在程序启动阶段调用一次即可。
//   主要工作:
//     1. 清零 SndCmd / RcvCmd 标志
//     2. 延时 1ms 让 DRV8301 稳定
//     3. 主动读取一次所有寄存器并填充 Spi_8301_Vars
//        (相当于让应用层在初始化后就拿到一份完整的寄存器快照)
//
//   原本代码里有一段 #if 0 包裹的硬编码配置 (默认栅极电流 0.25A / 六输入 PWM
//   模式 / Vds 阈值 0.73V 等), 但被屏蔽掉了, 作者注释说"不要在驱动层硬编码
//   默认值, 应该由应用层来配置"。这里保留了原样。
// ============================================================================
void DRV8301_setupSpi(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars) {
    DRV8301_RegName_e  drvRegName;
    uint16_t drvDataNew;

#if 0
    // 以下是被作者注释掉的"硬编码默认配置", 保留在此仅供参考
    drvRegName = DRV8301_RegName_Control_1;
    drvDataNew = (DRV8301_PeakCurrent_0p25_A   | \
                  DRV8301_Reset_Normal         | \
                  DRV8301_PwmMode_Six_Inputs   | \
                  DRV8301_OcMode_CurrentLimit  | \
                  DRV8301_VdsLevel_0p730_V);
    DRV8301_writeSpi(handle,drvRegName,drvDataNew);

    drvRegName = DRV8301_RegName_Control_2;
    drvDataNew = (DRV8301_OcTwMode_Both        | \
                  DRV8301_ShuntAmpGain_10VpV   | \
                  DRV8301_DcCalMode_Ch1_Load   | \
                  DRV8301_DcCalMode_Ch2_Load   | \
                  DRV8301_OcOffTimeMode_Normal);
    DRV8301_writeSpi(handle,drvRegName,drvDataNew);
#endif

    // 清空发送 / 接收命令标志
    Spi_8301_Vars->SndCmd = false;
    Spi_8301_Vars->RcvCmd = false;

    // 等待 DRV8301 内部寄存器稳定
    vTaskDelay(pdMS_TO_TICKS(1));

    // ---- 主动读取一次所有寄存器, 填充初始化快照 ----

    // 状态寄存器 1
    drvRegName = DRV8301_RegName_Status_1;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
    Spi_8301_Vars->Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
    Spi_8301_Vars->Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
    Spi_8301_Vars->Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
    Spi_8301_Vars->Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
    Spi_8301_Vars->Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);

    // 状态寄存器 2
    drvRegName = DRV8301_RegName_Status_2;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
    Spi_8301_Vars->Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);

    // 控制寄存器 1
    drvRegName = DRV8301_RegName_Control_1;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Ctrl_Reg_1.DRV8301_CURRENT = (DRV8301_PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.DRV8301_RESET = (DRV8301_Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.PWM_MODE = (DRV8301_PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.OC_MODE = (DRV8301_OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
    Spi_8301_Vars->Ctrl_Reg_1.OC_ADJ_SET = (DRV8301_VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);

    // 控制寄存器 2
    drvRegName = DRV8301_RegName_Control_2;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Ctrl_Reg_2.OCTW_SET = (DRV8301_OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
    Spi_8301_Vars->Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
    Spi_8301_Vars->Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
    Spi_8301_Vars->Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);

    return;
}

// ======================= 文件结束 =======================