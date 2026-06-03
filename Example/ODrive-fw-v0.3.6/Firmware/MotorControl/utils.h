
#ifndef __UTILS_H
#define __UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================== */
/*                         STM32 芯片唯一标识寄存器地址                        */
/* ========================================================================== */

/**
 * @brief 唯一ID寄存器地址（96位UUID的起始地址）
 * @details STM32F4系列芯片内置的96位唯一设备标识符，每个芯片出厂时唯一，
 *          可用于序列号、加密授权等场景。地址0x1FFF7A10为STM32F4系列标准地址。
 */
#define ID_UNIQUE_ADDRESS (0x1FFF7A10)

/**
 * @brief Flash容量寄存器地址
 * @details 存储芯片内置Flash容量的寄存器，读取值为以KB为单位的Flash大小。
 *          例如：读取值为1024表示芯片有1MB的Flash。
 */
#define ID_FLASH_ADDRESS (0x1FFF7A22)

/**
 * @brief 设备ID寄存器地址（DBGMCU_IDCODE）
 * @details 调试MCU单元中的IDCODE寄存器，包含设备标识符（DEV_ID，低12位）
 *          和芯片版本号（REV_ID，高16位）。用于运行时识别芯片型号和版本。
 */
#define ID_DBGMCU_IDCODE (0xE0042000)

/**
 * @brief 获取STM32设备标识符（Device ID）
 * @details 读取DBGMCU_IDCODE寄存器的低16位，并掩码保留低12位（DEV_ID字段）。
 *          该标识符用于区分不同的STM32芯片型号。
 * @return 设备ID值（16位，但仅低12位有效，高4位始终为0）
 * @retval 0x0413 STM32F405xx/07xx 和 STM32F415xx/17xx
 * @retval 0x0419 STM32F42xxx 和 STM32F43xxx
 * @retval 0x0423 STM32F401xB/C
 * @retval 0x0433 STM32F401xD/E
 * @retval 0x0431 STM32F411xC/E
 * @note 定义为宏，直接通过指针访问内存地址读取硬件寄存器值
 */
#define STM_ID_GetSignature() ((*(uint16_t *)(ID_DBGMCU_IDCODE)) & 0x0FFF)

/**
 * @brief 获取STM32芯片版本号（Revision ID）
 * @details 读取DBGMCU_IDCODE寄存器偏移+2处的16位值，即REV_ID字段。
 *          用于识别芯片的硅版本/修订版本，不同修订版可能存在硬件差异或Errata。
 * @return 芯片版本号（16位）
 * @retval 0x1000 Revision A
 * @retval 0x1001 Revision Z
 * @retval 0x1003 Revision Y
 * @retval 0x1007 Revision 1
 * @retval 0x2001 Revision 3
 */
#define STM_ID_GetRevision() (*(uint16_t *)(ID_DBGMCU_IDCODE + 2))

/**
 * @brief 获取STM32芯片内置Flash容量
 * @details 直接读取Flash容量寄存器的16位值，返回值为Flash大小（单位：KB）。
 *          例如返回1024表示芯片有1024KB（即1MB）的Flash。
 * @return Flash容量，单位为KB（千字节）
 */
#define STM_ID_GetFlashSize() (*(uint16_t *)(ID_FLASH_ADDRESS))

/**
 * @brief 获取96位唯一UUID的指定32位段
 * @details STM32的UUID共96位（12字节），分为3个32位段。
 *          通过参数x选择要读取的段（0、1或2）。
 * @param[in] x 段索引，取值范围0~2，对应96位UUID的三个32位部分
 * @return 指定段的32位UUID值；如果x不在0~2范围内则返回0
 * @note 96位UUID在芯片出厂时烧录，全球唯一，可用于设备识别和防克隆
 */
#define STM_ID_GetUUID(x) ((x >= 0 && x < 3) ? (*(uint32_t *)(ID_UNIQUE_ADDRESS + 4 * (x))) : 0)

/* ========================================================================== */
/*                              数学常量与宏定义                               */
/* ========================================================================== */

/**
 * @brief 圆周率常量π
 * @details 如果系统中已经定义了M_PI则先取消定义，再重新定义为float精度的π值。
 *          使用float而非double以匹配嵌入式系统常用的单精度浮点运算。
 */
#ifdef M_PI
#undef M_PI
#endif
#define M_PI 3.14159265358979323846f

/**
 * @brief 取最大值宏
 * @details 安全地返回两个值中的较大者。使用括号包裹参数和整个表达式，
 *          避免宏展开时的运算符优先级问题。
 * @param x 第一个比较值
 * @param y 第二个比较值
 * @return 较大的值
 */
#define MACRO_MAX(x, y) (((x) > (y)) ? (x) : (y))

/**
 * @brief 取最小值宏
 * @details 安全地返回两个值中的较小者。使用括号包裹参数和整个表达式，
 *          避免宏展开时的运算符优先级问题。
 * @param x 第一个比较值
 * @param y 第二个比较值
 * @return 较小的值
 */
#define MACRO_MIN(x, y) (((x) < (y)) ? (x) : (y))

/* ========================================================================== */
/*                              函数声明                                       */
/* ========================================================================== */

/**
 * @brief 空间矢量调制（Space Vector Modulation, SVM）算法
 * @details 将两相静止坐标系（α-β坐标系）下的电压矢量转换为三相PWM占空比。
 *          SVM是FOC（磁场定向控制）中逆变器调制的关键环节，相比传统SPWM，
 *          SVM能将直流母线电压利用率提高约15%，且输出谐波更小。
 *
 *          算法原理：
 *          1. 三相逆变器有6个有效开关状态（V1~V6）和2个零状态（V0, V7），
 *             在α-β平面上构成正六边形的6个扇区
 *          2. 根据α-β矢量的角度确定其所在的扇区（sextant）
 *          3. 在该扇区内，用相邻两个有效矢量和零矢量合成目标矢量
 *          4. 通过伏秒平衡原理计算各矢量的作用时间，进而得到三相PWM占空比
 *
 *          判断扇区的边界条件使用 tan(30°) = 1/√3 ≈ 0.577 作为斜率阈值，
 *          将α-β平面等分为6个60°扇区。
 *
 * @param[in]  alpha α轴分量（Clarke变换后的静止坐标系分量）
 * @param[in]  beta  β轴分量（Clarke变换后的静止坐标系分量）
 * @param[out] tA    A相PWM占空比（0.0~1.0，表示高电平时间占比）
 * @param[out] tB    B相PWM占空比（0.0~1.0）
 * @param[out] tC    C相PWM占空比（0.0~1.0）
 * @return 状态码
 * @retval  0  计算成功
 * @retval -1  输入超出范围（α-β矢量幅值超过√3/2，无法在该扇区内合成）
 * @note α-β矢量的幅值不能超过√3/2 ≈ 0.866，这是SVM的线性调制区限制，
 *       超过此值将进入过调制区，输出波形会失真
 */
int SVM(float alpha, float beta, float* tA, float* tB, float* tC);

/**
 * @brief 将角度归一化到[-π, π)区间
 * @details 通过不断加减2π，将任意角度值约束到[-π, π)范围内。
 *          常用于电机FOC控制中电角度的归一化处理。
 * @param[in] theta 输入角度（弧度制）
 * @return 归一化后的角度，范围[-π, π)
 * @warning 对于非常大的角度值可能需要多次循环，影响性能。
 *          建议传入的角度不要偏离[-π, π)太远。
 */
float wrap_pm_pi(float theta);

/**
 * @brief 快速近似atan2函数（二维反正切函数）
 * @details 使用多项式近似法快速计算atan2(y, x)，相比标准库的atan2函数
 *          速度快约3-5倍，最大误差约0.07弧度（约4度），适用于对精度要求不高
 *          但对实时性要求高的电机控制场景。
 *
 *          算法原理：
 *          1. 利用atan2的对称性，将问题归约到第一卦限（|y|<=|x|, x>0）
 *          2. 令 a = min(|x|,|y|) / max(|x|,|y|)，则a的范围为[0, 1]
 *          3. 使用三次多项式近似: r ≈ a + s·(-0.327622764 + s·(0.15931422 + s·(-0.0464964749)))
 *             其中 s = a²，该多项式在[0,1]区间内逼近arctan(a)
 *          4. 根据原始象限对结果进行对称变换
 *
 *          多项式系数的选择基于最小化最大误差原则（Minimax近似），
 *          在a∈[0,1]范围内最大绝对误差小于0.0015弧度。
 *
 * @param[in] y y坐标值（对边）
 * @param[in] x x坐标值（邻边）
 * @return 近似角度值，范围(-π, π]
 */
float fast_atan2(float y, float x);

/**
 * @brief 正模运算（数学意义上的取模，非C语言的取余）
 * @details C语言中%运算符对负数的行为是"向零取整的取余"，
 *          而数学上的模运算结果始终与除数同号（为正）。
 *          例如：mod(-1, 5) = 4，而 -1 % 5 = -1。
 * @param[in] dividend 被除数
 * @param[in] divisor  除数
 * @return 正模运算结果，始终在[0, |divisor|)范围内
 */
int mod(int dividend, int divisor);

/**
 * @brief 将截止时间（deadline）转换为超时时间（timeout）
 * @details 截止时间是某个未来的绝对时间点（毫秒），超时时间是相对当前剩余的毫秒数。
 *          如果截止时间已过，返回0。使用符号位检测判断是否超时，
 *          可正确处理系统时钟溢出（uint32_t回绕）的情况。
 * @param[in] deadline_ms 截止时间（以毫秒为单位的绝对时间点）
 * @return 剩余超时时间（毫秒）；如果截止时间已过则返回0
 */
uint32_t deadline_to_timeout(uint32_t deadline_ms);

/**
 * @brief 将超时时间转换为截止时间
 * @details 根据当前时间和给定的超时时长，计算出未来的截止时间点。
 *          用于设置一个需要在未来某个时间点触发的事件。
 * @param[in] timeout_ms 超时时间（毫秒）
 * @return 截止时间（毫秒），即 now_ms + timeout_ms
 */
uint32_t timeout_to_deadline(uint32_t timeout_ms);

/**
 * @brief 获取系统启动以来的微秒数
 * @details 结合HAL_GetTick()（毫秒级系统滴答）和定时器计数器（微秒级），
 *          提供高精度的时间戳。使用do-while循环确保在ms进位时读取到一致的
 *          tick和counter值，避免竞争条件。
 * @return 系统启动以来的微秒数（32位，约71分钟后溢出）
 */
uint32_t micros(void);

/**
 * @brief 微秒级精确延时
 * @details 使用忙等待（busy-wait）方式实现微秒级延时。
 *          通过循环比较当前微秒时间戳与起始时间的差值来实现精确延时。
 *          延时期间CPU持续运行，不进入低功耗模式。
 * @param[in] us 需要延时的微秒数
 * @note 忙等待会占用CPU资源，仅适用于短时间延时。
 *       长时间延时建议使用定时器中断或RTOS的延迟函数。
 */
void delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif  //__UTILS_H
