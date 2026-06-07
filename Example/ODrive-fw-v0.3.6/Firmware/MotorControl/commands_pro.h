/**
 * @file commands_pro.h
 * @brief ODrive 高级二进制命令协议头文件
 * 
 * 本文件定义了 ODrive 电机控制器的高级二进制通信协议。
 * 相比简单的文本命令(如 "v 0 1000")，二进制协议具有更高的传输效率
 * 和更低的通信开销，适用于实时性要求较高的应用场景。
 * 
 * 协议基于 VESC 二进制通信协议扩展，采用帧头+长度+负载+校验的封装格式，
 * 支持固件版本查询、电机状态获取、运动控制(占空比/电流/速度/位置)等功能。
 */

#ifndef COMM_H_
#define COMM_H_

#include "datatypes.h"

/**
 * @defgroup 固件版本与编译信息 固件版本与编译信息宏定义
 * @{
 */

/** @brief 固件主版本号 */
#define FW_VERSION_MAJOR		3
/** @brief 固件次版本号 */
#define FW_VERSION_MINOR		102

/** @brief 硬件板型名称标识 */
#define HW_NAME					"34b"

/**
 * @brief 编译工具链标识
 * 根据预定义宏自动选择当前使用的编译器类型
 */
#if defined ARM_MDK
#define __BUILD__					" MDK "     /**< ARM MDK-ARM (Keil) 编译器 */
#elif defined ARM_IAR
#define __BUILD__					" IAR "     /**< IAR EWARM 编译器 */
#else
#define __BUILD__					" GCC "     /**< GNU ARM GCC 编译器 */
#endif

/** @brief 应用程序类型标识: 新版本应用固件 */
#define ARM_APP_NEW

/**
 * @brief 目标项目标识
 * 根据编译宏区分应用固件、BootLoader 或无目标
 */
#if defined ARM_APP_NEW
#define __FOR__					"ODrive-fw-v0.3.6"  /**< 应用固件版本 */
#elif defined ARM_BOOT_NEW
#define __FOR__					"boot1.0"           /**< BootLoader 版本 */
#else
#define __FOR__					"none1.0"           /**< 无目标 */
#endif

/** @brief 编译信息分隔符 */
#define __BY__   " by "
/** @brief 编译信息分隔符 */
#define __AT__   " at "

/** @} */

/**
 * @defgroup STM32芯片唯一ID STM32芯片唯一ID地址定义
 * @{
 */

/**
 * @brief STM32 芯片 96 位唯一 ID 地址 (32位指针)
 * 
 * 每颗 STM32 芯片在出厂时都烧录了一个唯一的 96 位(12字节)ID，
 * 可用于设备识别、固件加密、授权验证等场景。
 * 该 ID 位于 STM32 的系统存储器区域，只读不可修改。
 * 地址: 0x1FFF7A10 (适用于 STM32F405/407 系列)
 * 
 * 使用示例:
 * @code
 *   uint32_t id1 = STM32_UUID[0];
 *   uint32_t id2 = STM32_UUID[1];
 *   uint32_t id3 = STM32_UUID[2];
 * @endcode
 */
#define STM32_UUID					((uint32_t*)0x1FFF7A10)

/**
 * @brief STM32 芯片 96 位唯一 ID 地址 (8位指针)
 * 
 * 以字节数组形式访问芯片唯一 ID，共 12 字节。
 * 适用于需要逐字节读取 ID 的场景(如序列号打印、MAC地址生成等)。
 * 
 * 使用示例:
 * @code
 *   for (int i = 0; i < 12; i++) {
 *       printf("%02X", STM32_UUID_8[i]);
 *   }
 * @endcode
 */
#define STM32_UUID_8				((uint8_t*)0x1FFF7A10)

/** @} */

/**
 * @defgroup 函数接口 高级二进制命令协议函数接口
 * @{
 */

/**
 * @brief 设置数据包发送回调函数
 * 
 * 该函数用于注册底层数据发送回调，当协议需要向外发送数据时
 * 会调用此回调函数。调用者需要提供实际的硬件发送实现(如 UART/USB/CAN 发送)。
 * 
 * @param func 指向数据包发送函数的指针
 *             - data: 待发送的数据缓冲区指针
 *             - len:  待发送数据的长度(字节)
 * 
 * 使用示例:
 * @code
 *   void uart_send(unsigned char *data, unsigned int len) {
 *       // 实际的 UART 发送实现
 *       HAL_UART_Transmit(&huart1, data, len, HAL_MAX_DELAY);
 *   }
 *   commands_set_send_func(uart_send);
 * @endcode
 */
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len));

/**
 * @brief 发送二进制数据包
 * 
 * 通过已注册的发送回调函数将数据发送出去。
 * 该函数会检查发送回调是否已注册，若未注册则不执行任何操作。
 * 
 * @param data 待发送的数据缓冲区指针
 * @param len  待发送数据的长度(字节)
 * 
 * @note 调用此函数前必须先调用 commands_set_send_func() 注册发送回调
 */
void commands_send_packet(unsigned char *data, unsigned int len);

/**
 * @brief 处理接收到的二进制命令数据包
 * 
 * 该函数是二进制命令协议的核心处理入口，负责解析接收到的数据帧，
 * 根据命令类型(COMM_PACKET_ID)执行相应的操作，并通过回调函数返回响应数据。
 * 
 * 支持的主要命令类型:
 * - COMM_FW_VERSION:        查询固件版本和芯片信息
 * - COMM_GET_VALUES:        获取电机运行状态(电压/电流/速度/位置等)
 * - COMM_SET_DUTY:          设置电机目标选择(0=M0, 1=M1)
 * - COMM_SET_CURRENT:       设置电机电流给定值
 * - COMM_SET_CURRENT_BRAKE: 设置电机控制模式切换
 * - COMM_SET_RPM:           设置电机速度给定值
 * - COMM_SET_POS:           设置电机位置给定值
 * - COMM_SET_HANDBRAKE:     设置手刹(停止所有电机)
 * - COMM_TERMINAL_CMD:      执行终端文本命令
 * - COMM_REBOOT:            重启控制器
 * 
 * @param data 接收到的数据缓冲区指针(包含命令ID和参数)
 * @param len  接收到的数据长度(字节)
 * 
 * @note 数据帧格式: [命令ID(1字节)] [参数数据(N字节)]
 */
void commands_process_packet(unsigned char *data, unsigned int len);

/**
 * @brief 格式化打印调试信息并发送
 * 
 * 类似 printf 的功能，将格式化字符串通过二进制协议发送出去。
 * 主要用于调试信息的远程输出，上位机接收后可显示在终端中。
 * 
 * @param format 格式化字符串(与 printf 格式相同)
 * @param ...    可变参数列表
 * 
 * 使用示例:
 * @code
 *   commands_printf("Motor current: %.2f A\n", current);
 *   commands_printf("Error: %d\n", error_code);
 * @endcode
 */
void commands_printf(const char* format, ...);

/** @} */

#endif /* COMM_H_ */
