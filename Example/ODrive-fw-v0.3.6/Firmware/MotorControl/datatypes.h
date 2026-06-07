/*
 * datatypes.h - ODrive 电机控制器数据类型定义文件
 *
 * 本文件定义了 ODrive 电机控制系统中使用的核心数据类型，
 * 包括通信协议命令枚举、数据结构等。
 * 主要用于电机控制器与上位机或其他设备之间的通信。
 *
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>


//#define ARM_ENCRYPT
#if defined ARM_ENCRYPT
#else
#endif

#if defined ARM_ENCRYPT
#define ENCRYPT_BUFFER_LEN 13
#else
#endif

// 数据类型定义


// 通信命令枚举 - 定义上位机与电机控制器之间的通信协议命令
typedef enum {
    COMM_FW_VERSION = 0,        // 获取固件版本信息
    COMM_JUMP_TO_BOOTLOADER,    // 跳转到 Bootloader 程序（用于固件升级）
    COMM_ERASE_NEW_APP,         // 擦除新的应用程序区域（准备固件升级）
    COMM_WRITE_NEW_APP_DATA,    // 写入新的应用程序数据（固件升级过程）
    COMM_GET_VALUES,            // 获取电机运行状态数据（转速、电流、温度等）
    COMM_SET_DUTY,              // 设置 PWM 占空比（开环控制）
    COMM_SET_CURRENT,           // 设置电机电流（闭环电流控制）
    COMM_SET_CURRENT_BRAKE,     // 设置电机电流（制动模式）
    COMM_SET_RPM,               // 设置电机转速（速度闭环控制）
    COMM_SET_POS,               // 设置电机位置（位置闭环控制）
    COMM_SET_HANDBRAKE,         // 设置手刹模式（保持当前位置）
    COMM_SET_DETECT,            // 设置电机检测参数
    COMM_SET_SERVO_POS,         // 设置舵机位置
    COMM_SET_MCCONF,            // 设置电机配置参数
    COMM_GET_MCCONF,            // 获取电机配置参数
    COMM_GET_MCCONF_DEFAULT,    // 获取电机配置的默认值
    COMM_SET_APPCONF,           // 设置应用程序配置参数
    COMM_GET_APPCONF,           // 获取应用程序配置参数
    COMM_GET_APPCONF_DEFAULT,   // 获取应用程序配置的默认值
    COMM_SAMPLE_PRINT,          // 采样数据打印（用于调试）
    COMM_TERMINAL_CMD,          // 终端命令执行
    COMM_PRINT,                 // 打印调试信息
    COMM_ROTOR_POSITION,        // 获取转子位置信息
    COMM_EXPERIMENT_SAMPLE,     // 实验采样数据
    COMM_DETECT_MOTOR_PARAM,    // 电机参数检测（电阻、电感、反电动势常数等）
    COMM_DETECT_MOTOR_R_L,      // 检测电机电阻和电感
    COMM_DETECT_MOTOR_FLUX_LINKAGE,  // 检测电机磁链
    COMM_DETECT_ENCODER,        // 编码器检测与校准
    COMM_DETECT_HALL_FOC,       // FOC 霍尔传感器检测
    COMM_REBOOT,                // 重启控制器
    COMM_ALIVE,                 // 心跳包（保持通信连接）
    COMM_GET_DECODED_PPM,       // 获取解码后的 PPM 信号值
    COMM_GET_DECODED_ADC,       // 获取解码后的 ADC 信号值
    COMM_GET_DECODED_CHUK,      // 获取解码后的 Chuk 控制器数据
    COMM_FORWARD_CAN,           // CAN 总线数据转发
    COMM_SET_CHUCK_DATA,        // 设置 Chuk 控制器数据
    COMM_CUSTOM_APP_DATA,       // 自定义应用程序数据
    COMM_NRF_START_PAIRING      // 开始 NRF 无线模块配对
} COMM_PACKET_ID;

#endif /* DATATYPES_H_ */
