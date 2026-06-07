/**
 * @file terminal.h
 * @brief 简易命令行终端接口
 *
 * 该模块实现了一个简单的命令行解析器，支持：
 * - 按空格分割用户输入的命令行字符串
 * - 内置多种电机控制命令（位置/速度/电流控制等）
 * - 注册最多32个自定义命令回调函数
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include "datatypes.h"

/**
 * @brief 处理用户输入的命令行字符串
 *
 * 该函数接收一行完整的命令字符串，按空格分割为参数数组（argv），
 * 然后根据第一个参数（命令名）查找并执行对应的处理逻辑。
 *
 * 支持的内置命令包括：
 * - p   : 位置控制（position control），需要电机编号、位置设定点、速度前馈、电流前馈
 * - v   : 速度控制（velocity control），需要电机编号、速度设定点、电流前馈
 * - c   : 电流控制（current control），需要电机编号、电流设定点
 * - i   : 显示设备信息（芯片签名、修订号、Flash大小、UUID）
 * - g   : 读取参数值（数字类型索引方式），支持 float/int/bool/uint16
 * - get : 读取参数值（字符类型索引方式），支持 f/i/b/u
 * - h   : 停止所有电机（halt），将所有电机的速度设定点设为0
 * - s   : 设置参数值（数字类型索引方式）
 * - set : 设置参数值（字符类型索引方式）
 * - m   : 配置监控槽位（monitoring slot）
 * - o   : 输出监控数据
 * - t   : 运行反齿槽转矩校准（Anti-Cogging Calibration）
 * - getf: 列出所有浮点型参数及其当前值
 * - geti: 列出所有整型参数及其当前值
 * - getb: 列出所有布尔型参数及其当前值
 * - getu: 列出所有 uint16 类型参数及其当前值
 * - hw_status: 显示硬件状态信息（固件版本、硬件名称、UUID等）
 * - help: 显示帮助信息，列出所有可用命令
 *
 * 如果命令不匹配任何内置命令，则会在已注册的自定义命令回调中查找。
 *
 * @param str 用户输入的命令行字符串（会被 strtok 修改）
 */
void terminal_process_string(char *str);

/**
 * @brief 注册自定义命令回调函数
 *
 * 允许外部模块向终端注册自定义命令。当用户输入的命令与内置命令不匹配时，
 * 系统会遍历所有已注册的回调，查找匹配的命令名并调用对应的回调函数。
 *
 * 最多支持注册 CALLBACK_LEN（32）个自定义命令。
 * 如果同一个命令被重复注册，新的回调将覆盖旧的回调。
 *
 * 示例用法：
 *   void my_cmd_handler(int argc, const char **argv) {
 *       // argc: 参数个数（包括命令名本身）
 *       // argv: 参数数组，argv[0] 为命令名
 *       commands_printf("Hello from my command!");
 *   }
 *
 *   terminal_register_command_callback(
 *       "my_cmd",           // 命令名
 *       "My custom command", // 帮助文本
 *       "[arg1] [arg2]",    // 参数名称说明
 *       my_cmd_handler       // 回调函数指针
 *   );
 *
 * @param command   命令名称（如 "my_cmd"），用户输入该名称时触发回调
 * @param help      帮助文本，在 help 命令中显示，可为 NULL
 * @param arg_names 参数名称说明（如 "[arg1] [arg2]"），在 help 命令中显示，可为 NULL
 * @param cbf       回调函数指针，当命令匹配时被调用
 *                  - argc: 参数个数
 *                  - argv: 参数数组（const char**）
 */
void terminal_register_command_callback(
		const char* command,
		const char *help,
		const char *arg_names,
		void(*cbf)(int argc, const char **argv));

#endif /* TERMINAL_H_ */
