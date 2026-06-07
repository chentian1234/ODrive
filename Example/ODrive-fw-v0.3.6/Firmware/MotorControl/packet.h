/**
 * @file packet.h
 * @brief 轻量级串行通信数据包协议头文件
 *
 * 帧格式说明：
 * 本协议采用定界符+长度+数据+CRC+结束符的帧格式，支持短帧和长帧两种模式：
 *
 * 短帧（数据长度 <= 256 字节）：
 *   [0x02][1字节长度][数据][CRC16高][CRC16低][0x03]
 *   - 起始符: 0x02，表示短帧模式，长度字段占1字节
 *   - 长度: 1字节，表示有效载荷数据的长度
 *   - 数据: 0~256字节的实际有效载荷
 *   - CRC16: 2字节，CRC16校验值（高字节在前，低字节在后）
 *   - 结束符: 0x03
 *
 * 长帧（数据长度 > 256 字节）：
 *   [0x03][2字节长度][数据][CRC16高][CRC16低][0x03]
 *   - 起始符: 0x03，表示长帧模式，长度字段占2字节
 *   - 长度: 2字节（大端序），表示有效载荷数据的长度
 *   - 数据: 257~1024字节的实际有效载荷
 *   - CRC16: 2字节，CRC16校验值（高字节在前，低字节在后）
 *   - 结束符: 0x03
 *
 * 注意：短帧和长帧使用不同的起始符（0x02 vs 0x03），但结束符统一为0x03。
 */

#ifndef PACKET_H_
#define PACKET_H_

#include <stdint.h>

/*================ 协议配置参数 ================*/

/**
 * @brief 接收超时时间（单位：毫秒）
 * 当连续 PACKET_RX_TIMEOUT 毫秒未收到新字节时，接收状态机将复位到初始状态，
 * 丢弃当前不完整的帧数据，防止因通信中断导致状态机卡死。
 */
#define PACKET_RX_TIMEOUT		1000

/**
 * @brief 支持的协议处理器（Handler）数量
 * 每个处理器维护独立的接收状态和回调函数，允许多个通信通道复用同一协议栈。
 */
#define PACKET_HANDLERS			2

/**
 * @brief 最大有效载荷长度（单位：字节）
 * 定义单帧数据部分的最大允许长度，超出此长度的帧将被拒绝接收。
 * 同时影响接收/发送缓冲区的大小分配。
 */
#define PACKET_MAX_PL_LEN		1024

/*================ 函数声明 ================*/

/**
 * @brief 初始化指定的数据包处理器
 * @param s_func 发送回调函数指针，组帧完成后通过此函数将数据发出
 *               函数原型: void send_func(unsigned char *data, unsigned int len)
 * @param p_func 接收处理回调函数指针，完整且校验通过的数据包通过此函数递交
 *               函数原型: void process_func(unsigned char *data, unsigned int len)
 * @param handler_num 处理器编号（0 ~ PACKET_HANDLERS-1）
 */
void packet_init(void (*s_func)(unsigned char *data, unsigned int len),
		void (*p_func)(unsigned char *data, unsigned int len), int handler_num);

/**
 * @brief 逐字节处理接收到的串行数据
 * @param rx_data 当前接收到的字节
 * @param handler_num 处理器编号（0 ~ PACKET_HANDLERS-1）
 *
 * 内部维护一个7状态（0~6）的接收状态机，每个字节驱动状态机前进或复位。
 * 应在每次UART/SPI等接收到数据时调用此函数。
 */
void packet_process_byte(uint8_t rx_data, int handler_num);

/**
 * @brief 周期调用的超时检查函数
 *
 * 遍历所有处理器，递减接收超时计数器。当计数器归零时，
 * 将对应处理器的接收状态机复位到初始状态（state 0），丢弃残缺帧。
 * 建议每1毫秒调用一次，与 PACKET_RX_TIMEOUT 的单位保持一致。
 */
void packet_timerfunc(void);

/**
 * @brief 组装并发送一个数据包
 * @param data 指向有效载荷数据的指针
 * @param len 有效载荷数据的长度（不能超过 PACKET_MAX_PL_LEN）
 * @param handler_num 处理器编号（0 ~ PACKET_HANDLERS-1）
 *
 * 组帧过程：
 *   1. 根据数据长度选择短帧（0x02起始）或长帧（0x03起始）模式
 *   2. 写入长度字段（短帧1字节，长帧2字节大端序）
 *   3. 拷贝有效载荷数据
 *   4. 计算CRC16校验值并写入（高字节在前）
 *   5. 写入结束符 0x03
 *   6. 通过初始化时注册的 send_func 回调发出完整帧
 */
void packet_send_packet(unsigned char *data, unsigned int len, int handler_num);

#endif /* PACKET_H_ */
