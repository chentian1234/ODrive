/**
 * @file crc.h
 * @brief CRC16校验头文件
 *
 * 提供CRC16-CCITT校验值的计算功能，用于数据包完整性校验。
 */

#ifndef CRC_H_
#define CRC_H_

/**
 * 函数声明
 */

/**
 * @brief 计算CRC16-CCITT校验值
 *
 * 使用查表法快速计算给定数据缓冲区的CRC16-CCITT校验和。
 * CRC16-CCITT是一种常用的循环冗余校验算法，广泛应用于通信协议
 * 中检测数据传输错误。
 *
 * @param buf 指向待校验数据缓冲区的指针
 * @param len 数据缓冲区的长度(字节数)
 * @return 计算得到的16位CRC校验值
 *
 * @note 生成多项式: x^16 + x^12 + x^5 + 1 (0x1021)
 * @note 应用场景: 通信帧校验、Flash数据完整性验证等
 */
unsigned short crc16(unsigned char *buf, unsigned int len);

#endif /* CRC_H_ */
