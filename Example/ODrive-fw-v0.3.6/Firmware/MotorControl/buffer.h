/**
 * @file buffer.h
 * @brief 数据序列化/反序列化工具
 * 
 * 用于将各种数据类型（整型、浮点）序列化为字节流，或从字节流中反序列化。
 * 常用于通信协议的数据打包与解包。
 * 
 * 【字节序说明】
 * 所有多字节数据均采用大端序(Big-Endian)排列：
 *   - 高字节存放在低地址，低字节存放在高地址
 *   - 例如 uint32_t 0x12345678 序列化后为: buffer[0]=0x12, buffer[1]=0x34, buffer[2]=0x56, buffer[3]=0x78
 * 
 * 【浮点数定点编码说明】
 * float16/float32 采用定点数编码方式：
 *   - 序列化: 浮点数 × scale 缩放因子 → 取整 → 序列化为整数
 *   - 反序列化: 读取整数 → 除以 scale 缩放因子 → 还原浮点数
 *   - 优点: 节省传输字节数，适合精度要求不高的场景
 *   - 注意: 存在精度损失，scale 越大精度越高，但可表示范围越小
 * 
 * float32_auto 自动精度浮点编码：
 *   - 将浮点数按 IEEE 754 格式手动编码为 32 位整数
 *   - 自动适应数值大小，无需指定 scale 参数
 *   - 精度与原始浮点数一致，适合需要精确传输浮点数的场景
 */

#ifndef BUFFER_H_
#define BUFFER_H_

#include <stdint.h>

/* ==================== 序列化函数（将数据追加到缓冲区） ==================== */
/**
 * @brief 将 int16_t 有符号16位整数以大端序追加到缓冲区
 * @param buffer 目标字节缓冲区
 * @param number 要写入的16位整数
 * @param index 缓冲区当前写入位置的索引指针，写入后自动递增
 */
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
/**
 * @brief 将 uint16_t 无符号16位整数以大端序追加到缓冲区
 * @param buffer 目标字节缓冲区
 * @param number 要写入的16位整数
 * @param index 缓冲区当前写入位置的索引指针，写入后自动递增
 */
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
/**
 * @brief 将 int32_t 有符号32位整数以大端序追加到缓冲区
 * @param buffer 目标字节缓冲区
 * @param number 要写入的32位整数
 * @param index 缓冲区当前写入位置的索引指针，写入后自动递增
 */
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
/**
 * @brief 将 uint32_t 无符号32位整数以大端序追加到缓冲区
 * @param buffer 目标字节缓冲区
 * @param number 要写入的32位整数
 * @param index 缓冲区当前写入位置的索引指针，写入后自动递增
 */
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);

/* ==================== 浮点数定点编码序列化 ==================== */
/**
 * @brief 将 float 浮点数以定点数格式序列化（16位编码）
 * 
 * 编码原理：将浮点数乘以 scale 缩放因子后取整，作为 int16_t 存入缓冲区
 *   - 示例：number=3.14, scale=1000 → 存储 3140 (占用2字节)
 *   - 精度：1/scale，本例精度为 0.001
 *   - 范围：约 -32.768 ~ 32.767 (当 scale=1000 时)
 * 
 * @param buffer 目标字节缓冲区
 * @param number 要写入的浮点数
 * @param scale 缩放因子（如1000表示保留3位小数精度）
 * @param index 缓冲区当前写入位置的索引指针，写入后自动递增
 */
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index);
/**
 * @brief 将 float 浮点数以定点数格式序列化（32位编码）
 * 
 * 编码原理：将浮点数乘以 scale 缩放因子后取整，作为 int32_t 存入缓冲区
 *   - 示例：number=3.14159265, scale=1000000 → 存储 3141592 (占用4字节)
 *   - 精度：1/scale，本例精度为 0.000001
 *   - 范围：约 -2147.48 ~ 2147.48 (当 scale=1000000 时)
 * 
 * @param buffer 目标字节缓冲区
 * @param number 要写入的浮点数
 * @param scale 缩放因子
 * @param index 缓冲区当前写入位置的索引指针，写入后自动递增
 */
void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);
/**
 * @brief 将 float 浮点数以 IEEE 754 兼容格式自动序列化（32位）
 * 
 * 自动精度编码原理：
 *   1. 使用 frexpf() 将浮点数分解为尾数(sig)和指数(e)
 *   2. 将尾数映射到 23 位有效位（与 IEEE 754 single precision 一致）
 *   3. 将指数和符号位打包成 32 位整数
 *   4. 以大端序写入缓冲区
 * 
 * 特点：
 *   - 无需指定 scale，自动适应数值大小
 *   - 精度与原始 float 一致（约6~7位有效数字）
 *   - 可表示极大/极小数值
 *   - 传输的是手动编码的整数，保证跨平台一致性
 * 
 * @param buffer 目标字节缓冲区
 * @param number 要写入的浮点数
 * @param index 缓冲区当前写入位置的索引指针，写入后自动递增
 */
void buffer_append_float32_auto(uint8_t* buffer, float number, int32_t *index);

/* ==================== 反序列化函数（从缓冲区读取数据） ==================== */
/**
 * @brief 从缓冲区以大端序读取 int16_t 有符号16位整数
 * @param buffer 源字节缓冲区
 * @param index 缓冲区当前读取位置的索引指针，读取后自动递增
 * @return 读取到的16位整数
 */
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
/**
 * @brief 从缓冲区以大端序读取 uint16_t 无符号16位整数
 * @param buffer 源字节缓冲区
 * @param index 缓冲区当前读取位置的索引指针，读取后自动递增
 * @return 读取到的16位整数
 */
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
/**
 * @brief 从缓冲区以大端序读取 int32_t 有符号32位整数
 * @param buffer 源字节缓冲区
 * @param index 缓冲区当前读取位置的索引指针，读取后自动递增
 * @return 读取到的32位整数
 */
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
/**
 * @brief 从缓冲区以大端序读取 uint32_t 无符号32位整数
 * @param buffer 源字节缓冲区
 * @param index 缓冲区当前读取位置的索引指针，读取后自动递增
 * @return 读取到的32位整数
 */
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);

/* ==================== 浮点数定点编码反序列化 ==================== */
/**
 * @brief 从缓冲区读取定点数编码的浮点数（16位）
 * 
 * 解码原理：读取 int16_t 后除以 scale 缩放因子还原浮点数
 *   - 与 buffer_append_float16 配对使用
 * 
 * @param buffer 源字节缓冲区
 * @param scale 缩放因子（需与序列化时使用的 scale 一致）
 * @param index 缓冲区当前读取位置的索引指针，读取后自动递增
 * @return 解码后的浮点数
 */
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);
/**
 * @brief 从缓冲区读取定点数编码的浮点数（32位）
 * 
 * 解码原理：读取 int32_t 后除以 scale 缩放因子还原浮点数
 *   - 与 buffer_append_float32 配对使用
 * 
 * @param buffer 源字节缓冲区
 * @param scale 缩放因子（需与序列化时使用的 scale 一致）
 * @param index 缓冲区当前读取位置的索引指针，读取后自动递增
 * @return 解码后的浮点数
 */
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);
/**
 * @brief 从缓冲区读取自动精度编码的浮点数（32位）
 * 
 * 解码原理：
 *   1. 读取 32 位整数
 *   2. 提取符号位、指数(8位)、尾数(23位)
 *   3. 使用 ldexpf() 将尾数和指数还原为浮点数
 * 
 *   - 与 buffer_append_float32_auto 配对使用
 * 
 * @param buffer 源字节缓冲区
 * @param index 缓冲区当前读取位置的索引指针，读取后自动递增
 * @return 解码后的浮点数
 */
float buffer_get_float32_auto(const uint8_t *buffer, int32_t *index);



#endif /* BUFFER_H_ */
