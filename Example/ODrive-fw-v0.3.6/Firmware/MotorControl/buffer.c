/**
 * @file buffer.c
 * @brief 数据序列化/反序列化工具实现
 * 
 * 本文件实现了将各种数据类型序列化为字节流（大端序）的功能。
 * 主要用于通信协议中数据的打包与解包操作。
 */
		
#include "utils.h"
#include "buffer.h"
#include <math.h>
#include <stdbool.h>

/*
 * ============================================================================
 * 【大端序序列化函数】
 * ============================================================================
 * 大端序（Big-Endian）：高字节存放在低地址，低字节存放在高地址
 * 例如：uint32_t 值 0x12345678 序列化为字节流：
 *   buffer[0] = 0x12  (最高字节)
 *   buffer[1] = 0x34
 *   buffer[2] = 0x56
 *   buffer[3] = 0x78  (最低字节)
 *
 * index 参数是一个指针，指向缓冲区中当前的写入位置。
 * 每次写入数据后，index 会自动递增相应字节数，便于连续写入多个数据。
 * ============================================================================
 */

/**
 * @brief 将 int16_t 有符号16位整数以大端序追加到缓冲区
 * 
 * 实现原理：
 *   - 右移8位取出高8位存入 buffer[index]
 *   - 直接取低8位存入 buffer[index+1]
 *   - index 递增2
 * 
 * @param buffer  目标字节缓冲区
 * @param number  要写入的16位整数（如：0x1234）
 * @param index   缓冲区索引指针，写入后自增2
 * 
 * 示例：number = 0x1234
 *   buffer[(*index)++] = 0x12
 *   buffer[(*index)++] = 0x34
 */
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;   /* 取出高8位，存入低地址 */
    buffer[(*index)++] = number;         /* 取出低8位，存入高地址 */
}

/**
 * @brief 将 uint16_t 无符号16位整数以大端序追加到缓冲区
 * 
 * 实现原理与 buffer_append_int16 相同，区别仅在于处理无符号类型。
 * 
 * @param buffer  目标字节缓冲区
 * @param number  要写入的无符号16位整数
 * @param index   缓冲区索引指针，写入后自增2
 */
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;   /* 取出高8位，存入低地址 */
    buffer[(*index)++] = number;         /* 取出低8位，存入高地址 */
}

/**
 * @brief 将 int32_t 有符号32位整数以大端序追加到缓冲区
 * 
 * 实现原理：
 *   - 依次右移 24/16/8/0 位取出4个字节
 *   - 按从高字节到低字节顺序存入缓冲区
 *   - index 递增4
 * 
 * @param buffer  目标字节缓冲区
 * @param number  要写入的32位整数
 * @param index   缓冲区索引指针，写入后自增4
 * 
 * 示例：number = 0x12345678
 *   buffer[0] = 0x12, buffer[1] = 0x34, buffer[2] = 0x56, buffer[3] = 0x78
 */
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;  /* 取出最高字节 (bits 24-31) */
    buffer[(*index)++] = number >> 16;  /* 取出次高字节 (bits 16-23) */
    buffer[(*index)++] = number >> 8;   /* 取出次低字节 (bits 8-15) */
    buffer[(*index)++] = number;         /* 取出最低字节 (bits 0-7) */
}

/**
 * @brief 将 uint32_t 无符号32位整数以大端序追加到缓冲区
 * 
 * 实现原理与 buffer_append_int32 相同，区别仅在于处理无符号类型。
 * 
 * @param buffer  目标字节缓冲区
 * @param number  要写入的无符号32位整数
 * @param index   缓冲区索引指针，写入后自增4
 */
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;  /* 取出最高字节 (bits 24-31) */
    buffer[(*index)++] = number >> 16;  /* 取出次高字节 (bits 16-23) */
    buffer[(*index)++] = number >> 8;   /* 取出次低字节 (bits 8-15) */
    buffer[(*index)++] = number;         /* 取出最低字节 (bits 0-7) */
}

/*
 * ============================================================================
 * 【浮点数定点编码序列化】
 * ============================================================================
 * 定点数编码原理：将浮点数通过乘以缩放因子（scale）转换为整数
 * 
 * 序列化公式：stored_value = (int)(number * scale)
 * 反序列化公式：original_value = stored_value / scale
 * 
 * 优点：
 *   - 节省存储空间（float16仅2字节，float32为4字节）
 *   - 适合嵌入式系统，避免直接传输IEEE 754浮点数的平台差异
 *   - 精度可控，通过调整scale参数
 * 
 * 缺点：
 *   - 存在精度损失，小数部分被截断
 *   - 可表示范围受限于整数类型的范围
 * 
 * scale 选择建议：
 *   - scale = 10    → 精度 0.1，   int16范围: -3276.8 ~ 3276.7
 *   - scale = 100   → 精度 0.01，  int16范围: -327.68 ~ 327.67
 *   - scale = 1000  → 精度 0.001， int16范围: -32.768 ~ 32.767
 *   - scale = 1000000 → 精度 0.000001，int32范围: -2147.48 ~ 2147.48
 * ============================================================================
 */

/**
 * @brief 将 float 浮点数以定点数格式序列化（16位编码）
 * 
 * 编码步骤：
 *   1. number * scale：将浮点数放大 scale 倍
 *   2. (int16_t)：强制转换为16位整数（截断小数部分）
 *   3. buffer_append_int16：以大端序写入缓冲区
 * 
 * @param buffer  目标字节缓冲区
 * @param number  要写入的浮点数
 * @param scale   缩放因子
 * @param index   缓冲区索引指针，写入后自增2
 * 
 * 使用示例：
 *   float temp = 25.5f;
 *   buffer_append_float16(buf, temp, 100.0f, &idx);
 *   // 实际写入的整数值为 2550
 */
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
    /* 浮点数 × 缩放因子 → 取整 → 以 int16_t 序列化 */
    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

/**
 * @brief 将 float 浮点数以定点数格式序列化（32位编码）
 * 
 * 编码步骤：
 *   1. number * scale：将浮点数放大 scale 倍
 *   2. (int32_t)：强制转换为32位整数（截断小数部分）
 *   3. buffer_append_int32：以大端序写入缓冲区
 * 
 * @param buffer  目标字节缓冲区
 * @param number  要写入的浮点数
 * @param scale   缩放因子
 * @param index   缓冲区索引指针，写入后自增4
 * 
 * 使用示例：
 *   float voltage = 12.3456f;
 *   buffer_append_float32(buf, voltage, 10000.0f, &idx);
 *   // 实际写入的整数值为 123456
 */
void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
    /* 浮点数 × 缩放因子 → 取整 → 以 int32_t 序列化 */
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

/*
 * ============================================================================
 * 【浮点数自动精度编码 - buffer_append_float32_auto】
 * ============================================================================
 * 
 * 该方法将浮点数手动编码为32位整数，模拟 IEEE 754 单精度浮点数格式，
 * 保证跨平台的数据一致性（不依赖宿主机的浮点数存储格式）。
 * 
 * IEEE 754 单精度浮点数格式（32位）：
 *   [31]     符号位 (1 bit)：0=正数, 1=负数
 *   [30-23]  指数位 (8 bits)：以 127 为偏置值的指数
 *   [22-0]   尾数位 (23 bits)：规格化后的小数部分
 * 
 * 编码流程：
 *   1. frexpf(number, &e) 将浮点数分解为：number = sig × 2^e
 *      - sig 的范围是 [0.5, 1.0) 或 (-1.0, -0.5]，或者为 0
 *      - e 是指数值
 * 
 *   2. 将 sig 映射到 23 位整数：
 *      - 因为 sig ∈ [0.5, 1.0)，所以 (sig_abs - 0.5) × 2.0 ∈ [0, 1)
 *      - 再乘以 2^23 = 8388608.0，得到 23 位整数
 *      - 此时 e 需要 +1 补偿（因为减去了 0.5）
 *      - 最终 e += 126（127偏置值 - 1补偿 = 126）
 * 
 *   3. 打包为32位整数：
 *      - 指数部分：(e & 0xFF) << 23
 *      - 尾数部分：sig_i & 0x7FFFFF
 *      - 符号位：如果 sig < 0，设置最高位
 * 
 * 参考：
 *   http://stackoverflow.com/questions/40416682/portable-way-to-serialize-float-as-32-bit-integer
 * ============================================================================
 */

/**
 * @brief 将 float 浮点数以 IEEE 754 兼容格式自动序列化（32位）
 * 
 * 特点：
 *   - 无需指定 scale 参数，自动适应数值大小
 *   - 精度与原始 float 完全一致（约6~7位有效数字）
 *   - 可表示极大范围（约 ±3.4×10^38）和极小数值
 *   - 手动编码保证跨平台字节级一致性
 * 
 * @param buffer  目标字节缓冲区
 * @param number  要写入的浮点数
 * @param index   缓冲区索引指针，写入后自增4
 * 
 * 示例：
 *   float pi = 3.14159265358979f;
 *   buffer_append_float32_auto(buf, pi, &idx);
 *   // 精确编码所有有效位，无需担心 scale 选择
 */
void buffer_append_float32_auto(uint8_t* buffer, float number, int32_t *index) {
    /* 步骤1：将浮点数分解为尾数(sig)和指数(e) */
    int e = 0;
    float sig = frexpf(number, &e);    /* number = sig × 2^e, sig ∈ [0.5, 1.0) */
    float sig_abs = fabsf(sig);        /* 取尾数的绝对值 */
    uint32_t sig_i = 0;                /* 用于存储23位尾数整数 */

    /* 步骤2：将尾数 sig 映射到 23 位整数
     * 因为 frexpf 返回的 sig ∈ [0.5, 1.0) 或 0
     * 所以先减去 0.5，再乘以 2.0，得到 [0, 1) 范围
     * 再乘以 2^23 = 8388608.0，映射到 [0, 2^23) 的整数范围
     */
    if (sig_abs >= 0.5f) {
        sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);  /* 映射到23位整数 */
        e += 126;  /* 指数偏置：127(IEEE偏置) - 1(sig减去0.5的补偿) = 126 */
    }

    /* 步骤3：将指数、尾数、符号位打包为32位整数
     * 位域布局：[31:符号][30-23:指数][22-0:尾数]
     */
    uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);  /* 指数和尾数组合 */
    if (sig < 0) {
        res |= 1 << 31;  /* 设置符号位（负数） */
    }

    /* 步骤4：以大端序写入32位整数 */
    buffer_append_uint32(buffer, res, index);
}

/*
 * ============================================================================
 * 【大端序反序列化函数】
 * ============================================================================
 * 将缓冲区中的字节流按大端序重新组合为原始数据类型
 * 
 * 读取顺序：低地址字节为高字节，高地址字节为低字节
 * 例如：buffer = {0x12, 0x34, 0x56, 0x78} → uint32_t = 0x12345678
 * ============================================================================
 */

/**
 * @brief 从缓冲区以大端序读取 int16_t 有符号16位整数
 * 
 * 实现原理：
 *   - 将 buffer[index] 作为高8位，左移8位
 *   - 将 buffer[index+1] 作为低8位，直接按位或
 *   - index 递增2
 * 
 * @param buffer  源字节缓冲区
 * @param index   缓冲区索引指针，读取后自增2
 * @return        读取到的16位整数
 * 
 * 示例：buffer = {0x12, 0x34}
 *   res = (0x12 << 8) | 0x34 = 0x1234
 */
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
    int16_t res = ((uint16_t) buffer[*index]) << 8 |   /* 高8位 */
                  ((uint16_t) buffer[*index + 1]);      /* 低8位 */
    *index += 2;
    return res;
}

/**
 * @brief 从缓冲区以大端序读取 uint16_t 无符号16位整数
 * 
 * @param buffer  源字节缓冲区
 * @param index   缓冲区索引指针，读取后自增2
 * @return        读取到的无符号16位整数
 */
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
    uint16_t res = ((uint16_t) buffer[*index]) << 8 |   /* 高8位 */
                   ((uint16_t) buffer[*index + 1]);      /* 低8位 */
    *index += 2;
    return res;
}

/**
 * @brief 从缓冲区以大端序读取 int32_t 有符号32位整数
 * 
 * 实现原理：
 *   - 依次将4个字节按高位到低位左移 24/16/8/0 位
 *   - 按位或组合为完整的32位整数
 *   - index 递增4
 * 
 * @param buffer  源字节缓冲区
 * @param index   缓冲区索引指针，读取后自增4
 * @return        读取到的32位整数
 * 
 * 示例：buffer = {0x12, 0x34, 0x56, 0x78}
 *   res = (0x12<<24) | (0x34<<16) | (0x56<<8) | 0x78 = 0x12345678
 */
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
    int32_t res = ((uint32_t) buffer[*index]) << 24 |      /* 最高字节 */
                  ((uint32_t) buffer[*index + 1]) << 16 |  /* 次高字节 */
                  ((uint32_t) buffer[*index + 2]) << 8 |   /* 次低字节 */
                  ((uint32_t) buffer[*index + 3]);          /* 最低字节 */
    *index += 4;
    return res;
}

/**
 * @brief 从缓冲区以大端序读取 uint32_t 无符号32位整数
 * 
 * @param buffer  源字节缓冲区
 * @param index   缓冲区索引指针，读取后自增4
 * @return        读取到的无符号32位整数
 */
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
    uint32_t res = ((uint32_t) buffer[*index]) << 24 |      /* 最高字节 */
                   ((uint32_t) buffer[*index + 1]) << 16 |  /* 次高字节 */
                   ((uint32_t) buffer[*index + 2]) << 8 |   /* 次低字节 */
                   ((uint32_t) buffer[*index + 3]);          /* 最低字节 */
    *index += 4;
    return res;
}

/*
 * ============================================================================
 * 【浮点数定点编码反序列化】
 * ============================================================================
 * 定点数解码原理：从缓冲区读取整数后除以 scale 缩放因子
 * 
 * 反序列化公式：original_value = stored_value / scale
 * 
 * 注意：scale 参数必须与序列化时使用的 scale 一致，
 * 否则会得到错误的结果。
 * ============================================================================
 */

/**
 * @brief 从缓冲区读取定点数编码的浮点数（16位）
 * 
 * 解码步骤：
 *   1. buffer_get_int16：以大端序读取16位整数
 *   2. 除以 scale：还原为浮点数
 * 
 * @param buffer  源字节缓冲区
 * @param scale   缩放因子（必须与序列化时一致）
 * @param index   缓冲区索引指针，读取后自增2
 * @return        解码后的浮点数
 * 
 * 使用示例：
 *   float temp = buffer_get_float16(buf, 100.0f, &idx);
 *   // 如果缓冲区中存储的是 2550，则返回 25.5
 */
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
    /* 读取 int16_t 整数 → 除以缩放因子 → 还原浮点数 */
    return (float)buffer_get_int16(buffer, index) / scale;
}

/**
 * @brief 从缓冲区读取定点数编码的浮点数（32位）
 * 
 * 解码步骤：
 *   1. buffer_get_int32：以大端序读取32位整数
 *   2. 除以 scale：还原为浮点数
 * 
 * @param buffer  源字节缓冲区
 * @param scale   缩放因子（必须与序列化时一致）
 * @param index   缓冲区索引指针，读取后自增4
 * @return        解码后的浮点数
 * 
 * 使用示例：
 *   float voltage = buffer_get_float32(buf, 10000.0f, &idx);
 *   // 如果缓冲区中存储的是 123456，则返回 12.3456
 */
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
    /* 读取 int32_t 整数 → 除以缩放因子 → 还原浮点数 */
    return (float)buffer_get_int32(buffer, index) / scale;
}

/*
 * ============================================================================
 * 【浮点数自动精度解码 - buffer_get_float32_auto】
 * ============================================================================
 * 
 * 解码流程（与编码过程相反）：
 *   1. 从缓冲区读取32位整数
 *   2. 提取各字段：
 *      - 符号位：bit 31
 *      - 指数位：bits 30-23
 *      - 尾数位：bits 22-0
 *   3. 将23位尾数整数还原为小数：sig = sig_i / (2^23 × 2) + 0.5
 *   4. 还原指数：e -= 126
 *   5. 处理符号：如果为负数，sig = -sig
 *   6. ldexpf(sig, e) 计算 sig × 2^e，得到最终浮点数
 * 
 * 这是 buffer_append_float32_auto 的逆过程。
 * ============================================================================
 */

/**
 * @brief 从缓冲区读取自动精度编码的浮点数（32位）
 * 
 * @param buffer  源字节缓冲区
 * @param index   缓冲区索引指针，读取后自增4
 * @return        解码后的浮点数
 * 
 * 示例：
 *   float pi = buffer_get_float32_auto(buf, &idx);
 *   // 精确还原所有有效位
 */
float buffer_get_float32_auto(const uint8_t *buffer, int32_t *index) {
    /* 步骤1：读取32位整数 */
    uint32_t res = buffer_get_uint32(buffer, index);

    /* 步骤2：提取IEEE 754格式各字段 */
    int e = (res >> 23) & 0xFF;           /* 指数位 (bits 30-23) */
    uint32_t sig_i = res & 0x7FFFFF;      /* 尾数位 (bits 22-0) */
    bool neg = res & (1 << 31);           /* 符号位 (bit 31) */

    /* 步骤3：将23位尾数整数还原为小数
     * 编码时：sig_i = (sig_abs - 0.5) × 2 × 2^23
     * 解码时：sig = sig_i / (2^23 × 2) + 0.5
     * 8388608.0f = 2^23
     */
    float sig = 0.0f;
    if (e != 0 || sig_i != 0) {
        sig = (float)sig_i / (8388608.0f * 2.0f) + 0.5f;  /* 还原尾数 */
        e -= 126;  /* 还原指数偏置 */
    }

    /* 步骤4：处理符号 */
    if (neg) {
        sig = -sig;
    }

    /* 步骤5：计算最终浮点数 sig × 2^e */
    return ldexpf(sig, e);
}
