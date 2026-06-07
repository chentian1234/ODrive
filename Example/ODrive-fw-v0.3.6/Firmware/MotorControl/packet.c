/**
 * @file packet.c
 * @brief 轻量级串行通信数据包协议实现
 *
 * 实现了基于状态机的串行数据包收发功能，支持短帧和长帧两种模式。
 */

#include <string.h>
#include "packet.h"
#include "crc.h"

/**
 * @brief 协议处理器的内部状态结构体
 *
 * 每个 handler 对应一个独立的通信通道，维护完整的收发上下文：
 *
 * - rx_state:       当前接收状态机的状态值（0~6）
 *                    0: 空闲/等待起始符
 *                    1: 已收到长帧起始符，等待长度高字节
 *                    2: 已收到长度高字节，等待长度低字节
 *                    3: 正在接收有效载荷数据
 *                    4: 数据接收完毕，等待CRC高字节
 *                    5: CRC高字节已收到，等待CRC低字节
 *                    6: CRC已收齐，等待帧尾结束符
 *
 * - rx_timeout:     接收超时计数器（毫秒）。每收到一个有效字节即重置为
 *                    PACKET_RX_TIMEOUT，每毫秒由 packet_timerfunc 递减。
 *                    归零时触发状态机复位。
 *
 * - send_func:      发送回调函数指针，组帧完成后调用此函数将数据通过底层
 *                    硬件（如 UART）发出。
 *
 * - process_func:   接收处理回调函数指针，当完整帧到达且CRC校验通过后，
 *                    通过此函数将有效载荷数据递交给上层应用。
 *
 * - payload_length: 当前帧的有效载荷长度，在接收长度字段时解析得到。
 *
 * - rx_buffer:      接收缓冲区，用于暂存正在接收的有效载荷数据。
 *                    大小由 PACKET_MAX_PL_LEN 定义。
 *
 * - tx_buffer:      发送缓冲区，用于组装待发送的完整帧。
 *                    大小 = 最大载荷 + 6（起始符1 + 最大长度字段2 + CRC2 + 结束符1）
 *
 * - rx_data_ptr:    接收数据指针，指向 rx_buffer 中下一个写入位置。
 *                    用于追踪当前已接收到的有效载荷字节数。
 *
 * - crc_high:       暂存接收到的CRC高字节
 *
 * - crc_low:        暂存接收到的CRC低字节
 */
typedef struct {
    volatile unsigned char rx_state;                    /**< 当前接收状态机状态 (0~6) */
    volatile unsigned short rx_timeout;                 /**< 接收超时计数器 (ms) */
    void(*send_func)(unsigned char *data, unsigned int len);    /**< 发送回调函数指针 */
    void(*process_func)(unsigned char *data, unsigned int len); /**< 接收处理回调函数指针 */
    unsigned int payload_length;                        /**< 当前帧有效载荷长度 */
    unsigned char rx_buffer[PACKET_MAX_PL_LEN];         /**< 接收缓冲区 */
    unsigned char tx_buffer[PACKET_MAX_PL_LEN + 6];     /**< 发送缓冲区 (+6为帧头/尾开销) */
    unsigned int rx_data_ptr;                           /**< 接收数据写入指针 */
    unsigned char crc_low;                              /**< CRC低字节暂存 */
    unsigned char crc_high;                             /**< CRC高字节暂存 */
} PACKET_STATE_t;

/**
 * @brief 所有处理器的状态数组
 *
 * 静态分配，大小为 PACKET_HANDLERS。每个元素独立维护一个通信通道的完整状态。
 */
static PACKET_STATE_t handler_states[PACKET_HANDLERS];

/**
 * @brief 初始化指定的数据包处理器
 * @param s_func 发送回调函数指针
 * @param p_func 接收处理回调函数指针
 * @param handler_num 处理器编号（0 ~ PACKET_HANDLERS-1）
 */
void packet_init(void (*s_func)(unsigned char *data, unsigned int len),
                 void (*p_func)(unsigned char *data, unsigned int len), int handler_num) {
    handler_states[handler_num].send_func = s_func;
    handler_states[handler_num].process_func = p_func;
}

/**
 * @brief 组装并发送一个数据包
 * @param data 指向有效载荷数据的指针
 * @param len 有效载荷数据的长度
 * @param handler_num 处理器编号
 *
 * 组帧过程详细说明：
 *
 * 1. 长度检查：若超过 PACKET_MAX_PL_LEN 则直接返回，不发送
 *
 * 2. 写入帧头：
 *    - 短帧模式 (len <= 256):
 *      tx_buffer[0] = 0x02  (短帧起始符)
 *      tx_buffer[1] = len   (1字节长度)
 *      帧头共2字节
 *
 *    - 长帧模式 (len > 256):
 *      tx_buffer[0] = 0x03       (长帧起始符)
 *      tx_buffer[1] = len >> 8   (长度高字节，大端序)
 *      tx_buffer[2] = len & 0xFF (长度低字节)
 *      帧头共3字节
 *
 * 3. 拷贝有效载荷数据到 tx_buffer
 *
 * 4. 计算CRC16校验值（对有效载荷数据计算），写入2字节：
 *    - 先写高字节 (crc >> 8)
 *    - 再写低字节 (crc & 0xFF)
 *
 * 5. 写入帧尾结束符 0x03
 *
 * 6. 通过 send_func 回调函数将完整帧发出
 *
 * 最终帧格式：
 *   短帧: [0x02][len(1B)][data...][CRC_H][CRC_L][0x03]
 *   长帧: [0x03][len_H][len_L][data...][CRC_H][CRC_L][0x03]
 */
void packet_send_packet(unsigned char *data, unsigned int len, int handler_num) {
    /* 检查数据长度是否超出协议允许的最大值 */
    if (len > PACKET_MAX_PL_LEN) {
        return;
    }

    int b_ind = 0;  /* 发送缓冲区写入索引 */

    /* 根据数据长度选择帧模式：短帧(<=256)或长帧(>256) */
    if (len <= 256) {
        /* 短帧模式: 起始符 0x02 + 1字节长度 */
        handler_states[handler_num].tx_buffer[b_ind++] = 2;
        handler_states[handler_num].tx_buffer[b_ind++] = len;
    } else {
        /* 长帧模式: 起始符 0x03 + 2字节长度(大端序) */
        handler_states[handler_num].tx_buffer[b_ind++] = 3;
        handler_states[handler_num].tx_buffer[b_ind++] = len >> 8;      /* 长度高字节 */
        handler_states[handler_num].tx_buffer[b_ind++] = len & 0xFF;    /* 长度低字节 */
    }

    /* 将有效载荷数据拷贝到发送缓冲区 */
    memcpy(handler_states[handler_num].tx_buffer + b_ind, data, len);
    b_ind += len;

    /* 计算CRC16校验值（仅对有效载荷数据计算），并写入帧 */
    unsigned short crc = crc16(data, len);
    handler_states[handler_num].tx_buffer[b_ind++] = (uint8_t)(crc >> 8);   /* CRC高字节 */
    handler_states[handler_num].tx_buffer[b_ind++] = (uint8_t)(crc & 0xFF); /* CRC低字节 */

    /* 写入帧尾结束符 */
    handler_states[handler_num].tx_buffer[b_ind++] = 3;

    /* 通过注册的发送回调函数发出完整帧 */
    if (handler_states[handler_num].send_func) {
        handler_states[handler_num].send_func(handler_states[handler_num].tx_buffer, b_ind);
    }
}

/**
 * @brief 周期调用的超时检查函数
 *
 * 应在主循环或定时器中断中每1毫秒调用一次。
 *
 * 超时处理机制：
 *   - 遍历所有已注册的处理器
 *   - 对每个处理器，若 rx_timeout > 0 则递减计数器
 *   - 若 rx_timeout 已归零，说明超时发生，将 rx_state 强制复位为 0
 *     （回到空闲状态，丢弃当前未完成的帧）
 *
 * 这种设计确保了即使在通信中断或数据丢失的情况下，
 * 接收状态机也不会永久卡在中间状态，具备自我恢复能力。
 */
void packet_timerfunc(void) {
    for (int i = 0; i < PACKET_HANDLERS; i++) {
        if (handler_states[i].rx_timeout) {
            handler_states[i].rx_timeout--;      /* 递减超时计数器 */
        } else {
            handler_states[i].rx_state = 0;      /* 超时，状态机复位 */
        }
    }
}

/**
 * @brief 逐字节处理接收到的串行数据
 * @param rx_data 当前接收到的字节
 * @param handler_num 处理器编号
 *
 * 核心接收逻辑：7状态（0~6）有限状态机
 *
 * 状态转换图：
 *
 *              +--------+
 *              | State 0 |  空闲/等待起始符
 *              +----+---+
 *                   |
 *           rx==2   |   rx==3
 *         (短帧)    |  (长帧)
 *              +----+----+
 *              |         |
 *              v         v
 *         +----+---+  +--+---+
 *         | State 2|  |State 1|  等待长度高字节(仅长帧)
 *         +---+----+  +--+---+
 *             ^          |
 *             |    rx数据 |
 *             +----------+
 *                   |
 *                   v
 *              +----+---+
 *              | State 2 |  等待长度低字节
 *              +----+----+
 *                   |
 *         长度合法(1~MAX)
 *                   |
 *                   v
 *              +----+---+
 *              | State 3 |  接收有效载荷数据
 *              +----+----+
 *                   |
 *         已接收payload_length字节
 *                   |
 *                   v
 *              +----+---+
 *              | State 4 |  等待CRC高字节
 *              +----+----+
 *                   |
 *                   v
 *              +----+---+
 *              | State 5 |  等待CRC低字节
 *              +----+----+
 *                   |
 *                   v
 *              +----+---+
 *              | State 6 |  等待帧尾结束符(0x03)
 *              +----+----+
 *                   |
 *            rx==3  |  CRC校验通过
 *                   v
 *              调用process_func回调
 *                   |
 *                   v
 *              回到 State 0
 *
 * 任何状态下收到无效数据或发生错误，都会回到 State 0 重新开始。
 * 每收到一个有效字节，rx_timeout 都会被重置为 PACKET_RX_TIMEOUT。
 */
void packet_process_byte(uint8_t rx_data, int handler_num) {
    switch (handler_states[handler_num].rx_state) {
        /*
         * 状态0: 空闲状态 / 等待帧起始符
         *
         * - 收到 0x02: 短帧起始，转入状态2（1字节长度模式，直接跳到等待长度低字节状态）
         * - 收到 0x03: 长帧起始，转入状态1（需要接收2字节长度）
         * - 收到其他: 忽略，保持状态0
         */
        case 0:
            if (rx_data == 2) {
                /* 短帧起始符(0x02)：1字节长度模式 */
                /* 状态 +2 直接跳到状态2，跳过状态1（长度高字节） */
                handler_states[handler_num].rx_state += 2;
                handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
                handler_states[handler_num].rx_data_ptr = 0;
                handler_states[handler_num].payload_length = 0;
            } else if (rx_data == 3) {
                /* 长帧起始符(0x03)：2字节长度模式 */
                handler_states[handler_num].rx_state++;
                handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
                handler_states[handler_num].rx_data_ptr = 0;
                handler_states[handler_num].payload_length = 0;
            } else {
                /* 非起始符，保持空闲状态 */
                handler_states[handler_num].rx_state = 0;
            }
            break;

        /*
         * 状态1: 等待长度高字节（仅长帧模式）
         *
         * 收到的字节作为长度字段的高8位，左移8位后存入 payload_length。
         * 完成后转入状态2等待长度低字节。
         */
        case 1:
            handler_states[handler_num].payload_length = (unsigned int)rx_data << 8;
            handler_states[handler_num].rx_state++;
            handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
            break;

        /*
         * 状态2: 等待长度低字节
         *
         * - 短帧: payload_length 初始为0，此处直接写入1字节长度值
         * - 长帧: payload_length 已有高字节，此处用 | 运算合并低字节
         *
         * 长度合法性检查：必须 > 0 且 <= PACKET_MAX_PL_LEN，
         * 否则丢弃该帧，回到状态0。
         */
        case 2:
            handler_states[handler_num].payload_length |= (unsigned int)rx_data;
            if (handler_states[handler_num].payload_length > 0 &&
                handler_states[handler_num].payload_length <= PACKET_MAX_PL_LEN) {
                /* 长度合法，转入状态3开始接收数据 */
                handler_states[handler_num].rx_state++;
                handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
            } else {
                /* 长度非法(0或超出上限)，复位状态机 */
                handler_states[handler_num].rx_state = 0;
            }
            break;

        /*
         * 状态3: 接收有效载荷数据
         *
         * 将每个收到的字节依次写入 rx_buffer，直到接收的字节数
         * 等于 payload_length 为止，然后转入状态4。
         */
        case 3:
            handler_states[handler_num].rx_buffer[handler_states[handler_num].rx_data_ptr++] = rx_data;
            if (handler_states[handler_num].rx_data_ptr == handler_states[handler_num].payload_length) {
                /* 数据接收完毕，转入状态4等待CRC */
                handler_states[handler_num].rx_state++;
            }
            handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
            break;

        /*
         * 状态4: 等待CRC高字节
         *
         * 收到的字节作为CRC16校验值的高8位暂存。
         * 完成后转入状态5。
         */
        case 4:
            handler_states[handler_num].crc_high = rx_data;
            handler_states[handler_num].rx_state++;
            handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
            break;

        /*
         * 状态5: 等待CRC低字节
         *
         * 收到的字节作为CRC16校验值的低8位暂存。
         * 完成后转入状态6等待帧尾。
         */
        case 5:
            handler_states[handler_num].crc_low = rx_data;
            handler_states[handler_num].rx_state++;
            handler_states[handler_num].rx_timeout = PACKET_RX_TIMEOUT;
            break;

        /*
         * 状态6: 等待帧尾结束符(0x03)
         *
         * - 若收到 0x03: 验证CRC校验值
         *   计算 rx_buffer 中数据的 CRC16，与接收到的 crc_high/crc_low 比较
         *   校验通过则调用 process_func 回调函数递交数据
         *
         * - 若收到非 0x03: CRC不验证，直接丢弃该帧
         *
         * 无论结果如何，最后都回到状态0等待下一帧。
         */
        case 6:
            if (rx_data == 3) {
                /* 收到帧尾结束符，进行CRC校验 */
                if (crc16(handler_states[handler_num].rx_buffer, handler_states[handler_num].payload_length)
                    == ((unsigned short)handler_states[handler_num].crc_high << 8
                        | (unsigned short)handler_states[handler_num].crc_low)) {
                    /* CRC校验通过，数据包接收成功！ */
                    if (handler_states[handler_num].process_func) {
                        handler_states[handler_num].process_func(handler_states[handler_num].rx_buffer,
                                handler_states[handler_num].payload_length);
                    }
                }
                /* CRC校验失败时静默丢弃，不执行任何操作 */
            }
            /* 无论是否成功接收，都回到初始状态等待下一帧 */
            handler_states[handler_num].rx_state = 0;
            break;

        /*
         * 默认分支: 遇到未知状态值，强制复位到状态0
         * （防御性编程，确保状态机不会因异常情况陷入死循环）
         */
        default:
            handler_states[handler_num].rx_state = 0;
            break;
    }
}
