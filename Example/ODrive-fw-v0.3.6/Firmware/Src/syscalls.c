/*
 * ============================================================================
 * 文件名: syscalls.c
 *
 * 文件用途:
 *   本文件重定向标准C库的系统调用（_write），实现printf功能通过UART或USB输出。
 *   这是嵌入式系统中实现调试输出的标准方法。
 *
 * 主要功能模块：
 *   1. _write()：重定向标准输出，支持USB CDC和UART DMA两种通道
 *   2. HAL_UART_TxCpltCallback()：UART DMA发送完成回调，释放信号量
 *
 * 发送通道选择:
 *   - USB CDC: 通过CDC_Transmit_FS发送，等待sem_usb_tx信号量
 *   - UART DMA: 通过HAL_UART_Transmit_DMA后台发送，等待sem_uart_dma信号量
 *   - 缓冲区大小: 64字节
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

#include <cmsis_os.h>
#include <commands.h>
#include <freertos_vars.h>
//#include <sys/unistd.h>
#include <usart.h>
#include <usbd_cdc_if.h>

//int _read(int file, char *data, int len) {}
//int _close(int file) {}
//int _lseek(int file, int ptr, int dir) {}
//int _fstat(int file, struct stat *st) {}
//int _isatty(int file) {}

#define UART_TX_BUFFER_SIZE 64
static uint8_t uart_tx_buf[UART_TX_BUFFER_SIZE];

/**
 * @brief 重定向_write系统调用，实现printf功能
 * 
 * 功能说明：
 * 重写标准库的_write函数，使printf()等标准输出函数可以通过UART或USB发送数据。
 * 这是嵌入式系统中实现printf功能的常见方法。
 * 
 * 发送通道选择：
 * 通过serial_printf_select变量选择发送通道：
 * 1. SERIAL_PRINTF_IS_USB: 通过USB CDC发送
 *    - 等待sem_usb_tx信号量（100ms超时）
 *    - 使用CDC_Transmit_FS()发送数据
 * 
 * 2. SERIAL_PRINTF_IS_UART: 通过UART DMA发送
 *    - 检查数据长度不超过缓冲区大小(64字节)
 *    - 等待sem_uart_dma信号量（100ms超时）
 *    - 使用HAL_UART_Transmit_DMA()后台DMA发送
 * 
 * 信号量机制：
 * 使用信号量保证同一时间只有一个任务使用发送通道，
 * 避免数据冲突和覆盖。
 * 
 * @param file: 文件描述符（标准输出通常为1）
 * @param data: 要发送的数据指针
 * @param len: 数据长度
 * @return 实际发送的字节数，发送失败返回0
 */
int _write(int file, char* data, int len) {
    // 已写入的字节数
    int written = 0;
    switch (serial_printf_select) {
        case SERIAL_PRINTF_IS_USB: {
            // 等待USB接口可用信号量
            const uint32_t usb_tx_timeout = 100; // ms
            osStatus sem_stat = osSemaphoreWait(sem_usb_tx, usb_tx_timeout);
            if (sem_stat == osOK) {
                uint8_t status = CDC_Transmit_FS((uint8_t*)data, len);  // 通过USB CDC发送
                written = (status == USBD_OK) ? len : 0;
            } // 如果信号量超时，written保持为0
        } break;

        case SERIAL_PRINTF_IS_UART: {
            // 检查数据长度是否超过缓冲区
            if (len > UART_TX_BUFFER_SIZE)
                return 0;
            // 等待UART DMA接口可用信号量
            const uint32_t uart_tx_timeout = 100; // ms
            osStatus sem_stat = osSemaphoreWait(sem_uart_dma, uart_tx_timeout);
            if (sem_stat == osOK) {
                memcpy(uart_tx_buf, data, len);                    // 复制数据到发送缓冲区
                HAL_UART_Transmit_DMA(&huart4, uart_tx_buf, len);  // 启动DMA后台传输
            } // 如果信号量超时，written保持为0
        } break;

        default: {
            written = 0;
        } break;
    }

    return written;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    osSemaphoreRelease(sem_uart_dma);
}
