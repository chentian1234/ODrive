/*
 * ============================================================================
 * 文件名: freertos_vars.h
 *
 * 文件用途:
 *   本文件声明所有FreeRTOS任务和信号量的外部变量，供其他模块引用。
 *   包含：
 *     - 4个信号量：USB接收/发送、UART DMA、USB中断
 *     - 4个任务句柄：电机0/1、命令解析、USB处理
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_H
#define __FREERTOS_H

/* 信号量声明 - 用于任务和中断之间的同步控制 */
extern osSemaphoreId sem_usb_irq;    /* USB中断信号量：由USB ISR释放，usb_update_thread等待 */
extern osSemaphoreId sem_uart_dma;   /* UART DMA信号量：控制UART DMA传输的互斥访问 */
extern osSemaphoreId sem_usb_rx;     /* USB接收信号量：标记USB CDC数据已接收 */
extern osSemaphoreId sem_usb_tx;     /* USB发送信号量：控制USB CDC数据发送的互斥访问 */

/* 任务句柄声明 - 用于外部模块访问任务状态 */
extern osThreadId thread_motor_0;    /* 电机0控制线程句柄（最高优先级 osPriorityHigh+1） */
extern osThreadId thread_motor_1;    /* 电机1控制线程句柄（高优先级 osPriorityHigh） */
extern osThreadId thread_cmd_parse;  /* 命令解析线程句柄（普通优先级 osPriorityNormal） */
extern osThreadId thread_usb_pump;   /* USB处理线程句柄（普通优先级 osPriorityNormal） */

#endif /* __FREERTOS_H */