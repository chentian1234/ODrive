/*
 * ============================================================================
 * 文件名: FreeRTOSConfig.h
 *
 * 文件用途:
 *   本文件是FreeRTOS实时操作系统的配置文件，定义所有RTOS核心参数。
 *   包括：
 *     - 调度器配置（抢占式/时间片、CPU时钟、Tick频率）
 *     - 内存配置（总堆大小15KB、最小栈128字、最大优先级7）
 *     - 中断配置（最低优先级15、最大系统调用优先级5）
 *     - API功能开关（任务删除、挂起、延时等）
 *     - 中断处理函数映射（SVC_Handler、PendSV_Handler等）
 *
 * 重要参数：
 *     configCPU_CLOCK_HZ:   168MHz (STM32F407系统时钟)
 *     configTICK_RATE_HZ:   1000Hz (1ms Tick周期)
 *     configTOTAL_HEAP_SIZE: 15360字节 (RTOS动态分配总内存)
 *     configMAX_PRIORITIES:  7个优先级 (Idle~High+1)
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * 应用程序特定定义
 *
 * 这些定义应根据具体硬件和应用需求进行调整。
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */   	      
/* 可在本区域添加额外的头文件包含 */
/* USER CODE END Includes */ 

/* 确保stdint只在编译器中使用，汇编器不使用 */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
    #include <stdint.h>
    #include "main.h" 
    extern uint32_t SystemCoreClock;
#endif

/* ============================================================================
 * 调度器配置
 * ============================================================================ */
#define configUSE_PREEMPTION                     1   /* 使用抢占式调度器 */
#define configSUPPORT_STATIC_ALLOCATION          0   /* 不支持静态内存分配 */
#define configSUPPORT_DYNAMIC_ALLOCATION         1   /* 支持动态内存分配 */
#define configUSE_IDLE_HOOK                      0   /* 不使用空闲任务钩子 */
#define configUSE_TICK_HOOK                      0   /* 不使用Tick钩子 */
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )  /* CPU时钟: 168MHz */
#define configTICK_RATE_HZ                       ((TickType_t)1000)   /* Tick频率: 1000Hz (1ms) */
#define configMAX_PRIORITIES                     ( 7 )                /* 最大优先级数: 0~6 */
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)      /* 最小栈大小: 128字(512字节) */
#define configTOTAL_HEAP_SIZE                    ((size_t)15360)      /* 总堆大小: 15KB */
#define configMAX_TASK_NAME_LEN                  ( 16 )               /* 任务名最大长度 */
#define configUSE_16_BIT_TICKS                   0   /* 使用32位Tick计数器 */
#define configUSE_MUTEXES                        1   /* 启用互斥量 */
#define configQUEUE_REGISTRY_SIZE                8   /* 队列注册表大小(用于调试) */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1   /* 使用硬件优化的任务选择 */

/* 协程配置（本系统不使用协程）*/
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* ============================================================================
 * API功能开关 - 设为1启用对应API函数，设为0则排除
 * ============================================================================ */
#define INCLUDE_vTaskPrioritySet            1   /* 启用: 设置任务优先级 */
#define INCLUDE_uxTaskPriorityGet           1   /* 启用: 获取任务优先级 */
#define INCLUDE_vTaskDelete                 1   /* 启用: 删除任务 */
#define INCLUDE_vTaskCleanUpResources       0   /* 禁用: 清理任务资源(不需要) */
#define INCLUDE_vTaskSuspend                1   /* 启用: 挂起任务 */
#define INCLUDE_vTaskDelayUntil             1   /* 启用: 精确延时 */
#define INCLUDE_vTaskDelay                  1   /* 启用: 相对延时 */
#define INCLUDE_xTaskGetSchedulerState      1   /* 启用: 获取调度器状态 */

/* ============================================================================
 * Cortex-M4 中断优先级配置
 * ============================================================================ */
/* Cortex-M使用4位优先级，共16级(0~15)，0为最高 */
#ifdef __NVIC_PRIO_BITS
 /* __BVIC_PRIO_BITS 将在使用CMSIS时定义 */
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4    /* STM32F4使用4位优先级 */
#endif

/* 可使用的最低中断优先级 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

/* 可调用FreeRTOS安全API的最高中断优先级
 * 注意：优先级数值越小，实际优先级越高！
 * 优先级高于此值的中断不能调用FreeRTOS的ISR安全API */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* 内核端口的中断优先级 */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY 绝不能设为零 !!!!
 * 详见 http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* 断言宏：如果条件为假，则关闭所有中断并进入死循环 */
/* USER CODE BEGIN 1 */   
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );} 
/* USER CODE END 1 */

/* ============================================================================
 * FreeRTOS端口中断处理函数与CMSIS标准名称的映射
 * ============================================================================ */
#define vPortSVCHandler    SVC_Handler      /* SVC异常处理 */
#define xPortPendSVHandler PendSV_Handler   /* PendSV异常处理 */

/* 重要：使用STM32Cube固件时，必须注释掉以下定义，
 * 否则会覆盖STM32Cube HAL中定义的SysTick_Handler */
/* #define xPortSysTickHandler SysTick_Handler */

/* USER CODE BEGIN Defines */   	      
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
/* USER CODE END Defines */ 

#endif /* FREERTOS_CONFIG_H */
