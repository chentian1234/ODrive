/*
 * ============================================================================
 * 文件名: freertos.c
 *
 * 文件用途:
 *   本文件实现FreeRTOS实时操作系统的任务与信号量配置，是ODrive固件的多任务
 *   调度核心。负责创建电机控制、通信、命令解析等所有任务，并配置用于任务间
 *   同步的信号量。
 *
 * 主要功能模块：
 *   1. MX_FREERTOS_Init()：创建信号量（USB中断、UART DMA、USB收发）
 *   2. StartDefaultTask()：默认任务函数，初始化所有子系统并创建工作线程
 *
 * 任务优先级（从高到低）:
 *   - osPriorityHigh+1: 电机0控制线程（最高优先级，硬实时）
 *   - osPriorityHigh:   电机1控制线程（高优先级，实时控制）
 *   - osPriorityNormal: 命令解析线程（处理UART/USB命令）
 *   - osPriorityNormal: USB处理线程（处理USB中断与数据收发）
 *   - osPriorityBelowNormal: 数据包定时器线程（后台检查）
 *
 * 信号量说明:
 *   - sem_usb_irq: USB中断信号量（ISR通知处理线程）
 *   - sem_uart_dma: UART DMA互斥信号量
 *   - sem_usb_rx: USB接收同步信号量
 *   - sem_usb_tx: USB发送同步信号量
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "freertos_vars.h"
#include "low_level.h"
#include "commands.h"
/* USER CODE END Includes */

/* 信号量列表 - 用于各模块间的同步与互斥 */

/**
 * @brief USB中断信号量
 * 用途: USB中断服务程序(ISR)通过此信号量通知usb_update_thread线程有USB事件发生
 * 初始状态: 无token(启动时被移除)，确保线程在没有USB中断时不会误触发
 */
osSemaphoreId sem_usb_irq;

/**
 * @brief UART DMA信号量
 * 用途: 保护UART DMA传输的互斥访问，确保同一时间只有一个任务使用UART DMA进行数据收发
 * 初始状态: 无token(启动时被移除)，首次使用前需等待释放
 */
osSemaphoreId sem_uart_dma;

/**
 * @brief USB接收信号量
 * 用途: USB数据接收同步信号量，用于通知usb_update_thread线程有USB数据到达，可以读取
 * 初始状态: 无token(启动时被移除)，确保线程等待实际的USB接收事件
 */
osSemaphoreId sem_usb_rx;

/**
 * @brief USB发送信号量
 * 用途: USB数据发送同步信号量，用于等待USB发送完成，确保发送缓冲区空闲后再进行下一次发送
 * 初始状态: 有1个token，允许第一次发送立即进行
 */
osSemaphoreId sem_usb_tx;

/* 任务/线程ID列表 - 用于引用和管理各个FreeRTOS任务 */

/**
 * @brief 电机0控制线程ID
 * 功能: 运行motor_thread函数，负责电机0的FOC控制、位置/速度/电流闭环控制
 * 优先级: osPriorityHigh+1 (最高)，确保电机控制循环的实时性
 */
osThreadId thread_motor_0;

/**
 * @brief 电机1控制线程ID
 * 功能: 运行motor_thread函数，负责电机1的FOC控制、位置/速度/电流闭环控制
 * 优先级: osPriorityHigh (略低于电机0)，与电机0共享控制逻辑但优先级稍低
 */
osThreadId thread_motor_1;

/**
 * @brief 命令解析线程ID
 * 功能: 运行cmd_parse_thread函数，负责解析来自UART/USB的命令并执行相应操作
 * 优先级: osPriorityNormal (中等)，命令解析不需要严格的实时性
 */
osThreadId thread_cmd_parse;

/**
 * @brief USB处理线程ID
 * 功能: 运行usb_update_thread函数，处理USB中断事件、USB数据收发
 * 优先级: osPriorityNormal (中等)，USB通信需要及时处理但不如电机控制紧急
 */
osThreadId thread_usb_pump;

/**
 * @brief 数据包定时器线程ID
 * 功能: 运行packet_timer_thread函数，定期检查和处理数据包超时、心跳等定时任务
 * 优先级: osPriorityBelowNormal (较低)，定时任务对实时性要求最低
 */
osThreadId task_packet_timer;

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* MISRA C 2004 规则 8.1 */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* 钩子函数原型 */

/**
 * @brief FreeRTOS初始化函数
 * 
 * 本函数负责创建系统中使用的所有信号量，配置它们的初始状态。
 * 
 * 信号量创建过程说明:
 * 1. sem_usb_irq (USB中断信号量):
 *    - 使用osSemaphoreCreate创建最大计数为1的二值信号量
 *    - 创建时默认有1个token，通过osSemaphoreWait(sem_usb_irq, 0)立即移除
 *    - 原因: USB中断信号量必须由USB ISR显式释放来触发，初始有token会导致
 *      usb_update_thread在没有实际USB中断时就被唤醒，可能造成USB枚举失败
 *      或读取到无效数据。移除初始token确保线程只响应真实的USB中断事件。
 * 
 * 2. sem_uart_dma (UART DMA信号量):
 *    - 创建最大计数为1的二值信号量
 *    - 用作互斥锁，保护UART DMA传输资源
 * 
 * 3. sem_usb_rx (USB接收信号量):
 *    - 创建最大计数为1的二值信号量
 *    - 通过osSemaphoreWait移除初始token，确保等待实际接收事件
 * 
 * 4. sem_usb_tx (USB发送信号量):
 *    - 创建最大计数为1的二值信号量
 *    - 保留初始token，允许首次发送立即进行
 * 
 * 注意: 代码中sem_usb_rx部分调用的是osSemaphoreWait(sem_usb_irq, 0)，
 *       这可能是代码笔误，实际应该是osSemaphoreWait(sem_usb_rx, 0)。
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 添加互斥锁 ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  // 初始化USB中断二值信号量，创建后移除初始token
  // 确保信号量仅在USB中断ISR释放时才触发，避免误唤醒
  osSemaphoreDef(sem_usb_irq);
  sem_usb_irq = osSemaphoreCreate(osSemaphore(sem_usb_irq), 1);
  osSemaphoreWait(sem_usb_irq, 0);

  // 创建UART DMA信号量并移除初始token
  // 用作互斥访问UART DMA资源，防止并发冲突
  osSemaphoreDef(sem_uart_dma);
  sem_uart_dma = osSemaphoreCreate(osSemaphore(sem_uart_dma), 1);

  // 创建USB接收信号量并移除初始token
  // 确保线程等待实际的USB数据接收事件
  osSemaphoreDef(sem_usb_rx);
  sem_usb_rx = osSemaphoreCreate(osSemaphore(sem_usb_rx), 1);
  osSemaphoreWait(sem_usb_irq, 0);  // 注: 此处使用sem_usb_irq可能是笔误，应为sem_usb_rx

  // 创建USB发送信号量
  // 保留初始token，允许首次USB发送立即执行
  osSemaphoreDef(sem_usb_tx);
  sem_usb_tx = osSemaphoreCreate(osSemaphore(sem_usb_tx), 1);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 启动定时器，添加新定时器 ... */
  /* USER CODE END RTOS_TIMERS */

  /* 创建默认任务线程 */
  /* defaultTask将以最低优先级(osPriorityIdle)运行，作为系统初始化入口 */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 添加队列 ... */
  /* USER CODE END RTOS_QUEUES */
}

/**
 * @brief 默认任务函数 - 系统主初始化入口
 * 
 * 本任务在FreeRTOS启动后以最低优先级(osPriorityIdle)运行，
 * 负责初始化所有外设和子系统，并创建所有实际工作线程。
 * 
 * 初始化顺序说明:
 * 1. USB设备初始化: 必须在其他通信模块之前完成，因为USB枚举需要时间
 * 2. 通信模块初始化: 设置UART/USB通信通道和协议处理
 * 3. 电机控制初始化: 配置ADC、PWM、编码器等电机相关外设
 * 
 * 任务优先级设置说明 (从高到低):
 * - osPriorityHigh+1: 电机0线程 (最高优先级，确保最严格的实时控制)
 * - osPriorityHigh:   电机1线程 (高优先级，仅次于电机0)
 * - osPriorityNormal: 命令解析线程 (处理用户命令，不需要严格实时)
 * - osPriorityNormal: USB处理线程 (与命令解析同级，USB中断会唤醒)
 * - osPriorityBelowNormal: 数据包定时器线程 (最低工作优先级)
 * 
 * 优先级设计原则:
 * - 电机控制是硬实时任务，必须最高优先级，确保控制周期稳定
 * - 通信和处理任务可以有一定延迟，优先级较低
 * - 定时器任务只负责后台检查，优先级最低
 * 
 * 为什么defaultTask最后要删除自己:
 * - defaultTask仅用于系统初始化，完成所有创建后不再需要
 * - 删除自身释放其占用的256字节栈空间和任务控制块资源
 * - 由于系统已有其他工作线程运行，删除后不影响系统正常工作
 * - 这是嵌入式系统中常见的初始化模式
 */
void StartDefaultTask(void const * argument)
{
  /* USB设备初始化 - 必须在其他通信之前完成 */
  /* USB枚举过程需要主机识别设备，此处初始化USB外设和描述符 */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */

  // 初始化通信模块
  // 设置UART和USB通信通道，注册回调函数，准备接收/发送数据
  init_communication();

  // 初始化电机控制模块
  // 配置ADC采样、PWM输出、编码器接口、电流采样等电机控制相关外设
  init_motor_control();

  // 创建电机0控制线程
  // 优先级: osPriorityHigh+1 (系统中最高优先级)
  // 原因: 电机0通常是主电机，需要最严格的实时控制周期(约8kHz)
  // 栈大小: 512字节，足以容纳FOC控制算法的局部变量
  osThreadDef(task_motor_0, motor_thread,   osPriorityHigh+1, 0, 512);
  thread_motor_0 = osThreadCreate(osThread(task_motor_0), &motors[0]);

  // 创建电机1控制线程
  // 优先级: osPriorityHigh (略低于电机0)
  // 原因: 电机1可以是辅助电机，优先级稍低但仍保证实时性
  osThreadDef(task_motor_1, motor_thread,   osPriorityHigh,   0, 512);
  thread_motor_1 = osThreadCreate(osThread(task_motor_1), &motors[1]);

  // 创建命令解析线程
  // 优先级: osPriorityNormal
  // 功能: 从UART/USB接收命令缓冲区读取数据，解析并执行相应操作
  // 包括: 电机控制命令、参数配置、状态查询等
  osThreadDef(task_cmd_parse, cmd_parse_thread, osPriorityNormal, 0, 512);
  thread_cmd_parse = osThreadCreate(osThread(task_cmd_parse), NULL);

  // 创建USB中断处理线程
  // 优先级: osPriorityNormal
  // 功能: 等待sem_usb_irq信号量(由USB ISR释放)，处理USB数据收发
  // 包括: USB CDC数据接收、发送、设备状态管理
  osThreadDef(task_usb_pump, usb_update_thread, osPriorityNormal, 0, 512);
  thread_usb_pump = osThreadCreate(osThread(task_usb_pump), NULL);
	
  // 创建数据包定时器线程
  // 优先级: osPriorityBelowNormal (所有工作线程中最低)
  // 功能: 定期检查通信数据包超时、发送心跳包、清理过期数据
  // 不需要高优先级，只需周期性运行即可
  osThreadDef(task_packet_timer, packet_timer_thread, osPriorityBelowNormal, 0, 512);
  task_packet_timer = osThreadCreate(osThread(task_packet_timer), NULL);	

  // 初始化任务完成，删除自身释放资源
  // defaultTask的使命已结束，所有工作线程已创建并运行
  // 删除自身可释放256字节栈空间，在资源受限的嵌入式系统中很重要
  vTaskDelete(defaultTaskHandle);

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

