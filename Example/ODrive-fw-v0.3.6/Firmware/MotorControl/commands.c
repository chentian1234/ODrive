/*
 * ============================================================================
 * 文件名: commands.c
 * 
 * 文件用途:
 *   本文件实现了 ODrive 电机控制器的命令解析与通信系统。主要功能包括：
 * 
 *   1. 命令解析协议：
 *      - p: 位置控制 (position control)    - 格式: p <电机编号> <目标位置> <速度前馈> <电流前馈>
 *      - v: 速度控制 (velocity control)    - 格式: v <电机编号> <目标速度> <电流前馈>
 *      - c: 电流控制 (current control)     - 格式: c <电机编号> <目标电流>
 *      - h: 停止所有电机 (halt)            - 格式: h
 *      - i: 打印设备信息 (info)            - 格式: i (返回签名/版本/Flash大小/UUID)
 *      - g: 获取变量值 (get)               - 格式: g <类型> <索引> (类型: 0=float,1=int,2=bool,3=uint16)
 *      - s: 设置变量值 (set)               - 格式: s <类型> <索引> <值>
 *      - m: 配置监控槽位 (monitor setup)   - 格式: m <类型> <索引> <槽位号>
 *      - o: 输出监控数据 (output monitor)  - 格式: o <输出数量>
 *      - t: 启动齿槽转矩补偿校准 (torque)  - 格式: t
 * 
 *   2. 监控槽位机制：
 *      - 系统维护一个 monitoring_slots 数组(最多20个槽位)
 *      - 每个槽位记录一个被监控变量的类型和索引
 *      - 通过 'm' 命令配置槽位，通过 'o' 命令批量读取并打印监控数据
 *      - 适用于高频实时数据采集（如示波器功能）
 * 
 *   3. USB/UART 双通信通道：
 *      - USB: 通过 USB CDC (虚拟串口) 进行通信，使用信号量 sem_usb_tx/rx 同步
 *      - UART: 通过 UART4 + DMA 进行通信，使用信号量 sem_uart_dma 同步
 *      - serial_printf_select 变量记录最近使用的通信接口，用于决定回复输出通道
 * 
 *   4. DMA 循环缓冲区工作原理：
 *      - UART 接收使用 DMA 模式，数据持续写入固定大小的循环缓冲区(dma_circ_buffer)
 *      - 通过读取 DMA 的 NDTR (剩余传输计数) 寄存器计算写入指针位置
 *      - 主循环定期比较写入指针与上次读取位置，提取新到达的数据
 *      - 状态机负责：识别起始符('$') -> 解析命令 -> 识别结束符('\r','\n','!') -> 执行命令
 * 
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */
#include <cmsis_os.h>
#include <commands.h>
#include <usart.h>
#include <gpio.h>
#include <freertos_vars.h>
#include <usbd_cdc.h>
#include <utils.h>

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/

/**
 * @brief 当前活动通信接口选择变量
 * 
 * 该变量自动更新为最近接收到命令的通信接口(USB或UART)。
 * 当命令处理函数需要通过 printf 返回数据时，会根据此变量决定
 * 使用哪个物理通道发送回复数据。
 * 
 * 可选值:
 *   - SERIAL_PRINTF_IS_NONE: 无活动接口，不发送任何回复
 *   - SERIAL_PRINTF_IS_USB:  通过 USB CDC 虚拟串口发送
 *   - SERIAL_PRINTF_IS_UART: 通过 UART4 串口发送
 */
SerialPrintf_t serial_printf_select = SERIAL_PRINTF_IS_NONE;

/* Private constant data -----------------------------------------------------*/

/**
 * @brief GPIO 引脚工作模式配置
 * 
 * 定义 GPIO 1,2 引脚的功能模式，编译时确定，运行时不可更改。
 * 可选模式:
 *   - GPIO_MODE_NONE:       GPIO 1,2 未配置
 *   - GPIO_MODE_UART:       GPIO 1,2 用作 UART 发送(Tx)和接收(Rx)
 *   - GPIO_MODE_STEP_DIR:   GPIO 1,2 用作 M0 电机的步进脉冲(Step)和方向(Dir)信号
 */
// TODO: make command to switch gpio_mode during run-time
//static const GpioMode_t gpio_mode = GPIO_MODE_NONE;     //GPIO 1,2 is not configured
static const GpioMode_t gpio_mode = GPIO_MODE_UART;     //GPIO 1,2 is UART Tx,Rx
// static const GpioMode_t gpio_mode = GPIO_MODE_STEP_DIR; //GPIO 1,2 is M0 Step,Dir

/**
 * @brief USB 接收数据缓冲区指针 (由 set_cmd_buffer 函数设置)
 * 
 * 当 USB CDC 接收到数据包时，CDC_Receive_FS 回调函数会调用 set_cmd_buffer()
 * 将接收到的数据缓冲区地址赋给此变量，供 cmd_parse_thread 线程读取处理。
 */
static uint8_t* usb_buf;

/**
 * @brief USB 接收数据长度 (字节数)
 * 
 * 与 usb_buf 配套使用，记录当前 USB 接收数据包的字节长度。
 */
static uint32_t usb_len;

/// USB 设备句柄 (外部声明，定义在 usbd_cdc 模块中)
extern USBD_HandleTypeDef hUsbDeviceFS;

/**
 * @brief 已接收命令计数器
 * 
 * 每成功接收并解析一条命令，此计数器递增。
 * 用于调试和日志输出，在命令处理前打印 "In [N] :" 标识。
 */
static uint32_t command_received_cnt = 0;

// 通过 USB/串口接口可访问的变量列表，支持 set/get/monitor 操作
// 注意: 此机制即将被弃用 (deprecated)
// 
// 数组中的每个元素是指向实际变量的指针，可通过 'g'(获取) 和 's'(设置) 命令访问
// 注释中 ro = read-only(只读), rw = read-write(可读写)
// 
// 索引 0: 总线电压 (Vbus) - 只读，反映当前供电电压
// 索引 1: 电气弧度每编码器计数 - 已弃用
float* const exposed_floats[] = {
    &vbus_voltage, // ro
    NULL, //&elec_rad_per_enc, // ro
    &motors[0].pos_setpoint, // rw
    &motors[0].pos_gain, // rw
    &motors[0].vel_setpoint, // rw
    &motors[0].vel_gain, // rw
    &motors[0].vel_integrator_gain, // rw
    &motors[0].vel_integrator_current, // rw
    &motors[0].vel_limit, // rw
    &motors[0].current_setpoint, // rw
    &motors[0].calibration_current, // rw
    &motors[0].phase_inductance, // ro
    &motors[0].phase_resistance, // ro
    &motors[0].current_meas.phB, // ro
    &motors[0].current_meas.phC, // ro
    &motors[0].DC_calib.phB, // rw
    &motors[0].DC_calib.phC, // rw
    &motors[0].shunt_conductance, // rw
    &motors[0].phase_current_rev_gain, // rw
    &motors[0].current_control.current_lim, // rw
    &motors[0].current_control.p_gain, // rw
    &motors[0].current_control.i_gain, // rw
    &motors[0].current_control.v_current_control_integral_d, // rw
    &motors[0].current_control.v_current_control_integral_q, // rw
    &motors[0].current_control.Ibus, // ro
    &motors[0].encoder.phase, // ro
    &motors[0].encoder.pll_pos, // rw
    &motors[0].encoder.pll_vel, // rw
    &motors[0].encoder.pll_kp, // rw
    &motors[0].encoder.pll_ki, // rw
    &motors[1].pos_setpoint, // rw
    &motors[1].pos_gain, // rw
    &motors[1].vel_setpoint, // rw
    &motors[1].vel_gain, // rw
    &motors[1].vel_integrator_gain, // rw
    &motors[1].vel_integrator_current, // rw
    &motors[1].vel_limit, // rw
    &motors[1].current_setpoint, // rw
    &motors[1].calibration_current, // rw
    &motors[1].phase_inductance, // ro
    &motors[1].phase_resistance, // ro
    &motors[1].current_meas.phB, // ro
    &motors[1].current_meas.phC, // ro
    &motors[1].DC_calib.phB, // rw
    &motors[1].DC_calib.phC, // rw
    &motors[1].shunt_conductance, // rw
    &motors[1].phase_current_rev_gain, // rw
    &motors[1].current_control.current_lim, // rw
    &motors[1].current_control.p_gain, // rw
    &motors[1].current_control.i_gain, // rw
    &motors[1].current_control.v_current_control_integral_d, // rw
    &motors[1].current_control.v_current_control_integral_q, // rw
    &motors[1].current_control.Ibus, // ro
    &motors[1].encoder.phase, // ro
    &motors[1].encoder.pll_pos, // rw
    &motors[1].encoder.pll_vel, // rw
    &motors[1].encoder.pll_kp, // rw
    &motors[1].encoder.pll_ki, // rw - 电机1 编码器PLL积分增益
};

/**
 * @brief 可通过接口访问的整型变量数组
 * 
 * 数组元素说明 (索引 -> 含义):
 *   电机0:
 *   索引 0: control_mode        - 控制模式 (位置/速度/电流) - 可读写
 *   索引 1: encoder_offset      - 编码器偏移量              - 可读写
 *   索引 2: encoder_state       - 编码器状态                - 只读
 *   索引 3: error               - 错误标志                  - 可读写(清除)
 *   电机1:
 *   索引 4-7: 同上
 */
int* const exposed_ints[] = {
    (int*)&motors[0].control_mode, // rw
    &motors[0].encoder.encoder_offset, // rw
    &motors[0].encoder.encoder_state, // ro
    &motors[0].error, // rw
    (int*)&motors[1].control_mode, // rw
    &motors[1].encoder.encoder_offset, // rw
    &motors[1].encoder.encoder_state, // ro
    &motors[1].error, // rw
};

/**
 * @brief 可通过接口访问的布尔型变量数组
 * 
 * 数组元素说明 (索引 -> 含义):
 *   电机0:
 *   索引 0: thread_ready    - 线程就绪标志     - 只读
 *   索引 1: enable_control  - 使能控制         - 可读写
 *   索引 2: do_calibration  - 启动校准         - 可读写
 *   索引 3: calibration_ok  - 校准成功标志     - 只读
 *   电机1:
 *   索引 4-7: 同上
 */
bool* const exposed_bools[] = {
    &motors[0].thread_ready, // ro - 电机0 线程就绪标志
    &motors[0].enable_control, // rw - 电机0 使能控制
    &motors[0].do_calibration, // rw - 电机0 启动校准
    &motors[0].calibration_ok, // ro - 电机0 校准成功标志
    &motors[1].thread_ready, // ro
    &motors[1].enable_control, // rw
    &motors[1].do_calibration, // rw
    &motors[1].calibration_ok, // ro
};

/**
 * @brief 可通过接口访问的 uint16 型变量数组
 * 
 * 数组元素说明 (索引 -> 含义):
 *   索引 0: control_deadline (电机0)  - 控制周期截止时间( ticks ) - 可读写
 *   索引 1: last_cpu_time (电机0)     - 上次CPU占用时间           - 只读
 *   索引 2: control_deadline (电机1)  - 可读写
 *   索引 3: last_cpu_time (电机1)     - 只读
 */
uint16_t* const exposed_uint16[] = {
    &motors[0].control_deadline, // rw - 电机0 控制周期截止时间
    &motors[0].last_cpu_time, // ro - 电机0 上次CPU占用时间
    &motors[1].control_deadline, // rw
    &motors[1].last_cpu_time, // ro
};

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 监控槽位数组
 * 
 * 监控槽位机制说明:
 *   - 本数组包含 20 个 monitoring_slot 结构体，每个槽位可绑定一个被监控变量
 *   - 每个槽位包含两个字段:
 *     * type:  变量类型 (0=float, 1=int, 2=bool, 3=uint16)
 *     * index: 在对应 exposed_* 数组中的索引
 *   - 使用流程:
 *     1. 通过 'm' 命令配置槽位: m <type> <index> <slot_number>
 *     2. 通过 'o' 命令输出监控数据: o <number_of_slots>
 *        系统会依次读取 slot 0 到 slot N-1 的值，以制表符分隔打印
 *   - 应用场景: 实时数据采集，类似于简易示波器功能
 */
monitoring_slot monitoring_slots[20] = {0};
/* Private function prototypes -----------------------------------------------*/
//static void print_monitoring(int limit);

#if defined ARM_PRINTF

#include <usart.h>
#include <usbd_cdc_if.h>

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/// UART 发送缓冲区大小 (字节)
#define UART_TX_BUFFER_SIZE 1024

/// UART 发送静态缓冲区，用于 DMA 传输
static uint8_t uart_tx_buf[UART_TX_BUFFER_SIZE];

/**
 * @brief 将数据写入当前活动的通信接口
 * 
 * 根据 serial_printf_select 的值，将数据通过 USB 或 UART 发送出去。
 * 
 * @param data 指向要发送的数据的指针
 * @param len  要发送的数据长度 (字节)
 * 
 * @return int 成功写入的字节数。如果接口不可用或超时，返回 0
 * 
 * 实现原理:
 *   - USB 通道: 等待 sem_usb_tx 信号量 (100ms超时)，然后调用 CDC_Transmit_FS 发送
 *   - UART 通道: 等待 sem_uart_dma 信号量 (100ms超时)，先将数据拷贝到 uart_tx_buf，
 *               再调用 HAL_UART_Transmit_DMA 启动 DMA 后台传输
 *   - 信号量机制: 发送开始时获取信号量，发送完成中断回调中释放信号量
 *               防止在传输过程中被其他任务打断
 */
int commands_write(char* data, int len) {
    // 实际成功写入的字节数
    int written = 0;
    switch (serial_printf_select) {
        case SERIAL_PRINTF_IS_USB: {
            // 等待 USB 发送信号量，确保接口可用
            // 注意: USB 驱动会在发送完成时释放该信号量
            const uint32_t usb_tx_timeout = 100; // ms
            osStatus sem_stat = osSemaphoreWait(sem_usb_tx, usb_tx_timeout);
            if (sem_stat == osOK) {
                uint8_t status = CDC_Transmit_FS((uint8_t*)data, len);  // 通过 CDC 端点发送
                written = (status == USBD_OK) ? len : 0;
            } // 如果信号量超时，written 保持为 0
        }
        break;

        case SERIAL_PRINTF_IS_UART: {
            // 检查数据长度是否超过缓冲区
            if (len > UART_TX_BUFFER_SIZE)
                return 0;
            // 等待 UART DMA 发送信号量，确保接口可用
            // 注意: HAL_UART_TxCpltCallback 会在发送完成时释放该信号量
            const uint32_t uart_tx_timeout = 100; // ms
            osStatus sem_stat = osSemaphoreWait(sem_uart_dma, uart_tx_timeout);
            if (sem_stat == osOK) {
                memcpy(uart_tx_buf, data, len);                    // 将数据拷贝到 UART 发送缓冲区
                HAL_UART_Transmit_DMA(&huart4, uart_tx_buf, len);  // 启动 DMA 后台发送
            } // 如果信号量超时，written 保持为 0
        }
        break;

        default: {
            written = 0;
        }
        break;
    }

    return written;
}

/**
 * @brief 格式化打印函数，类似于标准 printf
 * 
 * 将格式化字符串通过当前活动的通信接口 (USB 或 UART) 发送出去。
 * 
 * @param format 格式化字符串 (同标准 printf 格式)
 * @param ...    可变参数列表
 * 
 * 实现原理:
 *   - 使用静态缓冲区 print_buffer[1023] 存储格式化后的字符串
 *   - 调用 vsnprintf 进行格式化，限制最大长度防止缓冲区溢出
 *   - 通过 commands_write 将格式化后的数据发送出去
 *   - 发送时包含字符串结束符 (\0)
 */
void cmd_printf(const char* format, ...) {
    va_list arg;
    va_start(arg, format);
    int len;
    static char print_buffer[1023];

    len = vsnprintf(print_buffer, 1023, format, arg);
    va_end(arg);

    if (len > 0) {
        commands_write((char*)print_buffer, (len<1022) ? len+1 : 1023);
    }
}

#if defined ARM_TERMINAL

/// 自定义命令回调最大注册数量
#define CALLBACK_LEN 32

/**
 * @brief 终端命令回调结构体
 * 
 * 用于注册自定义命令。当收到无法被内置命令匹配的输入时，
 * 会遍历此数组查找注册的回调函数。
 */
typedef struct _terminal_callback_struct {
    const char *command;                          ///< 命令名称字符串
    const char *help;                             ///< 帮助文本 (可为 NULL)
    const char *arg_names;                        ///< 参数名称说明，例如 "[arg_a] [arg_b]" (可为 NULL)
    void(*cbf)(int argc, const char **argv);     ///< 回调函数指针，argc为参数数量，argv为参数数组
} terminal_callback_struct;

/// 命令回调注册表，最多容纳 CALLBACK_LEN (32) 个回调
static terminal_callback_struct callbacks[CALLBACK_LEN];

/// 下一个空闲回调槽位索引 (也表示已注册的回调数量)
static int callback_write = 0;

/**
 * @brief 处理以空格分隔的命令字符串 (终端模式)
 * 
 * 本函数解析以空格分隔的命令字符串，根据第一个参数(命令名)执行相应操作。
 * 仅在 ARM_TERMINAL 宏定义时启用。
 * 
 * @param buffer           指向命令字符串的缓冲区
 * @param len              命令字符串长度
 * @param response_interface 响应输出接口 (USB 或 UART)
 * 
 * 支持的命令格式 (以空格分隔):
 *   p <电机号> <目标位置> <速度前馈> <电流前馈>  - 位置控制
 *   v <电机号> <目标速度> <电流前馈>             - 速度控制
 *   c <电机号> <目标电流>                       - 电流控制
 *   i                                          - 打印设备信息
 *   g <类型> <索引>                             - 获取变量值 (类型: 0/1/2/3 或 f/i/b/u)
 *   get <类型字符> <索引>                       - 获取变量值 (可读性更好的版本)
 *   h                                          - 停止所有电机
 *   s <类型> <索引> <值>                        - 设置变量值
 *   set <类型字符> <索引> <值>                  - 设置变量值 (可读性更好的版本)
 *   m <类型> <索引> <槽位号>                    - 配置监控槽位
 *   o <输出数量>                               - 输出监控数据
 *   t                                          - 启动齿槽转矩补偿校准
 * 
 * 实现原理:
 *   1. 设置响应接口
 *   2. 在缓冲区末尾添加字符串结束符
 *   3. 使用 strtok 按空格分割参数，构建 argv 数组
 *   4. 根据 argv[0] 的命令名，用 if-else 链匹配并执行相应操作
 *   5. 如果没有匹配的内置命令，则遍历 callbacks 数组查找自定义回调
 */
void commands_process_string(uint8_t* buffer, int len, SerialPrintf_t response_interface) {
    // 设置响应输出接口
    serial_printf_select = response_interface;

    // TODO 这里用了一种不太优雅的方式在缓冲区末尾添加结束符:
    // 未来应该使用更合理的结构体打包方式替代 sscanf
    if (len) {
        buffer[len-1] = 0;
    }

    // 解析命令行参数，最多支持 64 个参数
    enum { kMaxArgs = 64 };
    int argc = 0;
    char *argv[kMaxArgs];

    char *str;
    buffer[len] = '\0';
    str = (char*)buffer;

    // 使用 strtok 按空格分割字符串
    char *p2 = strtok(str, " ");
    while (p2 && argc < kMaxArgs) {
        argv[argc++] = p2;
        p2 = strtok(0, " ");
    }

    if (argc == 0) {
        return;
    }

    // 检查命令类型
    if (strcmp(argv[0], "p") == 0) {
        // 位置控制: p <电机号> <目标位置> <速度前馈> <电流前馈>
        unsigned motor_number;
        float pos_setpoint, vel_feed_forward, current_feed_forward;

        sscanf(argv[1], "%u", &motor_number);
        if (argc == 5 && motor_number < num_motors) {
            sscanf(argv[2], "%f", &pos_setpoint);
            sscanf(argv[3], "%f", &vel_feed_forward);
            sscanf(argv[4], "%f", &current_feed_forward);
            set_pos_setpoint(&motors[motor_number], pos_setpoint, vel_feed_forward, current_feed_forward);
        } else {
            cmd_printf("This command requires 5 argument.\n");
        }
    } else if (strcmp(argv[0], "v") == 0) {
        // 速度控制: v <电机号> <目标速度> <电流前馈>
        unsigned motor_number;
        float vel_feed_forward, current_feed_forward;
        sscanf(argv[1], "%u", &motor_number);
        if (argc == 4 && motor_number < num_motors) {
            sscanf(argv[2], "%f", &vel_feed_forward);
            sscanf(argv[3], "%f", &current_feed_forward);
            set_vel_setpoint(&motors[motor_number], vel_feed_forward, current_feed_forward);
        } else {
            cmd_printf("This command requires 4 argument.\n");
        }
    } else if (strcmp(argv[0], "c") == 0) {
        // 电流控制: c <电机号> <目标电流>
        unsigned motor_number;
        float current_feed_forward;
        sscanf(argv[1], "%u", &motor_number);
        if (argc == 3 && motor_number < num_motors) {
            sscanf(argv[2], "%f", &current_feed_forward);
            set_current_setpoint(&motors[motor_number], current_feed_forward);
        } else {
            cmd_printf("This command requires 3 argument.\n");
        }
    } else if (strcmp(argv[0], "i") == 0) { // 打印设备信息
        // 读取设备签名、修订版本、Flash 大小和 UUID
        cmd_printf("Signature: %#x\n", STM_ID_GetSignature());
        cmd_printf("Revision: %#x\n", STM_ID_GetRevision());
        cmd_printf("Flash Size: %#x KiB\n", STM_ID_GetFlashSize());
        cmd_printf("UUID: 0x%lx%lx%lx\n", STM_ID_GetUUID(2), STM_ID_GetUUID(1), STM_ID_GetUUID(0));
    } else if (strcmp(argv[0], "g") == 0) { // 获取变量值 (数字类型编码版本)
        // 格式: g <0:float,1:int,2=bool,3:uint16> <索引>
        int type = 0;
        int index = 0;
        if (argc == 3) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 0: { // float
                    cmd_printf("%f\n",*exposed_floats[index]);
                    break;
                };
                case 1: { // int
                    cmd_printf("%d\n",*exposed_ints[index]);
                    break;
                };
                case 2: { // bool
                    cmd_printf("%d\n",*exposed_bools[index]);
                    break;
                };
                case 3: { // uint16
                    cmd_printf("%hu\n",*exposed_uint16[index]);
                    break;
                };
            }
        } else {
            cmd_printf("This command requires 3 argument.\n");
        }
    } else if (strcmp(argv[0], "get") == 0) { // 获取变量值 (字符类型编码版本，更易读)
        // 格式: get <f:float,i:int,b:bool,u:uint16> <索引>
        char type;
        int index = 0;
        if (argc == 3) {
            sscanf(argv[1], "%s", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 'f': {
                    cmd_printf("%f\n",*exposed_floats[index]);
                    break;
                };
                case 'i': {
                    cmd_printf("%d\n",*exposed_ints[index]);
                    break;
                };
                case 'b': {
                    cmd_printf("%d\n",*exposed_bools[index]);
                    break;
                };
                case 'u': {
                    cmd_printf("%hu\n",*exposed_uint16[index]);
                    break;
                };
            }
        } else {
            cmd_printf("This command requires 3 argument.\n");
        }
    } else if (strcmp(argv[0], "h") == 0) { // 停止 (HALT) - 将所有电机速度设为 0
        for (int i = 0; i < num_motors; i++) {
            set_vel_setpoint(&motors[i], 0.0f, 0.0f);
        }
    } else if (strcmp(argv[0], "s") == 0) { // 设置变量值 (数字类型编码版本)
        // 格式: s <0:float,1:int,2:bool,3:uint16> <索引> <值>
        int type = 0;
        int index = 0;
        if (argc == 4) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 0: { // float
                    sscanf(argv[3], "%f", exposed_floats[index]);
                    break;
                };
                case 1: { // int
                    sscanf(argv[3], "%d", exposed_ints[index]);
                    break;
                };
                case 2: { // bool
                    int btmp = 0;
                    sscanf(argv[3], "%d", &btmp);
                    *exposed_bools[index] = btmp ? true : false;
                    break;
                };
                case 3: { // uint16
                    sscanf(argv[3], "%hu", exposed_uint16[index]);
                    break;
                };
            }
        } else {
            cmd_printf("This command requires 4 argument.\n");
        }
    } else if (strcmp(argv[0], "set") == 0) { // 设置变量值 (字符类型编码版本，更易读)
        // 格式: set <f:float,i:int,b:bool,u:uint16> <索引> <值>
        int type = 0;
        int index = 0;
        if (argc == 4) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 'f': {
                    sscanf(argv[3], "%f", exposed_floats[index]);
                    break;
                };
                case 'i': {
                    sscanf(argv[3], "%d", exposed_ints[index]);
                    break;
                };
                case 'b': {
                    int btmp = 0;
                    sscanf(argv[3], "%d", &btmp);
                    *exposed_bools[index] = btmp ? true : false;
                    break;
                };
                case 'u': {
                    sscanf(argv[3], "%hu", exposed_uint16[index]);
                    break;
                };
            }
        } else {
            cmd_printf("This command requires 4 argument.\n");
        }
    } else if (strcmp(argv[0], "m") == 0) { // 配置监控槽位 (Setup Monitor)
        // 格式: m <0:float,1:int,2:bool,3:uint16> <索引> <槽位号>
        int type = 0;
        int index = 0;
        int slot = 0;
        if (argc == 4) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            sscanf(argv[3], "%u", &slot);
            monitoring_slots[slot].type = type;     // 设置槽位的变量类型
            monitoring_slots[slot].index = index;   // 设置槽位的变量索引
        } else {
            cmd_printf("This command requires 4 argument.\n");
        }
    } else if (strcmp(argv[0], "o") == 0) { // 输出监控数据 (Output Monitor)
        int limit = 0;
        if (argc == 2) {
            sscanf(argv[1], "%u", &limit);
            print_monitoring(limit);
        } else {
            cmd_printf("This command requires 2 argument.\n");
        }
    } else if (strcmp(argv[0], "t") == 0) { // 启动齿槽转矩补偿校准 (Anti-Cogging Calibration)
        for (int i = 0; i < num_motors; i++) {
            // 确保齿槽补偿映射已正确分配且电机无错误
            if (motors[i].anticogging.cogging_map != NULL && motors[i].error == ERROR_NO_ERROR) {
                motors[i].anticogging.calib_anticogging = true;
            }
        }
    } else {
        // 以上均不匹配时，尝试查找自定义注册的命令回调
        int i;
        bool found = false;
        for (i = 0; i < callback_write; i++) {
            if (strcmp(argv[0], callbacks[i].command) == 0) {
                callbacks[i].cbf(argc, (const char**)argv);
                found = true;
                break;
            }
        }

        if (found == false) {
            return;
        }
    }
}

/**
 * @brief 注册自定义命令回调到终端
 * 
 * 如果命令已注册，则旧的回调将被替换。
 * 
 * @param command   命令名称字符串
 * @param help      命令帮助文本，可为 NULL
 * @param arg_names 参数名称说明，例如 "[arg_a] [arg_b]"，可为 NULL
 * @param cbf       命令回调函数指针
 * 
 * 实现原理:
 *   1. 先遍历 callbacks 数组查找是否已存在相同命令 (通过地址和字符串两种比较方式)
 *   2. 如果找到则更新该位置的回调，否则在下一个空闲槽位注册
 *   3. callback_write 始终指向下一个空闲槽位，当达到最大数量时回绕到 0
 */
void commands_register_command_callback(
    const char* command,
    const char *help,
    const char *arg_names,
    void(*cbf)(int argc, const char **argv)) {

    int callback_num = callback_write;

    for (int i = 0; i < callback_write; i++) {
        // 首先通过地址比较，避免同一回调被重复注册
        if (callbacks[i].command == command) {
            callback_num = i;
            break;
        }

        // 通过字符串比较查找同名命令
        if (strcmp(callbacks[i].command, command) == 0) {
            callback_num = i;
            break;
        }
    }

    callbacks[callback_num].command = command;
    callbacks[callback_num].help = help;
    callbacks[callback_num].arg_names = arg_names;
    callbacks[callback_num].cbf = cbf;

    // 如果是新注册的命令，更新下一个空闲槽位索引
    if (callback_num == callback_write) {
        callback_write++;
        if (callback_write >= CALLBACK_LEN) {
            callback_write = 0;  // 回绕到开头
        }
    }
}
#endif
#endif

/**
 * ============================================================================
 * UART 命令处理模块 (仅在定义 ARM_COMMMAND_UART 时启用)
 * 
 * 本模块使用基于数据包的协议进行通信，与上面的终端模式(基于文本)不同。
 * 数据通过 packet 层进行编码/解码，支持校验和等功能。
 * ============================================================================
 */
#if defined ARM_COMMMAND_UART

#include "packet.h"
#include "commands_pro.h"

// 私有函数声明
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet_wrapper(unsigned char *data, unsigned int len);
static void send_packet(unsigned char *data, unsigned int len);
static void init_packet(void);

/**
 * @brief 处理接收到的数据包
 * 
 * @param data 数据包内容
 * @param len  数据包长度
 */
static void process_packet(unsigned char *data, unsigned int len) {
    commands_set_send_func(send_packet_wrapper);
    commands_process_packet(data, len);
}

/**
 * @brief 数据包发送包装函数 (适配 packet 层接口)
 */
static void send_packet_wrapper(unsigned char *data, unsigned int len) {
    packet_send_packet(data, len, PACKET_HANDLER);
}

/**
 * @brief 实际的数据发送函数
 */
static void send_packet(unsigned char *data, unsigned int len) {
    commands_write((char *)data, len);
}

/**
 * @brief 初始化 packet 处理层
 */
static void init_packet(void) {
    packet_init(send_packet, process_packet, PACKET_HANDLER);
}
#endif


/* 函数实现 ------------------------------------------------------------------*/

/**
 * @brief 初始化通信接口
 * 
 * 根据 gpio_mode 的配置，初始化相应的 GPIO 功能和通信协议层。
 * 
 * 配置选项:
 *   - GPIO_MODE_NONE:       不执行任何初始化
 *   - GPIO_MODE_UART:       将 GPIO 1,2 配置为 UART Tx/Rx，
 *                           如果定义了 ARM_COMMMAND_UART 则初始化 packet 协议层
 *   - GPIO_MODE_STEP_DIR:   将 GPIO 1,2 配置为步进脉冲/方向信号 (仅用于电机控制)
 */
void init_communication() {
    switch (gpio_mode) {
        case GPIO_MODE_NONE:
            break; // 不执行任何操作
        case GPIO_MODE_UART: {
#if defined ARM_COMMMAND_UART
            init_packet();   // 初始化 UART 数据包协议层
#endif
            SetGPIO12toUART();  // 将 GPIO 1,2 配置为 UART 功能
        }
        break;
        case GPIO_MODE_STEP_DIR: {
            SetGPIO12toStepDir();  // 将 GPIO 1,2 配置为 Step/Dir 功能
        }
        break;
        default:
            // TODO: 报告错误 - 遇到未知模式
            break;
    }
}

/**
 * @brief 解析电机控制命令 (精简模式)
 * 
 * 本函数是 commands_process_string 的精简版本，不使用 strtok 分割参数，
 * 而是直接通过 sscanf 从原始缓冲区长匹配格式字符串来解析命令。
 * 仅在未定义 ARM_TERMINAL 时使用。
 * 
 * @param buffer           指向命令字符串的缓冲区 (不含起始符 '$')
 * @param len              命令字符串长度
 * @param response_interface 响应输出接口 (USB 或 UART)
 * 
 * 支持的命令格式:
 *   p <电机号> <目标位置> <速度前馈> <电流前馈>  - 位置控制
 *   v <电机号> <目标速度> <电流前馈>             - 速度控制
 *   c <电机号> <目标电流>                       - 电流控制
 *   i                                          - 打印设备信息
 *   g <类型> <索引>                             - 获取变量值 (类型: 0=float,1=int,2=bool,3=uint16)
 *   h                                          - 停止所有电机
 *   s <类型> <索引> <值>                        - 设置变量值
 *   m <类型> <索引> <槽位号>                    - 配置监控槽位
 *   o <输出数量>                               - 输出监控数据
 *   t                                          - 启动齿槽转矩补偿校准
 * 
 * 与 commands_process_string 的区别:
 *   - 本函数直接从完整缓冲区匹配格式 (如 "p %u %f %f %f")
 *   - commands_process_string 先用 strtok 分割成 argv 数组，再逐个解析
 *   - 本函数更紧凑，适合资源受限的场景
 */
void motor_parse_cmd(uint8_t* buffer, int len, SerialPrintf_t response_interface) {
    // 设置响应输出接口
    serial_printf_select = response_interface;

    // TODO 这里用了一种不太优雅的方式在缓冲区末尾添加结束符:
    // 未来应该使用更合理的结构体打包方式替代 sscanf
    buffer[len-1] = 0;
//	  ((uint8_t *)buffer)[len < buffer_capacity ? len : (buffer_capacity - 1)] = 0;

    // 检查命令类型
    if (buffer[0] == 'p') {
        // 位置控制: p <电机号> <目标位置> <速度前馈> <电流前馈>
        unsigned motor_number;
        float pos_setpoint, vel_feed_forward, current_feed_forward;
        int numscan = sscanf((const char*)buffer, "p %u %f %f %f", &motor_number, &pos_setpoint, &vel_feed_forward, &current_feed_forward);
        if (numscan == 4 && motor_number < num_motors) {
            set_pos_setpoint(&motors[motor_number], pos_setpoint, vel_feed_forward, current_feed_forward);
        }
    } else if (buffer[0] == 'v') {
        // 速度控制: v <电机号> <目标速度> <电流前馈>
        unsigned motor_number;
        float vel_feed_forward, current_feed_forward;
        int numscan = sscanf((const char*)buffer, "v %u %f %f", &motor_number, &vel_feed_forward, &current_feed_forward);
        if (numscan == 3 && motor_number < num_motors) {
            set_vel_setpoint(&motors[motor_number], vel_feed_forward, current_feed_forward);
        }
    } else if (buffer[0] == 'c') {
        // 电流控制: c <电机号> <目标电流>
        unsigned motor_number;
        float current_feed_forward;
        int numscan = sscanf((const char*)buffer, "c %u %f", &motor_number, &current_feed_forward);
        if (numscan == 2 && motor_number < num_motors) {
            set_current_setpoint(&motors[motor_number], current_feed_forward);
        }
    } else if (buffer[0] == 'i') { // 打印设备信息
        // 读取设备签名、修订版本、Flash 大小和 UUID
        cmd_printf("Signature: %#x\n", STM_ID_GetSignature());
        cmd_printf("Revision: %#x\n", STM_ID_GetRevision());
        cmd_printf("Flash Size: %#x KiB\n", STM_ID_GetFlashSize());
        cmd_printf("UUID: 0x%lx%lx%lx\n", STM_ID_GetUUID(2), STM_ID_GetUUID(1), STM_ID_GetUUID(0));
    } else if (buffer[0] == 'g') { // 获取变量值
        // 格式: g <0:float,1:int,2:bool,3:uint16> <索引>
        int type = 0;
        int index = 0;
        int numscan = sscanf((const char*)buffer, "g %u %u", &type, &index);
        if (numscan == 2) {
            switch (type) {
                case 0: { // float
                    cmd_printf("%f\n",*exposed_floats[index]);
                    break;
                };
                case 1: { // int
                    cmd_printf("%d\n",*exposed_ints[index]);
                    break;
                };
                case 2: { // bool
                    cmd_printf("%d\n",*exposed_bools[index]);
                    break;
                };
                case 3: { // uint16
                    cmd_printf("%hu\n",*exposed_uint16[index]);
                    break;
                };
            }
        }
    } else if (buffer[0] == 'h') { // 停止 (HALT) - 将所有电机速度设为 0
        for (int i = 0; i < num_motors; i++) {
            set_vel_setpoint(&motors[i], 0.0f, 0.0f);
        }
    } else if (buffer[0] == 's') { // 设置变量值
        // 格式: s <0:float,1:int,2:bool,3:uint16> <索引> <值>
        int type = 0;
        int index = 0;
        int numscan = sscanf((const char*)buffer, "s %u %u", &type, &index);
        if (numscan == 2) {
            switch (type) {
                case 0: { // float
                    sscanf((const char*)buffer, "s %u %u %f", &type, &index, exposed_floats[index]);
                    break;
                };
                case 1: { // int
                    sscanf((const char*)buffer, "s %u %u %d", &type, &index, exposed_ints[index]);
                    break;
                };
                case 2: { // bool
                    int btmp = 0;
                    sscanf((const char*)buffer, "s %u %u %d", &type, &index, &btmp);
                    *exposed_bools[index] = btmp ? true : false;
                    break;
                };
                case 3: { // uint16
                    sscanf((const char*)buffer, "s %u %u %hu", &type, &index, exposed_uint16[index]);
                    break;
                };
            }
        }
    } else if (buffer[0] == 'm') { // 配置监控槽位
        // 格式: m <0:float,1:int,2:bool,3:uint16> <索引> <槽位号>
        int type = 0;
        int index = 0;
        int slot = 0;
        int numscan = sscanf((const char*)buffer, "m %u %u %u", &type, &index, &slot);
        if (numscan == 3) {
            monitoring_slots[slot].type = type;     // 设置槽位的变量类型
            monitoring_slots[slot].index = index;   // 设置槽位的变量索引
        }
    } else if (buffer[0] == 'o') { // 输出监控数据
        int limit = 0;
        int numscan = sscanf((const char*)buffer, "o %u", &limit);
        if (numscan == 1) {
            print_monitoring(limit);
        }
    } else if (buffer[0] == 't') { // 启动齿槽转矩补偿校准
        for (int i = 0; i < num_motors; i++) {
            // 确保齿槽补偿映射已正确分配且电机无错误
            if (motors[i].anticogging.cogging_map != NULL && motors[i].error == ERROR_NO_ERROR) {
                motors[i].anticogging.calib_anticogging = true;
            }
        }
    }
}

/**
 * @brief 打印监控数据
 * 
 * 依次读取指定数量的监控槽位，获取对应的变量值并以制表符分隔打印。
 * 
 * @param limit 要输出的监控槽位数量 (从 slot 0 开始)
 * 
 * 输出格式: 每个值之间用制表符(\t)分隔，最后以换行符(\n)结尾
 *          例如: "3.141592\t128\t1\t4096\t\n"
 * 
 * 实现原理:
 *   - 遍历 slot 0 到 slot limit-1
 *   - 根据 slot.type 确定变量类型，再从对应的 exposed_* 数组中取出值
 *   - 如果遇到无效的 type 值，设置 i=100 强制退出循环 (异常处理)
 */
void print_monitoring(int limit) {
    for (int i=0; i<limit; i++) {
        switch (monitoring_slots[i].type) {
            case 0: // float
                cmd_printf("%f\t",*exposed_floats[monitoring_slots[i].index]);
                break;
            case 1: // int
                cmd_printf("%d\t",*exposed_ints[monitoring_slots[i].index]);
                break;
            case 2: // bool
                cmd_printf("%d\t",*exposed_bools[monitoring_slots[i].index]);
                break;
            case 3: // uint16
                cmd_printf("%hu\t",*exposed_uint16[monitoring_slots[i].index]);
                break;
            default:
                i=100;  // 遇到无效类型，强制退出循环
        }
    }
    cmd_printf("\n");
}

/**
 * @brief 命令解析主线程
 * 
 * 这是通信系统的核心线程，负责:
 *   1. 从 UART DMA 循环缓冲区中读取数据并解析命令
 *   2. 处理 USB CDC 接收到的数据
 * 
 * @param argument 线程参数 (未使用)
 * 
 * DMA 循环缓冲区工作原理:
 *   ┌─────────────────────────────────────────┐
 *   │  dma_circ_buffer (64 字节)              │
 *   │  DMA 持续将 UART 接收的数据写入此缓冲区  │
 *   │  写入位置由 DMA 的 NDTR 寄存器间接指示    │
 *   │                                         │
 *   │  rcv_idx (写指针) = BUFFER_SIZE - NDTR   │
 *   │  last_rcv_idx (读指针) 由本线程维护      │
 *   │                                         │
 *   │  当写指针 != 读指针时，说明有新数据到达    │
 *   └─────────────────────────────────────────┘
 * 
 * 命令解析状态机:
 *   IDLE (空闲) --收到'$'--> ACTIVE (活动) --收到'\r'/'\n'/'!'--> 执行命令 --> IDLE
 *   ACTIVE 状态下收到的字符被逐字节存入 parse_buffer
 *   如果 parse_buffer 满仍未收到结束符，则丢弃并重置状态机
 * 
 * USB 处理机制:
 *   在处理完 UART 数据后，等待最多 1ms 检查是否有 USB 数据待处理
 *   如果有 USB 数据 (sem_usb_rx 信号量就绪)，则调用相应的解析函数
 *   然后调用 USBD_CDC_ReceivePacket 允许接收下一个 USB 数据包
 * 
 * 注意: 本线程永不退出，循环运行直到任务被删除
 */
void cmd_parse_thread(void const * argument) {

    /// UART 接收 DMA 循环缓冲区大小 (字节)
#define UART_RX_BUFFER_SIZE 64

    /// DMA 循环缓冲区 - DMA 硬件持续将 UART 接收到的数据写入此缓冲区
    static uint8_t dma_circ_buffer[UART_RX_BUFFER_SIZE];

    /// 命令解析缓冲区 - 用于临时存储一条完整命令
    static uint8_t parse_buffer[UART_RX_BUFFER_SIZE];

    // 配置 UART 使用 DMA 在循环缓冲区中持续接收数据
    // 不使用中断获取数据，而是通过定期轮询 DMA 状态来读取数据
    HAL_UART_Receive_DMA(&huart4, dma_circ_buffer, sizeof(dma_circ_buffer));

    // 计算 DMA 当前写入位置 (读指针初始位置)
    // NDTR = DMA 剩余传输计数, 当 DMA 写满整个缓冲区时会回绕
    uint32_t last_rcv_idx = UART_RX_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;

    // 状态机主循环 - 永久运行
    for (;;) {
        // 初始化接收状态机
        bool reset_read_state = false;  // 是否重置状态机
        bool read_active = false;        // 是否处于命令读取状态 (已收到起始符 '$')
        uint32_t parse_buffer_idx = 0;   // parse_buffer 中的当前写入位置

        // 运行状态机直到需要重置
        do {
            // 检查 UART 错误，如果发生错误则重启 DMA 接收
            if (huart4.ErrorCode != HAL_UART_ERROR_NONE) {
                HAL_UART_AbortReceive(&huart4);
                HAL_UART_Receive_DMA(&huart4, dma_circ_buffer, sizeof(dma_circ_buffer));
                break; // 重置状态机
            }

            // 获取 DMA 循环缓冲区的 "写指针" 位置 (DMA 下一次将写入的位置)
            uint32_t rcv_idx = UART_RX_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;

            // 当有新数据到达时 (写指针 != 读指针)，逐字节处理
            // 在休眠期间可能积累了多个字符，所以持续处理直到追上最新数据
            while (rcv_idx != last_rcv_idx) {
                // 从循环缓冲区中取出下一个字符，并更新读指针
                uint8_t c = dma_circ_buffer[last_rcv_idx];

#if defined ARM_COMMMAND_UART
                // 数据包模式: 直接将字节送入 packet 层处理
                serial_printf_select = SERIAL_PRINTF_IS_UART;
                packet_process_byte(c, PACKET_HANDLER);
#else
                // 文本模式: 回显字符 (UART 回显，方便调试)
                {
                    // 等待 UART 发送信号量，确保接口可用
                    // 注意: HAL_UART_TxCpltCallback 会在发送完成时释放信号量
                    const uint32_t uart_tx_timeout = 100; // ms
                    osStatus sem_stat = osSemaphoreWait(sem_uart_dma, uart_tx_timeout);
                    if (sem_stat == osOK) {
                        HAL_UART_Transmit_DMA(&huart4, &c, 1);  // 启动 DMA 后台发送 (回显单个字符)
                    } // 如果信号量超时，则跳过此次回显
                }
#endif
                // 更新读指针 (循环递增)
                if (++last_rcv_idx == UART_RX_BUFFER_SIZE)
                    last_rcv_idx = 0;

                // 查找起始字符 '$'
                if (c == '$') {
                    read_active = true;   // 进入命令读取状态
                    continue;             // 不记录起始符本身
                }

                // 在命令读取状态下，将字符存入解析缓冲区
                if (read_active) {
                    parse_buffer[parse_buffer_idx++] = c;

                    // 检查是否为命令结束符
                    if (c == '\r' || c == '\n' || c == '!') {
                        // 命令字符串结束: 将结束符替换为字符串结束符 \0
                        parse_buffer[parse_buffer_idx-1] = '\0';
                        cmd_printf("In [%u] : \n", command_received_cnt++);

#if defined ARM_TERMINAL
                        // 终端模式: 使用空格分隔的参数解析
                        commands_process_string(parse_buffer, parse_buffer_idx, SERIAL_PRINTF_IS_UART);
#else
                        // 精简模式: 使用 sscanf 直接匹配
                        motor_parse_cmd(parse_buffer, parse_buffer_idx, SERIAL_PRINTF_IS_UART);
#endif
                        // 重置接收状态机，等待下一条命令
                        reset_read_state = true;
                        break;
                    } else if (parse_buffer_idx == UART_RX_BUFFER_SIZE - 1) {
                        // 解析缓冲区即将溢出 (最后一个位置保留给 \0)
                        // 放弃当前命令，重置状态机
                        reset_read_state = true;
                        break;
                    }
                }
            }
            // 当执行到这里时，说明 UART 缓冲区中没有新数据需要处理了
            // 接下来检查是否有 USB 数据待处理: 等待最多 1ms
            // 如果有 USB 数据则处理，否则超时后返回继续检查 UART
            const uint32_t usb_check_timeout = 1; // ms
            osStatus sem_stat = osSemaphoreWait(sem_usb_rx, usb_check_timeout);
            if (sem_stat == osOK) {
                // USB 数据到达，打印接收日志
                cmd_printf("In [%u] : \n", command_received_cnt++);
#if defined ARM_TERMINAL
                // 终端模式: 使用空格分隔的参数解析
                commands_process_string(usb_buf, usb_len, SERIAL_PRINTF_IS_USB);
//                commands_process_string(usb_buf, usb_len, SERIAL_PRINTF_IS_UART);// 也可以通过 UART 回显
#else
                // 精简模式: 使用 sscanf 直接匹配
                motor_parse_cmd(usb_buf, usb_len, SERIAL_PRINTF_IS_USB);
#endif
                USBD_CDC_ReceivePacket(&hUsbDeviceFS);  // 允许接收下一个 USB 数据包
            }
        } while (!reset_read_state);
    }
    // 如果执行到这里，说明线程已结束 (理论上不会发生)
    vTaskDelete(osThreadGetId());
}

/**
 * @brief 设置 USB 命令缓冲区 (由 USB 接收中断回调调用)
 * 
 * 本函数在 CDC_Receive_FS 回调函数中被调用，将接收到的 USB 数据
 * 缓冲区地址和长度传递给命令解析线程。
 * 
 * @param buf 指向 USB 接收数据缓冲区的指针
 * @param len USB 接收数据的长度 (字节)
 * 
 * 调用时机: USB CDC 端点接收到完整数据包后，在 USB 中断处理中调用
 */
void set_cmd_buffer(uint8_t *buf, uint32_t len) {
    usb_buf = buf;
    usb_len = len;
}

/**
 * @brief USB 更新线程
 * 
 * 本线程负责处理 USB 中断事件。由于 STM32 的 USB 中断需要在
 * 合适的时机调用 HAL_PCD_IRQHandler，而中断上下文不适合做复杂处理，
 * 因此采用信号量通知 + 线程处理的方式。
 * 
 * 工作流程:
 *   1. 等待 sem_usb_irq 信号量 (由 OTG_FS_IRQHandler 中断释放)
 *   2. 收到信号后调用 HAL_PCD_IRQHandler 处理 USB 事件
 *   3. 处理完成后重新使能 USB 中断，允许下一次中断触发
 * 
 * 为什么需要这个线程?
 *   - USB 中断处理需要较长时间，不适合在中断上下文中执行
 *   - 通过信号量将中断处理延迟到线程上下文中，提高系统实时性
 *   - 中断处理完成后需要手动重新使能中断，否则会丢失后续中断
 */
void usb_update_thread() {
    for (;;) {
        // 等待 USB 中断信号 (OTG_FS_IRQHandler 释放)
        osStatus semaphore_status = osSemaphoreWait(sem_usb_irq, osWaitForever);
        if (semaphore_status == osOK) {
            // 收到信号，处理新的 USB 传输事件
            HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
            // 重新使能 USB 中断，允许下一次中断触发
            HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        }
    }
    vTaskDelete(osThreadGetId());
}

/**
 * @brief 数据包定时器线程
 * 
 * 本线程以 1ms 为周期调用 packet_timerfunc()，用于处理
 * UART 数据包协议的超时和定时任务。
 * 
 * @param argument 线程参数 (未使用)
 * 
 * 注意: pdMS_TO_TICKS 宏用于将毫秒转换为系统 tick 数
 *       FreeRTOS V8.1.0 及以上版本使用 pdMS_TO_TICKS
 *       低版本应使用 1 / portTICK_RATE_MS
 */
void packet_timer_thread(void const * argument) {
    /* 定时器周期: 1ms。注: 使用 pdMS_TO_TICKS 将毫秒转换为系统 tick 数, FreeRTOS V8.1.0 及以上版本使用此宏,
       如果使用低版本, 请使用 1 / portTICK_RATE_MS */
    const portTickType xDelay = pdMS_TO_TICKS(1);
    
    // 定时器循环 - 永久运行
    for (;;) {
        packet_timerfunc();    // 执行数据包定时器回调 (处理超时、重发等)
        vTaskDelay(xDelay);    // 延时 1ms
    }
    // 如果执行到这里，说明线程已结束 (理论上不会发生)
    vTaskDelete(osThreadGetId());
}

