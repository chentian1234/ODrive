#ifndef COMMANDS_H
#define COMMANDS_H

/* Includes ------------------------------------------------------------------*/
#include <low_level.h>
/* Exported types ------------------------------------------------------------*/

/**
 * GPIO工作模式枚举
 * 用于配置GPIO引脚的功能模式
 */
typedef enum {
    GPIO_MODE_NONE,      /**< 无模式，GPIO未分配功能 */
    GPIO_MODE_UART,      /**< UART串口模式，用于串口通信 */
    GPIO_MODE_STEP_DIR,  /**< 步进/方向模式，用于接收脉冲方向信号控制电机 */
} GpioMode_t;

/**
 * 串口打印输出选择枚举
 * 用于选择printf输出到哪个接口（USB或UART）
 */
typedef enum {
    SERIAL_PRINTF_IS_NONE,  /**< 不输出 */
    SERIAL_PRINTF_IS_USB,   /**< 输出到USB虚拟串口 */
    SERIAL_PRINTF_IS_UART,  /**< 输出到硬件UART串口 */
} SerialPrintf_t;

/**
 * 暴露的浮点型变量索引枚举
 * 用于USB/串口通信时读写电机控制器的浮点型参数
 * 注释中RO表示只读(Read Only)，RW表示可读写(Read/Write)
 */
typedef enum {
    VBUS_VOLTAGE = 0,                        /**< 总线电压，单位V，只读 */
    NULLED,                                  /**< 保留占位，原为ELEC_RAD_PER_ENC（每转电弧度对应编码器计数），只读 */
    M0_POS_SETPOINT,                         /**< M0电机位置设定值，单位转，可读写 */
    M0_POS_GAIN,                             /**< M0电机位置环增益，可读写 */
    M0_VEL_SETPOINT,                         /**< M0电机速度设定值，单位转/秒，可读写 */
    M0_VEL_GAIN,                             /**< M0电机速度环增益，可读写 */
    M0_VEL_INTEGRATOR_GAIN,                  /**< M0电机速度积分环增益，可读写 */
    M0_VEL_INTEGRATOR_CURRENT,               /**< M0电机速度积分器当前输出电流，可读写 */
    M0_VEL_LIMIT,                            /**< M0电机速度限制，单位转/秒，可读写 */
    M0_CURRENT_SETPOINT,                     /**< M0电机电流设定值，单位安培，可读写 */
    M0_CALIBRATION_CURRENT,                  /**< M0电机校准电流，单位安培，可读写 */
    M0_PHASE_INDUCTANCE,                     /**< M0电机相电感，单位亨利，只读 */
    M0_PHASE_RESISTANCE,                     /**< M0电机相电阻，单位欧姆，只读 */
    M0_CURRENT_MEAS_PHB,                     /**< M0电机B相电流测量值，只读 */
    M0_CURRENT_MEAS_PHC,                     /**< M0电机C相电流测量值，只读 */
    M0_DC_CALIB_PHB,                         /**< M0电机B相直流偏置校准值，可读写 */
    M0_DC_CALIB_PHC,                         /**< M0电机C相直流偏置校准值，可读写 */
    M0_SHUNT_CONDUCTANCE,                    /**< M0电机分流器电导值，可读写 */
    M0_PHASE_CURRENT_REV_GAIN,               /**< M0电机相电流反向增益，可读写 */
    M0_CURRENT_CONTROL_CURRENT_LIM,          /**< M0电机电流控制电流限制，可读写 */
    M0_CURRENT_CONTROL_P_GAIN,               /**< M0电机电流控制比例增益(P)，可读写 */
    M0_CURRENT_CONTROL_I_GAIN,               /**< M0电机电流控制积分增益(I)，可读写 */
    M0_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_D,  /**< M0电机电流控制d轴积分器电压，可读写 */
    M0_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_Q,  /**< M0电机电流控制q轴积分器电压，可读写 */
    M0_CURRENT_CONTROL_IBUS,                 /**< M0电机电流控制总线电流，只读 */
    M0_ENCODER_PHASE,                        /**< M0编码器当前相位，只读 */
    M0_ENCODER_PLL_POS,                      /**< M0编码器PLL位置估计值，可读写 */
    M0_ENCODER_PLL_VEL,                      /**< M0编码器PLL速度估计值，可读写 */
    M0_ENCODER_PLL_KP,                       /**< M0编码器PLL比例增益，可读写 */
    M0_ENCODER_PLL_KI,                       /**< M0编码器PLL积分增益，可读写 */
    M1_POS_SETPOINT,                         /**< M1电机位置设定值，单位转，可读写 */
    M1_POS_GAIN,                             /**< M1电机位置环增益，可读写 */
    M1_VEL_SETPOINT,                         /**< M1电机速度设定值，单位转/秒，可读写 */
    M1_VEL_GAIN,                             /**< M1电机速度环增益，可读写 */
    M1_VEL_INTEGRATOR_GAIN,                  /**< M1电机速度积分环增益，可读写 */
    M1_VEL_INTEGRATOR_CURRENT,               /**< M1电机速度积分器当前输出电流，可读写 */
    M1_VEL_LIMIT,                            /**< M1电机速度限制，单位转/秒，可读写 */
    M1_CURRENT_SETPOINT,                     /**< M1电机电流设定值，单位安培，可读写 */
    M1_CALIBRATION_CURRENT,                  /**< M1电机校准电流，单位安培，可读写 */
    M1_PHASE_INDUCTANCE,                     /**< M1电机相电感，单位亨利，只读 */
    M1_PHASE_RESISTANCE,                     /**< M1电机相电阻，单位欧姆，只读 */
    M1_CURRENT_MEAS_PHB,                     /**< M1电机B相电流测量值，只读 */
    M1_CURRENT_MEAS_PHC,                     /**< M1电机C相电流测量值，只读 */
    M1_DC_CALIB_PHB,                         /**< M1电机B相直流偏置校准值，可读写 */
    M1_DC_CALIB_PHC,                         /**< M1电机C相直流偏置校准值，可读写 */
    M1_SHUNT_CONDUCTANCE,                    /**< M1电机分流器电导值，可读写 */
    M1_PHASE_CURRENT_REV_GAIN,               /**< M1电机相电流反向增益，可读写 */
    M1_CURRENT_CONTROL_CURRENT_LIM,          /**< M1电机电流控制电流限制，可读写 */
    M1_CURRENT_CONTROL_P_GAIN,               /**< M1电机电流控制比例增益(P)，可读写 */
    M1_CURRENT_CONTROL_I_GAIN,               /**< M1电机电流控制积分增益(I)，可读写 */
    M1_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_D,  /**< M1电机电流控制d轴积分器电压，可读写 */
    M1_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_Q,  /**< M1电机电流控制q轴积分器电压，可读写 */
    M1_CURRENT_CONTROL_IBUS,                 /**< M1电机电流控制总线电流，只读 */
    M1_ENCODER_PHASE,                        /**< M1编码器当前相位，只读 */
    M1_ENCODER_PLL_POS,                      /**< M1编码器PLL位置估计值，可读写 */
    M1_ENCODER_PLL_VEL,                      /**< M1编码器PLL速度估计值，可读写 */
    M1_ENCODER_PLL_KP,                       /**< M1编码器PLL比例增益，可读写 */
    M1_ENCODER_PLL_KI,                       /**< M1编码器PLL积分增益，可读写 */
		FLOATS_END                                 /**< 浮点型变量结束标记，用于遍历 */
} Exposed_Floats_t;

/**
 * 暴露的整型变量索引枚举
 * 用于USB/串口通信时读写电机控制器的整型参数
 * 注释中RO表示只读(Read Only)，RW表示可读写(Read/Write)
 */
typedef enum {
    M0_CONTROL_MODE = 0,                     /**< M0电机控制模式，可读写 */
    M0_ENCODER_ENCODER_OFFSET,               /**< M0编码器偏移量，可读写 */
    M0_ENCODER_ENCODER_STATE,                /**< M0编码器状态，只读 */
    M0_ERROR,                                /**< M0电机错误标志，可读写 */
    M1_CONTROL_MODE,                         /**< M1电机控制模式，可读写 */
    M1_ENCODER_ENCODER_OFFSET,               /**< M1编码器偏移量，可读写 */
    M1_ENCODER_ENCODER_STATE,                /**< M1编码器状态，只读 */
    M1_ERROR,                                /**< M1电机错误标志，可读写 */
		INTS_END                                 /**< 整型变量结束标记，用于遍历 */
} Exposed_Ints_t;

/**
 * 暴露的布尔型变量索引枚举
 * 用于USB/串口通信时读写电机控制器的布尔型参数
 * 注释中RO表示只读(Read Only)，RW表示可读写(Read/Write)
 */
typedef enum {
    M0_THREAD_READY = 0,                     /**< M0线程就绪状态，只读 */
    M0_ENABLE_CONTROL,                       /**< M0使能控制，可读写 */
    M0_DO_CALIBRATION,                       /**< M0执行校准命令，可读写 */
    M0_CALIBRATION_OK,                       /**< M0校准完成标志，只读 */
    M1_THREAD_READY,                         /**< M1线程就绪状态，只读 */
    M1_ENABLE_CONTROL,                       /**< M1使能控制，可读写 */
    M1_DO_CALIBRATION,                       /**< M1执行校准命令，可读写 */
    M1_CALIBRATION_OK,                       /**< M1校准完成标志，只读 */
		BOOLS_END                                /**< 布尔型变量结束标记，用于遍历 */
} Exposed_Bools_t;

/**
 * 暴露的16位无符号整型变量索引枚举
 * 用于USB/串口通信时读写电机控制器的16位无符号整型参数
 * 注释中RO表示只读(Read Only)，RW表示可读写(Read/Write)
 */
typedef enum {
    M0_CONTROL_DEADLINE = 0,                 /**< M0控制周期/截止时间，单位tick，可读写 */
    M0_LAST_CPU_TIME,                        /**< M0上一次CPU运行时间，只读 */
    M1_CONTROL_DEADLINE,                     /**< M1控制周期/截止时间，单位tick，可读写 */
    M1_LAST_CPU_TIME,                        /**< M1上一次CPU运行时间，只读 */
		UINT16_END                               /**< 16位无符号整型变量结束标记，用于遍历 */
} Exposed_Uint16_t;
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/** 串口打印输出选择，用于指定printf输出到哪个接口(USB或UART) */
extern SerialPrintf_t serial_printf_select;

/** 暴露的浮点型变量指针数组，用于USB/串口通信时按索引访问浮点参数 */
extern float* const exposed_floats[];

/** 暴露的整型变量指针数组，用于USB/串口通信时按索引访问整型参数 */
extern int* const exposed_ints[];

/** 暴露的布尔型变量指针数组，用于USB/串口通信时按索引访问布尔参数 */
extern bool* const exposed_bools[];

/** 暴露的16位无符号整型变量指针数组，用于USB/串口通信时按索引访问uint16参数 */
extern uint16_t* const exposed_uint16[];

/** 监控槽数组，最多20个槽位，用于实时监控电机参数 */
extern monitoring_slot monitoring_slots[20];
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化通信模块
 * @note 在系统启动时调用，初始化USB、UART等通信接口
 */
void init_communication();

/**
 * @brief 命令解析线程
 * @param argument 线程参数
 * @note 作为RTOS线程运行，持续解析来自通信接口的命令
 */
void cmd_parse_thread(void const * argument);

/**
 * @brief 数据包定时器线程
 * @param argument 线程参数
 * @note 作为RTOS线程运行，处理数据包的超时和定时任务
 */
void packet_timer_thread(void const * argument);

/**
 * @brief 解析电机控制命令
 * @param buffer 命令数据缓冲区
 * @param len 命令数据长度
 * @param response_interface 响应输出接口(USB或UART)
 * @note 解析接收到的电机控制命令并执行相应操作
 */
void motor_parse_cmd(uint8_t* buffer, int len, SerialPrintf_t response_interface);

#define ARM_TERMINAL
#if defined ARM_TERMINAL

/**
 * @brief 处理终端命令字符串
 * @param buffer 命令字符串缓冲区
 * @param len 命令字符串长度
 * @param response_interface 响应输出接口
 * @note 解析并执行来自终端的命令行指令
 */
void commands_process_string(uint8_t* buffer, int len, SerialPrintf_t response_interface);

/**
 * @brief 注册命令回调函数
 * @param command 命令名称
 * @param help 命令帮助信息
 * @param arg_names 参数名称说明
 * @param cbf 命令回调函数指针
 * @note 用于向终端命令系统注册新的命令及其处理函数
 */
void commands_register_command_callback(
    const char* command,
    const char *help,
    const char *arg_names,
    void(*cbf)(int argc, const char **argv));
#endif

/**
 * @brief 打印监控数据
 * @param limit 打印数量限制
 * @note 将监控槽中的数据通过通信接口输出
 */
void print_monitoring(int limit);		

/**
 * @brief 设置命令缓冲区
 * @param buf 命令数据缓冲区指针
 * @param len 命令数据长度
 * @note 用于设置待处理的命令数据
 */
void set_cmd_buffer(uint8_t *buf, uint32_t len);

/**
 * @brief USB更新线程
 * @note 作为RTOS线程运行，处理USB通信的数据收发
 */
void usb_update_thread();

#if defined ARM_PRINTF

#endif
#if defined ARM_TERMINAL

#endif
		
#define ARM_COMMMAND_UART		
#if defined ARM_COMMMAND_UART

#endif

#if defined ARM_PRINTF
/**
 * @brief 格式化打印输出(支持变参)
 * @param format 格式化字符串
 * @param ... 可变参数
 * @note 当启用ARM_PRINTF宏时可用，用于格式化输出到ARM终端
 */
void cmd_printf(const char* format, ...);
#endif
#if defined ARM_COMMMAND_UART
#define PACKET_HANDLER 0
#endif			

#endif /* COMMANDS_H */
