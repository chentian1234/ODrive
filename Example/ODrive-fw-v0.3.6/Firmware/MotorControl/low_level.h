/* Define to prevent recursive inclusion -------------------------------------*/
/**
  * @file low_level.h
  * @brief ODrive 电机底层控制头文件
  * 
  * 本文件定义了电机控制的核心数据结构、枚举类型、宏定义以及底层控制函数。
  * 主要包含：
  *   - 电机控制模式（电压/电流/速度/位置控制）
  *   - 电机类型定义（高电流/云台电机）
  *   - FOC（磁场定向控制）相关结构体
  *   - 编码器与无传感器估算器配置
  *   - 电机校准与测量函数
  *   - 中断回调函数声明
  */

#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "drv8301.h"

/**
  * @brief 电机索引宏定义
  */
#define M0 0   /**< 电机0索引 */
#define M1 1   /**< 电机1索引 */

/**
  * @brief 等待相位测量信号的默认超时时间
  */
#define PH_CURRENT_MEAS_TIMEOUT 2 // [ms] 相位电流测量超时时间，单位毫秒

    /* Exported types ------------------------------------------------------------*/
    
    /**
      * @brief 电机线程信号枚举
      * 
      * 用于电机控制线程间的信号通信，采用位掩码方式定义。
      */
    typedef enum {
        M_SIGNAL_PH_CURRENT_MEAS = 1u << 0   /**< 相位电流测量完成信号 */
    } Motor_thread_signals_t;

    /**
      * @brief 齿槽转矩补偿（反齿槽效应）配置结构体
      * 
      * 用于存储和配置电机的齿槽转矩补偿参数。齿槽转矩是永磁电机中由于
      * 定子齿槽效应引起的转矩波动，通过预先标定的位置-转矩映射表进行补偿。
      */
    typedef struct {
        int index;                    /**< 索引，用于标识当前的补偿位置索引 */
        float *cogging_map;           /**< 齿槽转矩映射表指针，存储不同位置下的补偿转矩值 */
        bool use_anticogging;         /**< 是否启用齿槽转矩补偿功能 */
        bool calib_anticogging;       /**< 是否正在进行齿槽转矩标定 */
        float calib_pos_threshold;    /**< 标定时的位置容差阈值，超过此值认为位置有效 */
        float calib_vel_threshold;    /**< 标定时的速度容差阈值，超过此值认为速度有效 */
    } Anticogging_t;

    /**
      * @brief 电机错误码枚举
      * 
      * 定义了电机控制过程中可能出现的各种错误类型，用于故障诊断和状态报告。
      */
    typedef enum {
        ERROR_NO_ERROR = 0,                       /**< 无错误 */
        ERROR_PHASE_RESISTANCE_TIMING = 1,        /**< 相位电阻测量时序错误 */
        ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT = 2, /**< 相位电阻测量超时 */
        ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 3,  /**< 相位电阻测量值超出范围 */
        ERROR_PHASE_INDUCTANCE_TIMING = 4,        /**< 相位电感测量时序错误 */
        ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT = 5, /**< 相位电感测量超时 */
        ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 6,  /**< 相位电感测量值超出范围 */
        ERROR_ENCODER_RESPONSE = 7,               /**< 编码器无响应 */
        ERROR_ENCODER_MEASUREMENT_TIMEOUT = 8,    /**< 编码器测量超时 */
        ERROR_ADC_FAILED = 9,                     /**< ADC转换失败 */
        ERROR_CALIBRATION_TIMING = 10,            /**< 校准时序错误 */
        ERROR_FOC_TIMING = 11,                    /**< FOC计算时序错误 */
        ERROR_FOC_MEASUREMENT_TIMEOUT = 12,       /**< FOC测量超时 */
        ERROR_SCAN_MOTOR_TIMING = 13,             /**< 电机扫描时序错误 */
        ERROR_FOC_VOLTAGE_TIMING = 14,            /**< FOC电压控制时序错误 */
        ERROR_GATEDRIVER_INVALID_GAIN = 15,       /**< 栅极驱动器增益无效 */
        ERROR_PWM_SRC_FAIL = 16,                  /**< PWM源故障 */
        ERROR_UNEXPECTED_STEP_SRC = 17,           /**< 意外的步进信号源 */
        ERROR_POS_CTRL_DURING_SENSORLESS = 18,    /**< 无传感器模式下尝试位置控制 */
        ERROR_SPIN_UP_TIMEOUT = 19,               /**< 启动超时（无传感器模式启动失败） */
        ERROR_DRV_FAULT = 20,                     /**< DRV8301驱动芯片故障 */
        ERROR_NOT_IMPLEMENTED_MOTOR_TYPE = 21,    /**< 未实现的电机类型 */
        ERROR_ENCODER_CPR_OUT_OF_RANGE = 22,      /**< 编码器每转脉冲数(CPR)超出范围 */
        ERROR_DC_BUS_BROWNOUT = 23,               /**< 直流母线欠压保护触发 */
    } Error_t;

    /**
      * @brief 电机控制模式枚举
      * 
      * 定义了电机支持的不同控制模式。注意：这些值按照控制层级从低到高排列，
      * 以便支持使用 "<" 操作符进行控制级别比较。
      * 控制层级：电压控制 < 电流控制 < 速度控制 < 位置控制
      */
    typedef enum {
        CTRL_MODE_VOLTAGE_CONTROL = 0,    /**< 电压控制模式：直接控制相电压，最低层级控制 */
        CTRL_MODE_CURRENT_CONTROL = 1,    /**< 电流控制模式：通过FOC控制相电流 */
        CTRL_MODE_VELOCITY_CONTROL = 2,   /**< 速度控制模式：闭环速度控制，内部包含电流环 */
        CTRL_MODE_POSITION_CONTROL = 3    /**< 位置控制模式：闭环位置控制，内部包含速度环和电流环 */
    } Motor_control_mode_t;

    /**
      * @brief 电机类型枚举
      * 
      * 定义支持的电机类型，不同类型需要不同的控制策略。
      */
    typedef enum {
        MOTOR_TYPE_HIGH_CURRENT = 0,      /**< 高电流电机：标准BLDC/PMSM电机，需要完整的FOC控制 */
        // MOTOR_TYPE_LOW_CURRENT = 1,    /**< 低电流电机：暂未实现 */
        MOTOR_TYPE_GIMBAL = 2             /**< 云台电机：高电阻低电感电机，可直接使用电压控制 */
    } Motor_type_t;

    /**
      * @brief B相和C相电流数据结构体
      * 
      * 存储电机B相和C相的电流测量值。在三相平衡系统中，
      * 通过测量两相电流即可推算出第三相电流（Ia + Ib + Ic = 0）。
      */
    typedef struct {
        float phB;    /**< B相电流值，单位安培[A] */
        float phC;    /**< C相电流值，单位安培[A] */
    } Iph_BC_t;

    /**
      * @brief 电流控制环状态结构体
      * 
      * 包含FOC电流控制环的所有状态变量和参数。电流环是最内层的控制环，
      * 负责控制电机的d轴和q轴电流，从而实现对转矩的精确控制。
      */
    typedef struct {
        float current_lim;                        /**< 电流限制值，单位安培[A]，用于过流保护 */
        float p_gain;                             /**< 比例增益（P增益），单位[V/A]，电流环比例控制系数 */
        float i_gain;                             /**< 积分增益（I增益），单位[V/(A·s)]，电流环积分控制系数 */
        float v_current_control_integral_d;       /**< d轴电流控制积分项输出，单位伏特[V] */
        float v_current_control_integral_q;       /**< q轴电流控制积分项输出，单位伏特[V] */
        float Ibus;                               /**< 直流母线电流，单位安培[A]，反映总功耗 */
        // Voltage applied at end of cycle:
        float final_v_alpha;                      /**< 控制周期结束时施加的α轴电压，单位伏特[V] */
        float final_v_beta;                       /**< 控制周期结束时施加的β轴电压，单位伏特[V] */
        float Iq_setpoint;                        /**< q轴电流设定值，单位安培[A]，q轴电流决定输出转矩 */
        float Iq_measured;                        /**< q轴电流测量值，单位安培[A] */
        float max_allowed_current;                /**< 允许的最大电流值，单位安培[A] */
        float Id_measured;                        /**< d轴电流测量值，单位安培[A] */
        float Iq_act;                             /**< q轴实际电流值，单位安培[A] */
    } Current_control_t;

    /**
      * @brief 转子运行模式枚举
      * 
      * 定义电机的转子位置获取方式：编码器模式或无传感器估算模式。
      */
    typedef enum {
        ROTOR_MODE_ENCODER,                              /**< 编码器模式：使用物理编码器获取转子位置 */
        ROTOR_MODE_SENSORLESS,                           /**< 无传感器模式：通过反电动势估算转子位置 */
        ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS           /**< 编码器运行+无传感器测试模式：使用编码器控制，同时运行无传感器估算器进行测试验证 */
    } Rotor_mode_t;

    /**
      * @brief 无传感器估算器状态结构体
      * 
      * 包含无传感器FOC控制中使用的磁链观测器和PLL（锁相环）的所有状态变量。
      * 无传感器控制通过观测电机的反电动势来估算转子位置和速度，适用于
      * 没有安装编码器的场合或作为编码器的冗余备份。
      */
    typedef struct {
        float phase;                                    /**< 估算的转子电角度相位，单位弧度[rad] */
        float pll_pos;                                  /**< PLL（锁相环）估算的位置，单位弧度[rad] */
        float pll_vel;                                  /**< PLL（锁相环）估算的速度，单位弧度每秒[rad/s] */
        float pll_kp;                                   /**< PLL比例增益，影响位置跟踪响应速度 */
        float pll_ki;                                   /**< PLL积分增益，影响稳态精度和抗扰动能力 */
        float observer_gain;                            /**< 观测器增益，单位[rad/s]，决定观测器收敛速度 */
        float flux_state[2];                            /**< 磁链状态向量（二维），单位韦伯[Vs]，用于估算转子位置 */
        float V_alpha_beta_memory[2];                   /**< 历史αβ轴电压存储（二维），单位伏特[V]，用于观测器计算 */
        float pm_flux_linkage;                          /**< 永磁体磁链，单位[V/(rad/s)]，即反电动势常数Ke */
        bool estimator_good;                            /**< 估算器状态标志，true表示估算器数据可靠 */
        float spin_up_current;                          /**< 启动电流，单位安培[A]，用于无传感器模式启动时建立反电动势 */
        float spin_up_acceleration;                     /**< 启动加速度，单位[rad/s²]，启动过程中的转速爬升率 */
        float spin_up_target_vel;                       /**< 启动目标速度，单位[rad/s]，启动过程需要达到的最低速度 */
    } Sensorless_t;

    /**
      * @brief 编码器配置与状态结构体
      * 
      * 包含编码器的所有配置参数和运行状态。编码器用于精确测量转子的
      * 位置和速度，是实现高性能位置/速度闭环控制的关键传感器。
      */
    typedef struct {
        TIM_HandleTypeDef* encoder_timer;               /**< 编码器定时器句柄指针，用于读取编码器计数值 */
        bool use_index;                                 /**< 是否使用编码器Z信号（索引脉冲）进行归零 */
        bool index_found;                               /**< 是否已找到编码器索引脉冲 */
        bool manually_calibrated;                       /**< 是否已完成手动校准 */
        float idx_search_speed;                         /**< 索引搜索速度，查找Z信号时的扫描速度 */
        int32_t encoder_cpr;                            /**< 编码器每转脉冲数（Counts Per Revolution），编码器的分辨率 */
        int32_t encoder_offset;                         /**< 编码器零位偏移量，校准后存储的机械零位与电气零位的偏差 */
        int32_t encoder_state;                          /**< 编码器当前状态值（原始计数值） */
        int32_t motor_dir;                              /**< 电机方向，1表示正转对齐编码器，-1表示反转对齐编码器 */
        float encoder_calib_range;                      /**< 编码器校准范围容差，判断校准是否成功的阈值 */
        float phase;                                    /**< 从编码器获取的转子电角度相位，单位弧度[rad] */
        float pll_pos;                                  /**< 基于编码器位置的PLL位置，单位弧度[rad] */
        float pll_vel;                                  /**< 基于编码器位置的PLL速度，单位弧度每秒[rad/s] */
        float pll_kp;                                   /**< PLL比例增益，用于编码器速度估算 */
        float pll_ki;                                   /**< PLL积分增益，用于编码器速度估算 */
    } Encoder_t;

    /**
      * @brief 轴控制遗留结构体（旧版兼容）
      * 
      * 用于兼容旧版axis对象的结构体，保留控制使能标志。
      */
    typedef struct {
        bool* enable_control;                           /**< 控制使能标志指针，指向是否允许电机控制的标志位 */
    } Axis_legacy_t;

/**
  * @brief 时序日志缓冲区大小
  * 
  * 定义时序性能日志的环形缓冲区条目数，用于分析和优化控制循环的执行时间。
  */
#define TIMING_LOG_SIZE 16

    /**
      * @brief 电机控制核心结构体
      * 
      * 这是电机控制的最主要数据结构，包含了电机控制所需的所有状态、参数和配置。
      * 每个电机实例对应一个Motor_t结构体，ODrive支持双电机控制（M0和M1）。
      * 该结构体涵盖了：
      *   - 控制模式和参数设定
      *   - PID控制器参数
      *   - 电流环、速度环、位置环配置
      *   - 编码器/无传感器模式配置
      *   - 硬件接口（定时器、ADC、栅极驱动器）
      *   - 错误状态和诊断信息
      *   - 性能监测（时序日志、CPU使用率）
      */
    typedef struct {
        Axis_legacy_t axis_legacy;                      /**< 轴控制遗留对象（旧版兼容），包含使能控制指针 */
        Motor_control_mode_t control_mode;              /**< 当前电机控制模式（电压/电流/速度/位置） */
        bool enable_step_dir;                           /**< 是否启用步进/方向信号输入模式 */
        float counts_per_step;                          /**< 每步对应的脉冲数，用于step/dir模式的位置换算 */
        Error_t error;                                  /**< 当前错误码，记录最近发生的故障类型 */
        int32_t pole_pairs;                             /**< 电机极对数，电机转子磁极对数，用于电气角度计算 */
        float pos_setpoint;                             /**< 位置设定值，目标位置，单位取决于编码器CPR */
        float pos_gain;                                 /**< 位置环比例增益（P增益），决定位置跟踪响应速度 */
        float vel_setpoint;                             /**< 速度设定值，目标速度，单位弧度每秒[rad/s] */
        float vel_gain;                                 /**< 速度环比例增益（P增益），决定速度跟踪响应速度 */
        float vel_integrator_gain;                      /**< 速度环积分增益（I增益），消除稳态速度误差 */
        float vel_integrator_current;                   /**< 速度环积分器输出电流值，用于累积速度误差补偿 */
        float vel_limit;                                /**< 速度限制值，单位[rad/s]，电机最大允许速度 */
        float current_setpoint;                         /**< 电流设定值，目标电流，单位安培[A] */
        float calibration_current;                      /**< 校准时使用的电流值，单位安培[A]，用于电阻/电感测量 */
        float resistance_calib_max_voltage;             /**< 电阻校准时的最大电压限制，单位伏特[V] */
        float dc_bus_brownout_trip_level;               /**< 直流母线欠压保护触发电平，单位伏特[V] */
        float phase_inductance;                         /**< 电机相电感，单位亨[H]，电机参数，用于电流环控制 */
        float phase_resistance;                         /**< 电机相电阻，单位欧姆[Ω]，电机参数，用于电流环控制 */
        TaskHandle_t motor_thread;                        /**< 电机控制线程ID，FreeRTOS线程句柄 */
        bool thread_ready;                              /**< 线程就绪标志，true表示电机控制线程已初始化完成 */
        bool enable_control;                            /**< 控制使能标志，通过USB等接口设置的电机控制开关。
                                                         *   发生错误时会自动清零，需要calibration_ok=true才能启用 */
        bool do_calibration;                            /**< 校准触发标志，设置为true时启动电机校准流程。
                                                         *   校准完成后自动复位为false */
        bool calibration_ok;                            /**< 校准成功标志，true表示电机校准已完成且参数有效 */
        TIM_HandleTypeDef* motor_timer;                 /**< 电机PWM定时器句柄指针，用于产生三相PWM波形 */
        uint16_t next_timings[3];                       /**< 下一个PWM周期的三路定时器比较值，对应三相桥臂的占空比 */
        uint16_t control_deadline;                      /**< 控制周期截止时间戳，用于监控控制循环是否超时 */
        uint16_t last_cpu_time;                         /**< 上一次控制循环的CPU执行时间，用于性能监控 */
        Iph_BC_t current_meas;                          /**< 当前测量的B/C相电流值 */
        Iph_BC_t DC_calib;                              /**< 直流偏置校准值，ADC零电流时的偏置补偿 */
        DRV8301_Obj gate_driver;                        /**< DRV8301栅极驱动器对象，控制三相桥臂的MOSFET */
        DRV_SPI_8301_Vars_t gate_driver_regs;           /**< DRV8301寄存器本地视图，通过SPI读取的驱动器状态和配置 */
        Motor_type_t motor_type;                        /**< 电机类型（高电流/云台） */
        float shunt_conductance;                        /**< 分流电导值，单位西门子[S]，电流采样电路参数，电导=1/电阻 */
        float phase_current_rev_gain;                   /**< 相位电流反向增益，ADC采样值转换为安培的换算系数 */
        Current_control_t current_control;              /**< 电流控制环状态，包含FOC电流控制的所有变量 */
        Rotor_mode_t rotor_mode;                        /**< 转子运行模式（编码器/无传感器） */
        Encoder_t encoder;                              /**< 编码器配置与状态 */
        Sensorless_t sensorless;                        /**< 无传感器估算器状态 */
        uint32_t loop_counter;                          /**< 控制循环计数器，记录控制循环执行的总次数 */
        uint16_t timing_log[TIMING_LOG_SIZE];           /**< 时序日志数组，记录各阶段的执行时间，用于性能分析 */
        // Cache for remote procedure calls arguments
        struct {
            float pos_setpoint;                         /**< 远程调用缓存：位置设定值参数 */
            float vel_feed_forward;                     /**< 远程调用缓存：速度前馈参数，用于提高响应速度 */
            float current_feed_forward;                 /**< 远程调用缓存：电流前馈参数，用于提高响应速度 */
        } set_pos_setpoint_args;                        /**< 位置设定参数缓存（用于远程过程调用） */
        struct {
            float vel_setpoint;                         /**< 远程调用缓存：速度设定值参数 */
            float current_feed_forward;                 /**< 远程调用缓存：电流前馈参数 */
        } set_vel_setpoint_args;                        /**< 速度设定参数缓存（用于远程过程调用） */
        struct {
            float current_setpoint;                     /**< 远程调用缓存：电流设定值参数 */
        } set_current_setpoint_args;                    /**< 电流设定参数缓存（用于远程过程调用） */
        Anticogging_t anticogging;                      /**< 齿槽转矩补偿配置与状态 */
        DRV8301_FaultType_e drv_fault;                  /**< DRV8301驱动芯片故障类型，详细的硬件故障码 */
    } Motor_t;

    /**
      * @brief 时序日志类型枚举
      * 
      * 定义不同的时序日志记录点，用于测量和分析代码各阶段的执行时间。
      * 通过在关键代码段插入计时点，可以诊断性能瓶颈和控制周期延迟。
      */
    typedef enum {
        TIMING_LOG_GENERAL,                             /**< 通用时序日志记录点 */
        TIMING_LOG_ADC_CB_M0_I,                         /**< M0电机相电流ADC中断回调时序 */
        TIMING_LOG_ADC_CB_M0_DC,                        /**< M0电机电流DC偏置ADC中断回调时序 */
        TIMING_LOG_ADC_CB_M1_I,                         /**< M1电机相电流ADC中断回调时序 */
        TIMING_LOG_ADC_CB_M1_DC,                        /**< M1电机电流DC偏置ADC中断回调时序 */
        TIMING_LOG_MEAS_R,                              /**< 相位电阻测量时序 */
        TIMING_LOG_MEAS_L,                              /**< 相位电感测量时序 */
        TIMING_LOG_ENC_CALIB,                           /**< 编码器校准时序 */
        TIMING_LOG_IDX_SEARCH,                          /**< 编码器索引脉冲搜索时序 */
        TIMING_LOG_FOC_VOLTAGE,                         /**< FOC电压控制时序 */
        TIMING_LOG_FOC_CURRENT,                         /**< FOC电流控制时序 */
    } TimingLog_t;

    /**
      * @brief 监控槽位结构体
      * 
      * 用于配置实时监控的数据槽，每个槽位指定一个要监控的数据类型和索引。
      */
    typedef struct {
        int type;                                       /**< 监控数据类型标识 */
        int index;                                      /**< 监控数据索引，指定同一类型中的具体实例 */
    } monitoring_slot;

    /* Exported constants --------------------------------------------------------*/
    /**
      * @brief 电机数量常量
      * 
      * 系统中支持的电机数量，ODrive通常为2（双电机控制）。
      */
    extern const size_t num_motors;

    /**
      * @brief 每个编码器脉冲对应的电气弧度
      * 
      * 将编码器计数转换为电气角度的换算系数，计算方式为：
      * elec_rad_per_enc = pole_pairs * 2 * PI / encoder_cpr
      */
    extern const float elec_rad_per_enc;

    /* Exported variables --------------------------------------------------------*/
    /**
      * @brief 直流母线电压
      * 
      * 当前测量的直流母线（电源）电压值，单位伏特[V]。
      * 该电压用于PWM占空比计算和欠压保护判断。
      */
    extern float vbus_voltage;

    /**
      * @brief 制动电阻值
      * 
      * 外接制动电阻的阻值，单位欧姆[Ω]。
      * 电机再生制动时，多余能量通过制动电阻消耗。
      */
    extern float brake_resistance;

    /**
      * @brief 电机控制数组
      * 
      * 系统中所有电机实例的数组，可通过索引访问各个电机。
      */
    extern Motor_t motors[];
    /* Exported macro ------------------------------------------------------------*/
    /* Exported functions --------------------------------------------------------*/

    /* ===== 设定值接口函数 ===== */

    /**
      * @brief 设置位置设定值（带前馈补偿）
      * 
      * 设置电机的目标位置，并可选地提供速度和电流前馈值以提高动态响应。
      * 前馈参数可设为0.0f以禁用前馈控制。
      * 
      * @param[in] motor             电机实例指针
      * @param[in] pos_setpoint      目标位置设定值
      * @param[in] vel_feed_forward  速度前馈值，用于提前补偿预期速度需求，可设为0.0f
      * @param[in] current_feed_forward 电流前馈值，用于提前补偿预期转矩需求，可设为0.0f
      */
    void set_pos_setpoint(Motor_t* motor, float pos_setpoint, float vel_feed_forward, float current_feed_forward);

    /**
      * @brief 设置速度设定值（带前馈补偿）
      * 
      * 设置电机的目标速度，并可选地提供电流前馈值以提高动态响应。
      * 前馈参数可设为0.0f以禁用前馈控制。
      * 
      * @param[in] motor             电机实例指针
      * @param[in] vel_setpoint      目标速度设定值，单位[rad/s]
      * @param[in] current_feed_forward 电流前馈值，用于提前补偿预期转矩需求，可设为0.0f
      */
    void set_vel_setpoint(Motor_t* motor, float vel_setpoint, float current_feed_forward);

    /**
      * @brief 设置电流设定值
      * 
      * 直接设置电机的目标电流（q轴电流），用于电流控制模式。
      * 
      * @param[in] motor             电机实例指针
      * @param[in] current_setpoint  目标电流设定值，单位安培[A]
      */
    void set_current_setpoint(Motor_t* motor, float current_setpoint);

    /* ===== 中断回调函数 ===== */

    /**
      * @brief 步进信号回调函数
      * 
      * 在检测到步进脉冲信号时调用，用于step/dir模式下的位置增量控制。
      * 
      * @param[in] GPIO_Pin 触发中断的GPIO引脚编号
      */
    void step_cb(uint16_t GPIO_Pin);

    /**
      * @brief 编码器索引脉冲回调函数
      * 
      * 在检测到编码器Z信号（索引脉冲）时调用，用于编码器零位校准。
      * 
      * @param[in] GPIO_Pin    触发中断的GPIO引脚编号
      * @param[in] motor_index 电机索引（M0或M1）
      */
    void enc_index_cb(uint16_t GPIO_Pin, uint8_t motor_index);

    /**
      * @brief PWM触发ADC转换回调函数
      * 
      * 在PWM定时器触发ADC采样时调用，用于读取相电流采样值。
      * 
      * @param[in] hadc      ADC句柄指针
      * @param[in] injected  是否为注入组转换，true表示注入组，false表示规则组
      */
    void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc, bool injected);

    /**
      * @brief 母线电压ADC采样回调函数
      * 
      * 在ADC完成母线电压采样时调用，用于读取直流母线电压和制动电流。
      * 
      * @param[in] hadc      ADC句柄指针
      * @param[in] injected  是否为注入组转换
      */
    void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc, bool injected);

    /* ===== 通用工具函数 ===== */

    /**
      * @brief 安全断言检查
      * 
      * 类似于标准assert，但提供更安全的错误处理。
      * 当参数为0（假）时，触发故障保护机制。
      * 
      * @param[in] arg 断言条件，0表示断言失败
      */
    void safe_assert(int arg);

    /**
      * @brief 初始化电机控制系统
      * 
      * 执行电机控制相关的硬件初始化和软件配置，包括：
      *   - 配置PWM定时器和ADC
      *   - 初始化栅极驱动器
      *   - 创建电机控制线程
      *   - 初始化各结构体默认值
      */
    void init_motor_control();

    /**
      * @brief 设置编码器计数值
      * 
      * 手动设置编码器的当前计数值，用于位置归零或校准。
      * 
      * @param[in] motor   电机实例指针
      * @param[in] count   要设置的编码器计数值
      */
    void setEncoderCount(Motor_t* motor, uint32_t count);

    /* ===== 校准与测量函数 ===== */

    /**
      * @brief 齿槽转矩补偿标定
      * 
      * 执行齿槽转矩补偿的标定流程，通过缓慢旋转电机并记录不同位置下的
      * 电流响应，建立位置-补偿转矩映射表。
      * 
      * @param[in] motor   电机实例指针
      * @return true       标定成功
      * @return false      标定失败
      */
    bool anti_cogging_calibration(Motor_t* motor);

    /**
      * @brief 电机校准主函数
      * 
      * 执行完整的电机校准流程，包括：
      *   - 相位电阻测量
      *   - 相位电感测量
      *   - 编码器偏移校准
      *   - 编码器索引搜索
      * 
      * @param[in] motor   电机实例指针
      * @return true       校准成功，所有参数有效
      * @return false      校准失败，检查motor->error获取错误码
      */
    bool motor_calibration(Motor_t* motor);


    /* ===== 内部工具函数 ===== */

    /**
      * @brief 检查代码执行时序并记录
      * 
      * 计算当前时刻与上次调用之间的时间差，并记录到指定的时序日志中。
      * 用于性能分析和控制周期监控。
      * 
      * @param[in] motor    电机实例指针
      * @param[in] log_idx  时序日志类型索引
      * @return uint16_t    自上次调用以来的时间差（定时器计数值）
      */
    uint16_t check_timing(Motor_t* motor, TimingLog_t log_idx);

    /**
      * @brief 全局故障处理
      * 
      * 触发全局故障状态，停止所有电机运行并记录错误码。
      * 这是一个安全保护函数，用于紧急停止和故障响应。
      * 
      * @param[in] error   错误码，标识故障类型
      */
    void global_fault(int error);

    /**
      * @brief ADC采样值转换为相电流
      * 
      * 将ADC原始采样值转换为实际的相电流值（安培）。
      * 使用分流电阻和增益参数进行换算。
      * 
      * @param[in] motor     电机实例指针
      * @param[in] ADCValue  ADC原始采样值
      * @return float        转换后的相电流值，单位安培[A]
      */
    float phase_current_from_adcval(Motor_t* motor, uint32_t ADCValue);

    /* ===== 硬件初始化函数 ===== */

    /**
      * @brief DRV8301栅极驱动器配置
      * 
      * 通过SPI接口配置DRV8301栅极驱动芯片的寄存器，包括：
      *   - 增益设置
      *   - 保护功能配置
      *   - 工作状态设置
      * 
      * @param[in] motor   电机实例指针
      */
    void DRV8301_setup(Motor_t* motor);

    /**
      * @brief 启动ADC和PWM
      * 
      * 启动ADC采样和PWM输出，使能电机控制的硬件外设。
      * 此函数在初始化完成后调用，开始电机控制循环。
      */
    void start_adc_pwm();

    /**
      * @brief 启动PWM定时器
      * 
      * 启动指定的PWM定时器，产生三相桥臂的PWM波形。
      * 
      * @param[in] htim   定时器句柄指针
      */
    void start_pwm(TIM_HandleTypeDef* htim);

    /**
      * @brief 同步两个定时器
      * 
      * 将两个定时器同步，使它们产生相位一致的PWM波形。
      * 用于双电机应用中保持两个电机的PWM同步。
      * 
      * @param[in] htim_a              定时器A句柄指针（主定时器）
      * @param[in] htim_b              定时器B句柄指针（从定时器）
      * @param[in] TIM_CLOCKSOURCE_ITRx 定时器触发源选择（ITRx触发）
      * @param[in] count_offset        计数器偏移量，用于相位补偿
      */
    void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b,
                     uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset);

    /* ===== 测量与校准函数 ===== */

    /**
      * @brief 测量相位电阻
      * 
      * 通过向电机绕组注入测试电流并测量电压降来计算相位电阻。
      * 这是电机校准的第一步，电阻值用于电流环的解耦计算。
      * 
      * @param[in] motor        电机实例指针
      * @param[in] test_current 测试电流，单位安培[A]
      * @param[in] max_voltage  最大测试电压，单位伏特[V]，用于保护电机
      * @return true            测量成功
      * @return false           测量失败（超时或值超出范围）
      */
    bool measure_phase_resistance(Motor_t* motor, float test_current, float max_voltage);

    /**
      * @brief 测量相位电感
      * 
      * 通过向电机绕组施加电压阶跃并测量电流变化率来计算相位电感。
      * 电感值用于电流环的带宽计算和前馈补偿。
      * 
      * @param[in] motor         电机实例指针
      * @param[in] voltage_low   低电平电压，单位伏特[V]
      * @param[in] voltage_high  高电平电压，单位伏特[V]
      * @return true             测量成功
      * @return false            测量失败（超时或值超出范围）
      */
    bool measure_phase_inductance(Motor_t* motor, float voltage_low, float voltage_high);

    /**
      * @brief 编码器偏移校准
      * 
      * 通过施加已知方向的电压矢量使电机旋转到特定电气角度，
      * 同时读取编码器值，计算机械角度与电气角度的偏移量。
      * 
      * @param[in] motor           电机实例指针
      * @param[in] voltage_magnitude 校准电压幅值，单位伏特[V]
      * @return true               校准成功
      * @return false              校准失败
      */
    bool calib_enc_offset(Motor_t* motor, float voltage_magnitude);

    /**
      * @brief 搜索编码器索引脉冲
      * 
      * 控制电机旋转一周，寻找编码器的Z信号（索引脉冲），
      * 用于建立绝对位置参考。
      * 
      * @param[in] motor   电机实例指针
      * @param[in] v_d     d轴电压，单位伏特[V]
      * @param[in] v_q     q轴电压，单位伏特[V]，控制旋转速度
      * @return true       成功找到索引脉冲
      * @return false      搜索失败（超时）
      */
    bool scan_for_enc_idx(Motor_t* motor, float v_d, float v_q);

    /* ===== 测试函数 ===== */

    /**
      * @brief 电机扫描循环（测试用）
      * 
      * 以正弦方式驱动电机旋转，用于测试和诊断。
      * 通过扫描不同频率和电压，可以分析电机的响应特性。
      * 
      * @param[in] motor            电机实例指针
      * @param[in] omega            扫描角频率，单位[rad/s]
      * @param[in] voltage_magnitude 电压幅值，单位伏特[V]
      */
    void scan_motor_loop(Motor_t* motor, float omega, float voltage_magnitude);

    /* ===== 主控制循环函数 ===== */

    /**
      * @brief 执行控制周期检查
      * 
      * 检查控制循环是否超时，监控CPU使用率，并更新时序日志。
      * 用于确保控制循环在规定的周期内完成。
      * 
      * @param[in] motor   电机实例指针
      * @return true       检查通过，控制周期正常
      * @return false      检查失败，可能发生超时
      */
    bool do_checks(Motor_t* motor);

    /**
      * @brief 执行控制循环更新
      * 
      * 处理每个控制周期的输入更新，包括：
      *   - 读取新的设定值
      *   - 更新传感器数据
      *   - 处理控制模式切换
      * 
      * @param[in] motor   电机实例指针
      * @return true       更新成功
      * @return false      更新失败
      */
    bool loop_updates(Motor_t* motor);

    /**
      * @brief 更新转子状态
      * 
      * 根据当前的转子模式（编码器/无传感器）更新转子位置和速度信息。
      * 这是FOC控制的关键步骤，需要准确的转子位置进行坐标变换。
      * 
      * @param[in] motor   电机实例指针
      */
    void update_rotor(Motor_t* motor);

    /**
      * @brief 检查是否使用编码器模式
      * 
      * @param[in] motor   电机实例指针
      * @return true       当前使用编码器获取转子位置
      * @return false      当前未使用编码器
      */
    bool using_encoder(Motor_t* motor);

    /**
      * @brief 检查是否使用无传感器模式
      * 
      * @param[in] motor   电机实例指针
      * @return true       当前使用无传感器估算器获取转子位置
      * @return false      当前未使用无传感器模式
      */
    bool using_sensorless(Motor_t* motor);

    /**
      * @brief 获取转子相位
      * 
      * 根据当前转子模式返回转子的电气角度相位。
      * 
      * @param[in] motor   电机实例指针
      * @return float      转子电气角度相位，单位弧度[rad]
      */
    float get_rotor_phase(Motor_t* motor);

    /**
      * @brief 获取PLL估算速度
      * 
      * 返回锁相环估算的转子速度。
      * 
      * @param[in] motor   电机实例指针
      * @return float      PLL估算速度，单位[rad/s]
      */
    float get_pll_vel(Motor_t* motor);

    /**
      * @brief 无传感器模式启动
      * 
      * 在无传感器模式下，通过开环启动（"启动斜坡"）使电机达到
      * 最低速度，以便反电动势观测器能够正常工作。
      * 
      * @param[in] motor   电机实例指针
      * @return true       启动成功，已达到目标速度
      * @return false      启动失败（超时）
      */
    bool spin_up_sensorless(Motor_t* motor);

    /**
      * @brief 更新制动电流
      * 
      * 根据当前母线电压和制动电阻值，自动计算并设置制动电流，
      * 以维持母线电压在安全范围内。
      */
    void update_brake_current();

    /**
      * @brief 设置制动电流
      * 
      * 手动设置制动电阻的消耗电流。
      * 
      * @param[in] brake_current   制动电流值，单位安培[A]
      */
    void set_brake_current(float brake_current);

    /**
      * @brief 排队PWM调制时序
      * 
      * 将αβ坐标系下的调制信号转换为PWM定时器的比较值，
      * 并排队到下一个PWM周期更新。
      * 
      * @param[in] motor      电机实例指针
      * @param[in] mod_alpha  α轴调制信号（归一化）
      * @param[in] mod_beta   β轴调制信号（归一化）
      */
    void queue_modulation_timings(Motor_t* motor, float mod_alpha, float mod_beta);

    /**
      * @brief 排队电压矢量时序
      * 
      * 将αβ坐标系下的电压矢量转换为PWM定时器的比较值，
      * 并排队到下一个PWM周期更新。与调制时序不同，此函数
      * 直接接受电压值而非归一化的调制信号。
      * 
      * @param[in] motor     电机实例指针
      * @param[in] v_alpha   α轴电压，单位伏特[V]
      * @param[in] v_beta    β轴电压，单位伏特[V]
      */
    void queue_voltage_timings(Motor_t* motor, float v_alpha, float v_beta);

    /**
      * @brief FOC电压控制
      * 
      * 在dq坐标系下施加指定的d轴和q轴电压。
      * 用于云台电机模式或开环电压控制。
      * 
      * @param[in] motor   电机实例指针
      * @param[in] v_d     d轴电压设定值，单位伏特[V]
      * @param[in] v_q     q轴电压设定值，单位伏特[V]
      * @return true       控制成功
      * @return false      控制失败
      */
    bool FOC_voltage(Motor_t* motor, float v_d, float v_q);

    /**
      * @brief FOC电流控制
      * 
      * 执行完整的FOC电流控制循环，包括：
      *   - Clarke变换（三相→αβ）
      *   - Park变换（αβ→dq）
      *   - PI电流调节
      *   - 反Park变换（dq→αβ）
      *   - SVPWM调制
      * 
      * @param[in] motor    电机实例指针
      * @param[in] Id_des   d轴目标电流，单位安培[A]，通常设为0（弱磁控制除外）
      * @param[in] Iq_des   q轴目标电流，单位安培[A]，决定输出转矩
      * @return true        控制成功
      * @return false       控制失败
      */
    bool FOC_current(Motor_t* motor, float Id_des, float Iq_des);

    /**
      * @brief 电机控制主循环
      * 
      * 电机控制线程的主循环函数，在每个控制周期中依次执行：
      *   1. 时序和状态检查
      *   2. 输入更新（设定值、传感器数据）
      *   3. 转子位置更新
      *   4. 根据控制模式执行相应的控制算法
      *   5. 输出PWM调制信号
      * 
      * 此函数由RTOS线程周期性调用，是电机控制的核心。
      * 
      * @param[in] motor   电机实例指针
      */
    void control_motor_loop(Motor_t* motor);

    /**
      * @brief 电机控制线程
      * 
      * FreeRTOS电机控制线程的入口函数。
      * 线程创建后在此函数中执行初始化，然后进入控制主循环。
      * 
      * @param[in] argument  线程参数，通常为电机实例指针（Motor_t*）
      */
    void motor_thread(void * argument);

#ifdef __cplusplus
}
#endif

#endif //__LOW_LEVEL_H
