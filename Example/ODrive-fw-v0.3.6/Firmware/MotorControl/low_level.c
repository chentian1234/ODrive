/* Includes ------------------------------------------------------------------*/
/*
 * 本文件是ODrive电机控制的核心实现，包含完整的FOC（磁场定向控制）算法。
 *
 * 主要功能模块：
 *   1. 电机初始化（DRV8301门极驱动、ADC/PWM配置、定时器同步）
 *   2. 电机参数测量（相电阻、相电感、编码器偏移校准）
 *   3. FOC电流控制（Clarke变换 → Park变换 → PI控制器 → 反Park变换 → SVM空间矢量调制）
 *   4. 位置/速度/电流三环控制
 *   5. 转子位置观测（编码器PLL锁相环 + 无感磁链观测器）
 *   6. 制动电阻控制
 *   7. 防齿槽转矩补偿（Anti-cogging）
 *
 * FOC控制原理概述：
 *   通过坐标变换将三相交流电流转换为旋转坐标系下的直流量(Id, Iq)，
 *   其中Id控制磁通，Iq控制转矩。解耦后可以用简单的PI控制器实现高性能控制。
 *
 *   坐标变换链：
 *     三相静止(abc) → 两相静止(alpha-beta) → 两相旋转(d-q)
 *     Clarke变换                 Park变换
 *
 *   SVM空间矢量调制：
 *     将(d-q)坐标系下的电压指令转换为PWM占空比，控制三相MOSFET桥臂。
 */

// Because of broken cmsis_os.h, we need to include arm_math first,
// otherwise chip specific defines are ommited
#include <stm32f405xx.h>
#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#include <arm_math.h>

#include <low_level.h>

#include <cmsis_os.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <adc.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <utils.h>

#include "main.h"

#include "commands_pro.h"

/* Private defines -----------------------------------------------------------*/

#define STANDALONE_MODE // Drive operates without USB communication
#define DEBUG_PRINT

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
/**
 * 直流母线电压 [V]
 * 由ADC实时采样获得。初始值设为12V，防止ADC采样未就绪时出现除以零错误。
 * 该值用于：
 *   - SVM调制时计算电压-占空比转换因子
 *   - 制动电阻控制计算
 *   - 低压欠压保护（brownout）
 */
// This value is updated by the DC-bus reading ADC.
// Arbitrary non-zero inital value to avoid division by zero if ADC reading is late
float vbus_voltage = 12.0f;

#if HW_VERSION_MAJOR == 3
#if HW_VERSION_MINOR <= 3
#define SHUNT_RESISTANCE (675e-6f)
#else
//#define SHUNT_RESISTANCE (500e-6f)
#define SHUNT_RESISTANCE (750e-6f)
#endif
#endif
// TODO stick parameter into struct
//#define ENCODER_CPR (600*4)
//#define POLE_PAIRS 7
//static float elec_rad_per_enc = POLE_PAIRS * 2 * M_PI * (1.0f / (float)ENCODER_CPR);


/**
 * ================================================================================
 * ODrive 双电机配置表
 * ================================================================================
 * ODrive支持双电机同时控制，motors[]数组包含两个Motor_t对象。
 *   M0（电机0）: 使用 TIM1(PWM) + TIM3(编码器) + ADC2/ADC3(电流采样)
 *   M1（电机1）: 使用 TIM8(PWM) + TIM4(编码器) + ADC2/ADC3(电流采样)
 *
 * 关键参数说明：
 *   control_mode         : 控制模式（位置/速度/电流环）
 *   pole_pairs           : 电机极对数，N5065和Turnigy SK3系列为7
 *   pos_gain             : 位置环比例增益 [(counts/s) / counts]
 *   vel_gain             : 速度环比例增益 [A/(counts/s)] 或 [A/(rad/s)]
 *   vel_integrator_gain  : 速度环积分增益 [A/(counts/s * s)]
 *   current_lim          : 电流限制 [A]
 *   calibration_current  : 校准时使用的测试电流 [A]
 *   phase_resistance     : 相电阻 [ohm]，由measure_phase_resistance()测量
 *   phase_inductance     : 相电感 [H]，由measure_phase_inductance()测量
 *   shunt_conductance    : 分流器电导 [S] = 1/SHUNT_RESISTANCE
 *
 * 控制环结构（三环级联）：
 *   位置环(外环) → 速度环(中环) → 电流环(内环) → SVM → PWM → 电机
 *
 *   位置环: vel_des = pos_gain × (pos_setpoint - pos_actual)
 *   速度环: Iq_des = vel_gain × (vel_des - vel_actual) + vel_integrator × ∫vel_error
 *   电流环: Vd/Vq = PI(Id_error/Iq_error) → Park逆变换 → SVM空间矢量调制
 *
 * 注意：云台电机(gimbal)模式下，电流单位改为电压单位。
 *   例如: vel_gain 变为 [V/(count/s)]，current_lim 决定最大输出电压。
 */
// TODO: Migrate to C++, clearly we are actually doing object oriented code here...
// TODO: For nice encapsulation, consider not having the motor objects public
Motor_t motors[] = {
    {
//        .control_mode = CTRL_MODE_POSITION_CONTROL, //see: Motor_control_mode_t
        .control_mode = CTRL_MODE_VELOCITY_CONTROL, //see: Motor_control_mode_t
        .enable_step_dir = false,                    //auto enabled after calibration
        .counts_per_step = 2.0f,
        .error = ERROR_NO_ERROR,
        .pole_pairs = 7, // This value is correct for N5065 motors and Turnigy SK3 series.
        .pos_setpoint = 0.0f,
        .pos_gain = 20.0f,  // [(counts/s) / counts]
        .vel_setpoint = 0.0f,
        // .vel_setpoint = 800.0f, <sensorless example>
        //.vel_gain = 15.0f / 10000.0f,  // [A/(counts/s)]
//      .vel_gain = 15.0f / 200.0f, // [A/(rad/s)] <sensorless example>
        .vel_gain = 15.0f / 1000.0f, // [A/(rad/s)] <sensorless example>
//        .vel_integrator_gain = 10.0f / 10000.0f,  // [A/(counts/s * s)]
//        .vel_integrator_gain = 0.0f, // [A/(rad/s * s)] <sensorless example>
				.vel_integrator_gain = 1.0f, // [A/(rad/s * s)] <sensorless example>
        .vel_integrator_current = 0.0f,  // [A]
        .vel_limit = 20000.0f,           // [counts/s]
        .current_setpoint = 0.0f,        // [A]
        .calibration_current = 10.0f,    // [A]
        .resistance_calib_max_voltage = 1.0f, // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
        .dc_bus_brownout_trip_level = 8.0f, // [V]
        .phase_inductance = 0.0f,        // to be set by measure_phase_inductance
        .phase_resistance = 0.0f,        // to be set by measure_phase_resistance
        .motor_thread = 0,
        .thread_ready = false,
        .enable_control = true,
        .do_calibration = false,
        .calibration_ok = false,
        .motor_timer = &htim1,
        .next_timings = { TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2 },
        .control_deadline = TIM_1_8_PERIOD_CLOCKS,
        .last_cpu_time = 0,
        .current_meas = { 0.0f, 0.0f },
        .DC_calib = { 0.0f, 0.0f },
        .gate_driver = {
            .spiHandle = &hspi3,
            // Note: this board has the EN_Gate pin shared!
            .EngpioHandle = EN_GATE_GPIO_Port,
            .EngpioNumber = EN_GATE_Pin,
            .nCSgpioHandle = M0_nCS_GPIO_Port,
            .nCSgpioNumber = M0_nCS_Pin,
            .RxTimeOut = false,
            .enableTimeOut = false,
        },
        // .gate_driver_regs Init by DRV8301_setup
        .motor_type = MOTOR_TYPE_HIGH_CURRENT,
        // .motor_type = MOTOR_TYPE_GIMBAL,
        .shunt_conductance = 1.0f / SHUNT_RESISTANCE,  //[S]
        .phase_current_rev_gain = 0.0f,                // to be set by DRV8301_setup
        .current_control = {
            // Read out max_allowed_current to see max supported value for current_lim.
            // You can change DRV8301_ShuntAmpGain to get a different range.
            // .current_lim = 75.0f, //[A]
            .current_lim = 10.0f,  //[A]
            .p_gain = 0.0f,        // [V/A] should be auto set after resistance and inductance measurement
            .i_gain = 0.0f,        // [V/As] should be auto set after resistance and inductance measurement
            .v_current_control_integral_d = 0.0f,
            .v_current_control_integral_q = 0.0f,
            .Ibus = 0.0f,
            .final_v_alpha = 0.0f,
            .final_v_beta = 0.0f,
            .Iq_setpoint = 0.0f,
            .Iq_measured = 0.0f,
            .max_allowed_current = 0.0f,
        },
        .rotor_mode = ROTOR_MODE_SENSORLESS,
        // .rotor_mode = ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS,
        //.rotor_mode = ROTOR_MODE_ENCODER,
        .encoder = {
            .encoder_timer = &htim3,
            .use_index = false,
            .index_found = false,
            .manually_calibrated = false,
            .idx_search_speed = 10.0f, // [rad/s electrical]
            .encoder_cpr = (600 * 4), // Default resolution of CUI-AMT102 encoder,
            .encoder_offset = 0,
            .encoder_state = 0,
            .motor_dir = 1,   // 1 or -1
            .encoder_calib_range = 0.02,
            .phase = 0.0f,    // [rad]
            .pll_pos = 0.0f,  // [rad]
            .pll_vel = 0.0f,  // [rad/s]
            .pll_kp = 0.0f,   // [rad/s / rad]
            .pll_ki = 0.0f,   // [(rad/s^2) / rad]
        },
        .sensorless = {
            .phase = 0.0f,                        // [rad]
            .pll_pos = 0.0f,                      // [rad]
            .pll_vel = 0.0f,                      // [rad/s]
            .pll_kp = 0.0f,                       // [rad/s / rad]
            .pll_ki = 0.0f,                       // [(rad/s^2) / rad]
            .observer_gain = 1000.0f,             // [rad/s]
            .flux_state = { 0.0f, 0.0f },           // [Vs]
            .V_alpha_beta_memory = { 0.0f, 0.0f },  // [V]
//            .pm_flux_linkage = 1.58e-3f,          // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
						.pm_flux_linkage = 1.43e-3f,          // [V / (rad/s)]  { 5.51328895422 / (7 * 550) }
            .estimator_good = false,
            .spin_up_current = 10.0f,        // [A]
            .spin_up_acceleration = 400.0f,  // [rad/s^2]
            .spin_up_target_vel = 400.0f,    // [rad/s]
        },
        .loop_counter = 0,
        .timing_log = { 0 },
        .anticogging = {
            .index = 0,
            .cogging_map = NULL,
            .use_anticogging = false,
            .calib_anticogging = false,
            .calib_pos_threshold = 1.0f,
            .calib_vel_threshold = 1.0f,
        },
        .drv_fault = DRV8301_FaultType_NoFault,
    },
    {                                             // M1
        .control_mode = CTRL_MODE_POSITION_CONTROL,  //see: Motor_control_mode_t
        .enable_step_dir = false,                    //auto enabled after calibration
        .counts_per_step = 2.0f,
        .error = ERROR_NO_ERROR,
        .pole_pairs = 7, // This value is correct for N5065 motors and Turnigy SK3 series.
        .pos_setpoint = 0.0f,
        .pos_gain = 20.0f,  // [(counts/s) / counts]
        .vel_setpoint = 0.0f,
        .vel_gain = 15.0f / 10000.0f,             // [A/(counts/s)]
        .vel_integrator_gain = 10.0f / 10000.0f,  // [A/(counts/s * s)]
        .vel_integrator_current = 0.0f,           // [A]
        .vel_limit = 20000.0f,                    // [counts/s]
        .current_setpoint = 0.0f,                 // [A]
        .calibration_current = 10.0f,             // [A]
        .resistance_calib_max_voltage = 1.0f, // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
        .dc_bus_brownout_trip_level = 8.0f, // [V]
        .phase_inductance = 0.0f,                 // to be set by measure_phase_inductance
        .phase_resistance = 0.0f,                 // to be set by measure_phase_resistance
        .motor_thread = 0,
        .thread_ready = false,
        .enable_control = true,
        .do_calibration = false,
        .calibration_ok = false,
        .motor_timer = &htim8,
        .next_timings = { TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2 },
        .control_deadline = (3 * TIM_1_8_PERIOD_CLOCKS) / 2,
        .last_cpu_time = 0,
        .current_meas = { 0.0f, 0.0f },
        .DC_calib = { 0.0f, 0.0f },
        .gate_driver = {
            .spiHandle = &hspi3,
            // Note: this board has the EN_Gate pin shared!
            .EngpioHandle = EN_GATE_GPIO_Port,
            .EngpioNumber = EN_GATE_Pin,
            .nCSgpioHandle = M1_nCS_GPIO_Port,
            .nCSgpioNumber = M1_nCS_Pin,
            .RxTimeOut = false,
            .enableTimeOut = false,
        },
        // .gate_driver_regs Init by DRV8301_setup
        .motor_type = MOTOR_TYPE_HIGH_CURRENT,
        .shunt_conductance = 1.0f / SHUNT_RESISTANCE,  //[S]
        .phase_current_rev_gain = 0.0f,                // to be set by DRV8301_setup
        .current_control = {
            // Read out max_allowed_current to see max supported value for current_lim.
            // You can change DRV8301_ShuntAmpGain to get a different range.
            // .current_lim = 75.0f, //[A]
            .current_lim = 10.0f,  //[A]
            .p_gain = 0.0f,        // [V/A] should be auto set after resistance and inductance measurement
            .i_gain = 0.0f,        // [V/As] should be auto set after resistance and inductance measurement
            .v_current_control_integral_d = 0.0f,
            .v_current_control_integral_q = 0.0f,
            .Ibus = 0.0f,
            .final_v_alpha = 0.0f,
            .final_v_beta = 0.0f,
            .Iq_setpoint = 0.0f,
            .Iq_measured = 0.0f,
            .max_allowed_current = 0.0f,
        },
        .rotor_mode = ROTOR_MODE_SENSORLESS,
        // .rotor_mode = ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS,
        //.rotor_mode = ROTOR_MODE_ENCODER,
        .encoder = {
            .encoder_timer = &htim4,
            .use_index = false,
            .index_found = false,
            .manually_calibrated = false,
            .idx_search_speed = 10.0f, // [rad/s electrical]
            .encoder_cpr = (600 * 4), // Default resolution of CUI-AMT102 encoder,
            .encoder_offset = 0,
            .encoder_state = 0,
            .motor_dir = 1,   // 1 or -1
            .encoder_calib_range = 0.02,
            .phase = 0.0f,    // [rad]
            .pll_pos = 0.0f,  // [rad]
            .pll_vel = 0.0f,  // [rad/s]
            .pll_kp = 0.0f,   // [rad/s / rad]
            .pll_ki = 0.0f,   // [(rad/s^2) / rad]
        },
        .sensorless = {
            .phase = 0.0f,                        // [rad]
            .pll_pos = 0.0f,                      // [rad]
            .pll_vel = 0.0f,                      // [rad/s]
            .pll_kp = 0.0f,                       // [rad/s / rad]
            .pll_ki = 0.0f,                       // [(rad/s^2) / rad]
            .observer_gain = 1000.0f,             // [rad/s]
            .flux_state = { 0.0f, 0.0f },           // [Vs]
            .V_alpha_beta_memory = { 0.0f, 0.0f },  // [V]
            .pm_flux_linkage = 1.58e-3f,          // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
            .estimator_good = false,
            .spin_up_current = 10.0f,        // [A]
            .spin_up_acceleration = 400.0f,  // [rad/s^2]
            .spin_up_target_vel = 400.0f,    // [rad/s]
        },
        .loop_counter = 0,
        .timing_log = { 0 },
        .anticogging = {
            .index = 0,
            .cogging_map = NULL,
            .use_anticogging = false,
            .calib_anticogging = false,
            .calib_pos_threshold = 1.0f,
            .calib_vel_threshold = 1.0f,
        },
        .drv_fault = DRV8301_FaultType_NoFault,
    }
};
/** 电机数量，通过数组大小自动计算，当前为2（双电机驱动） */
const size_t num_motors = sizeof(motors) / sizeof(motors[0]);

/**
 * 制动电阻阻值 [Ω]
 * 当电机处于再生发电状态（减速或外部拖动）时，电能回馈到直流母线，
 * 导致母线电压升高。制动电阻将多余能量以热能形式耗散。
 * 典型值0.47Ω，需根据实际功率需求选择。
 */
float brake_resistance = 0.47f;     // [ohm]

/* Private constant data -----------------------------------------------------*/

/** 1/√3 ≈ 0.577，用于Clarke变换和Park变换 */
static const float one_by_sqrt3 = 0.57735026919f;

/** √3/2 ≈ 0.866，用于SVM空间矢量调制中的矢量幅值计算 */
static const float sqrt3_by_2 = 0.86602540378f;

/** 电流采样周期 [秒]，由硬件定时器配置决定（通常为8kHz或16kHz） */
static const float current_meas_period = CURRENT_MEAS_PERIOD;

/** 电流采样频率 [Hz]，current_meas_hz = 1 / current_meas_period */
static const int current_meas_hz = CURRENT_MEAS_HZ;

/* Private variables ---------------------------------------------------------*/
/* Function implementations --------------------------------------------------*/

//--------------------------------
// 命令处理 (Command Handling)
//--------------------------------
/**
 * @brief 设置位置控制目标
 * @param motor 电机对象指针
 * @param pos_setpoint 目标位置 [counts]
 * @param vel_feed_forward 速度前馈 [counts/s]，用于改善动态响应
 * @param current_feed_forward 电流前馈 [A]，用于补偿已知负载
 * 
 * 设置电机进入位置控制模式。位置环的输出作为速度环的给定，
 * 速度环的输出作为电流环的给定。前馈项可显著提高跟踪精度。
 */
void set_pos_setpoint(Motor_t *motor, float pos_setpoint, float vel_feed_forward, float current_feed_forward) {
    motor->pos_setpoint = pos_setpoint;
    motor->vel_setpoint = vel_feed_forward;
    motor->current_setpoint = current_feed_forward;
    motor->control_mode = CTRL_MODE_POSITION_CONTROL;
#ifdef DEBUG_PRINT
    commands_printf("%-32s : oked\n", "POSITION_CONTROL");
    commands_printf("%-32s : %6.0f\n", "pos_setpoint", motor->pos_setpoint);
    commands_printf("%-32s : %3.3f\n", "vel_setpoint", motor->vel_setpoint);
    commands_printf("%-32s : %3.3f\n", "current_setpoint", motor->current_setpoint);
#endif
}

/**
 * @brief 设置速度控制目标
 * @param motor 电机对象指针
 * @param vel_setpoint 目标速度 [counts/s 或 rad/s]
 * @param current_feed_forward 电流前馈 [A]
 * 
 * 设置电机进入速度控制模式。速度环直接输出电流指令给电流环。
 */
void set_vel_setpoint(Motor_t *motor, float vel_setpoint, float current_feed_forward) {
    motor->vel_setpoint = vel_setpoint;
    motor->current_setpoint = current_feed_forward;
    motor->control_mode = CTRL_MODE_VELOCITY_CONTROL;
#ifdef DEBUG_PRINT
    commands_printf("%-32s : oked\n", "VELOCITY_CONTROL");
    commands_printf("%-32s : %3.3f\n", "vel_setpoint", motor->vel_setpoint);
    commands_printf("%-32s : %3.3f\n", "current_setpoint", motor->current_setpoint);
#endif
}

/**
 * @brief 设置电流控制目标
 * @param motor 电机对象指针
 * @param current_setpoint 目标电流 [A]
 * 
 * 设置电机进入纯电流控制模式。直接给定Iq电流指令，
 * 不经过位置和速度环，适用于转矩直接控制场景。
 */
void set_current_setpoint(Motor_t *motor, float current_setpoint) {
    motor->current_setpoint = current_setpoint;
    motor->control_mode = CTRL_MODE_CURRENT_CONTROL;
#ifdef DEBUG_PRINT
    commands_printf("%-32s : oked\n", "CURRENT_CONTROL");
    commands_printf("%-32s : %3.3f\n", "current_setpoint", motor->current_setpoint);
#endif
}

//--------------------------------
// 工具函数 (Utility)
//--------------------------------
/**
 * @brief 检查定时器计数值，记录当前时刻相对于PWM周期的相位
 * @param motor 电机对象指针
 * @param log_idx 时序日志索引
 * @return 归一化后的定时值 [clocks]
 * 
 * 本函数读取当前定时器CNT值，考虑中心对齐模式下的上下计数方向，
 * 将计数值归一化到一个连续的数值范围。用于调试FOC控制时序是否满足deadline。
 * 
 * 中心对齐PWM模式下：
 *   - 向上计数阶段（DIR=0）：CNT从0递增到ARR
 *   - 向下计数阶段（DIR=1）：CNT从ARR递减到0
 * 本函数将两种情况映射到统一的绝对时间轴上。
 */
uint16_t check_timing(Motor_t *motor, TimingLog_t log_idx) {
    TIM_HandleTypeDef *htim = motor->motor_timer;
    uint16_t timing = htim->Instance->CNT;
    bool down = htim->Instance->CR1 & TIM_CR1_DIR;
    if (down) {
        uint16_t delta = TIM_1_8_PERIOD_CLOCKS - timing;
        timing = TIM_1_8_PERIOD_CLOCKS + delta;
    }

    if (log_idx < TIMING_LOG_SIZE) {
        motor->timing_log[log_idx] = timing;
    }

    return timing;
}

/**
 * @brief 全局故障处理 - 立即停止所有电机
 * @param error 故障代码
 * 
 * 当发生严重故障（如过流、欠压、ADC失败等）时，
 * 本函数立即关闭所有电机的PWM输出（MOE=0），切断MOSFET驱动，
 * 并设置所有电机的故障标志，停止制动电阻。
 * 
 * 安全关键函数：此操作必须尽可能快地执行，确保电机安全。
 */
void global_fault(int error) {
    // Disable motors NOW!
    for (int i = 0; i < num_motors; ++i) {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motors[i].motor_timer);
    }
    // Set fault codes, etc.
    for (int i = 0; i < num_motors; ++i) {
        motors[i].error = error;
//        *(motors[i].axis_legacy.enable_control) = false;
        motors[i].enable_control = false;
    }
    // disable brake resistor
    set_brake_current(0.0f);
}

/**
 * @brief 将ADC采样值转换为相电流值 [A]
 * @param motor 电机对象指针
 * @param ADCValue ADC原始采样值（12位，0~4095）
 * @return 相电流值 [A]
 * 
 * 转换原理：
 *   1. ADC读取的是运放输出电压，中心点为Vref/2 = 1.65V（对应ADC值2048 = 1<<11）
 *   2. adcval_bal = ADCValue - 2048，得到偏移量（有符号）
 *   3. amp_out_volt = (3.3V / 4096) × adcval_bal，转换为运放输出电压
 *   4. shunt_volt = amp_out_volt × phase_current_rev_gain，去除DRV8301放大倍数
 *      phase_current_rev_gain = 1/Gain，其中Gain由DRV8301配置（10/20/40/80 V/V）
 *   5. current = shunt_volt × shunt_conductance，根据分流器电阻计算电流
 *      shunt_conductance = 1 / SHUNT_RESISTANCE
 * 
 * 信号链：
 *   相电流 → 分流器(产生mV级电压) → DRV8301放大 → ADC采样 → 软件转换
 */
float phase_current_from_adcval(Motor_t *motor, uint32_t ADCValue) {
    int adcval_bal = (int)ADCValue - (1 << 11);
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * motor->phase_current_rev_gain;
    float current = shunt_volt * motor->shunt_conductance;
    return current;
}

//--------------------------------
// 初始化 (Initialization)
//--------------------------------
/**
 * @brief 电机控制底层初始化
 * 
 * 本函数完成电机控制系统的全部初始化，包括：
 *   1. DRV8301门极驱动器配置（SPI通信、过流保护、增益设置）
 *   2. ADC和PWM启动（中心对齐PWM、ADC注入/规则转换、中断使能）
 *   3. TIM1和TIM8定时器同步（确保双电机相位对齐）
 *   4. 编码器接口启动（TIM3/TIM4编码器模式）
 *   5. 等待电流采样DC_CAL滤波收敛（约1.5秒）
 * 
 * 初始化流程时序：
 *   DRV8301_setup() → start_adc_pwm() → HAL_TIM_Encoder_Start() → osDelay(1500ms)
 * 
 * 注意：初始化完成后，PWM输出仍处于禁用状态(MOE=0)，
 *       需要在校准完成后手动使能。
 */
// Initalises the low level motor control and then starts the motor control threads
void init_motor_control() {
    // Init gate drivers
    DRV8301_setup(&motors[0]);
    DRV8301_setup(&motors[1]);

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();

    // Start Encoders
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    //TODO: Enable index on only one channel
    if (motors[0].encoder.use_index || motors[1].encoder.use_index) {
        SetupENCIndexGPIO();
    }

    // Wait for current sense calibration to converge
    // TODO make timing a function of calibration filter tau
    osDelay(1500);
}

/**
 * @brief 配置DRV8301门极驱动器
 * @param motor 电机对象指针
 * 
 * DRV8301是TI的三相MOSFET预驱动芯片，负责：
 *   - 将MCU的3.3V PWM信号转换为MOSFET门极驱动电压
 *   - 提供电流采样放大器（shunt amplifier）
 *   - 过流保护（OC保护）
 * 
 * 配置内容：
 *   1. 使能DRV8301（通过EN_GATE引脚）
 *   2. SPI初始化，建立与DRV8301的通信
 *   3. OC_MODE = LatchShutDown：过流时锁存关断，需要SPI清除
 *   4. OC_ADJ_SET = 0.730V：过流阈值约150A（@100°C）
 *      Vds检测阈值设置，通过检测MOSFET导通压降判断过流
 *   5. GAIN = 40VpV：分流放大器增益40V/V
 *      配合750μΩ分流器，电流测量范围：±55A
 *      配合500μΩ分流器，电流测量范围：±75A
 *      增益越高，测量范围越小但精度越高
 *   6. 计算max_allowed_current：根据硬件参数计算最大允许电流
 *      max_input = margin × 0.3 × shunt_conductance    (ADC输入限制)
 *      max_swing = margin × 1.6 × shunt_conductance × phase_current_rev_gain  (放大器摆幅限制)
 *   7. 通过SPI写入配置并回读验证
 * 
 * 增益与量程对应关系：
 *   Gain=10V/V → 分流750μΩ → ±400A（量程大，精度低）
 *   Gain=20V/V → 分流750μΩ → ±200A
 *   Gain=40V/V → 分流750μΩ → ±100A（本配置）
 *   Gain=80V/V → 分流750μΩ → ±50A（量程小，精度高）
 */
// Set up the gate drivers
void DRV8301_setup(Motor_t *motor) {
    DRV8301_Obj *gate_driver = &motor->gate_driver;
    DRV_SPI_8301_Vars_t *local_regs = &motor->gate_driver_regs;

    DRV8301_enable(gate_driver);
    DRV8301_setupSpi(gate_driver, local_regs);

    // TODO we can use reporting only if we actually wire up the nOCTW pin
    local_regs->Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
    // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
    local_regs->Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A
    local_regs->Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_40VpV;
    // local_regs->Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_20VpV;

    switch (local_regs->Ctrl_Reg_2.GAIN) {
    case DRV8301_ShuntAmpGain_10VpV:
        motor->phase_current_rev_gain = 1.0f / 10.0f;
        break;
    case DRV8301_ShuntAmpGain_20VpV:
        motor->phase_current_rev_gain = 1.0f / 20.0f;
        break;
    case DRV8301_ShuntAmpGain_40VpV:
        motor->phase_current_rev_gain = 1.0f / 40.0f;
        break;
    case DRV8301_ShuntAmpGain_80VpV:
        motor->phase_current_rev_gain = 1.0f / 80.0f;
        break;
    }

    float margin = 0.90f;
    float max_input = margin * 0.3f * motor->shunt_conductance;
    float max_swing = margin * 1.6f * motor->shunt_conductance * motor->phase_current_rev_gain;
    motor->current_control.max_allowed_current = MACRO_MIN(max_input, max_swing);

    local_regs->SndCmd = true;
    DRV8301_writeData(gate_driver, local_regs);
    local_regs->RcvCmd = true;
    DRV8301_readData(gate_driver, local_regs);
}

/**
 * @brief 启动ADC和PWM，建立电流采样与PWM调制的同步关系
 * 
 * 这是整个电机控制系统最关键的低层配置，建立了：
 *   1. ADC转换链：hadc1(母线电压) + hadc2(M0/M1相B电流) + hadc3(M0/M1相C电流)
 *   2. PWM输出链：htim1(M0三相PWM) + htim8(M1三相PWM)
 *   3. 定时器同步：TIM1为主定时器，TIM8为从定时器，通过ITR0触发同步
 *   4. ADC触发机制：PWM的CCR4比较事件触发ADC注入/规则转换
 *   5. 制动电阻PWM：htim2通道3/4控制制动电阻
 * 
 * 双电机ADC/PWM复用机制：
 *   - TIM1触发ADC2/3的注入转换(injected) → 电机0
 *   - TIM8触发ADC2/3的规则转换(regular) → 电机1
 *   - 利用中心对齐PWM的上下计数阶段区分电流采样和DC_CAL采样：
 *     向上计数(SVM矢量0)：采样真实电流
 *     向下计数(SVM矢量7)：采样DC偏移（零电流状态）
 * 
 * 安全特性：
 *   - 调试暂停时自动冻结TIM1/TIM8，防止调试时PWM继续运行
 *   - 初始化后PWM输出禁用(MOE=0)，需校准后手动使能
 *   - 制动电阻PWM配置为浮动输出（CCR3=0, CCR4=周期+1），确保初始不导通
 */
void start_adc_pwm() {
    // Enable ADC and interrupts
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    // Warp field stabilize.
    osDelay(2);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_EOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_EOC);

    // Ensure that debug halting of the core doesn't leave the motor PWM running
    __HAL_DBGMCU_FREEZE_TIM1();
    __HAL_DBGMCU_FREEZE_TIM8();

    start_pwm(&htim1);
    start_pwm(&htim8);
    // TODO: explain why this offset
    sync_timers(&htim1, &htim8, TIM_CLOCKSOURCE_ITR0, TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128);

    // Motor output starts in the disabled state
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);

    // Start brake resistor PWM in floating output configuration
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

/**
 * @brief 启动单个定时器的PWM输出
 * @param htim 定时器句柄（TIM1或TIM8）
 * 
 * 配置中心对齐PWM模式下的三相桥臂驱动：
 *   - CCR1/CCR2/CCR3初始设为50%占空比（半周期），使三相桥臂处于中间状态
 *   - 每个通道同时启动PWM和互补PWM（PWMN），用于驱动半桥上管和下管
 *   - CCR4设为1，用于触发ADC转换中断（CCR4比较事件 = ADC采样触发点）
 *   - 中心对齐模式下，PWM频率 = 定时器时钟 / (2 × ARR)
 * 
 * 死区时间：
 *   互补PWM输出之间由硬件自动插入死区时间(BDTR寄存器配置)，
 *   防止上下管同时导通导致直通短路。
 */
void start_pwm(TIM_HandleTypeDef *htim) {
    // Init PWM
    int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
    htim->Instance->CCR1 = half_load;
    htim->Instance->CCR2 = half_load;
    htim->Instance->CCR3 = half_load;

    // This hardware obfustication layer really is getting on my nerves
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);

    htim->Instance->CCR4 = 1;
    HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_4);
}

/**
 * @brief 同步两个高级定时器(TIM1和TIM8)，实现双电机PWM相位对齐
 * @param htim_a 主定时器句柄（通常为TIM1）
 * @param htim_b 从定时器句柄（通常为TIM8）
 * @param TIM_CLOCKSOURCE_ITRx 内部触发源选择（ITR0~ITR3）
 * @param count_offset 主定时器计数器偏移值
 * 
 * ============================================================================
 * 双定时器同步机制详解
 * ============================================================================
 * 
 * ODrive需要同时控制两个电机，每个电机由一个高级定时器(TIM1/TIM8)驱动。
 * 为了确保两个电机的PWM波形相位关系确定，必须将两个定时器严格同步。
 * 
 * 同步原理：主从触发模式(Master-Slave Trigger)
 * 
 *   TIM1(主定时器)                    TIM8(从定时器)
 *   ┌──────────────┐                 ┌──────────────┐
 *   │ CR2:MMS=TRGO │───TRGO信号─────▶│ SMCR:TS=ITR0 │
 *   │              │  (计数器使能时)  │ SMS=Trigger  │
 *   └──────────────┘                 └──────────────┘
 * 
 * 同步步骤：
 *   1. 保存原始寄存器状态（BDTR_MOE、CR2、SMCR）
 *   2. 关闭两个定时器的输出（MOE=0），防止同步过程中产生异常PWM
 *   3. 停止两个定时器计数（CEN=0）
 *   4. 配置TIM1为Master模式：CR2.MMS = TRGO，计数器使能时产生TRGO信号
 *   5. 配置TIM8为Slave模式：SMCR.TS = ITR0（选择TIM1的TRGO），
 *                            SMCR.SMS = Trigger（收到触发信号时启动）
 *   6. 暂时关闭中心对齐模式（CMS），因为DIR位在中心对齐模式下只读
 *   7. 将两个定时器都设为向上计数（DIR=0）
 *   8. 恢复中心对齐模式（CMS）
 *   9. 设置计数器初值：
 *      TIM1.CNT = count_offset（偏移半个周期减去128个时钟）
 *      TIM8.CNT = 0
 *      偏移的目的：让两个定时器的ADC采样点错开，避免同时采样
 *   10. 启动TIM1，TIM1的TRGO信号会立即触发TIM8启动
 *   11. 恢复原始寄存器配置
 * 
 * 相位关系：
 *   TIM8相对于TIM1延迟 count_offset 个时钟周期启动。
 *   在中心对齐PWM模式下，这确保了两个电机的电流采样时刻错开，
 *   使ADC有足够时间完成转换。
 * 
 * 时序图：
 *   TIM1: ────┬────CNT=count_offset─────┬────PWM周期开始
 *             │ (立即启动TRGO)
 *   TIM8:     └─────────────────────────┬────CNT=0 (TRGO触发启动)
 */
void sync_timers(TIM_HandleTypeDef *htim_a, TIM_HandleTypeDef *htim_b,
                 uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset) {
    // Store intial timer configs
    uint16_t MOE_store_a = htim_a->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t MOE_store_b = htim_b->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t CR2_store = htim_a->Instance->CR2;
    uint16_t SMCR_store = htim_b->Instance->SMCR;
    // Turn off output
    htim_a->Instance->BDTR &= ~(TIM_BDTR_MOE);
    htim_b->Instance->BDTR &= ~(TIM_BDTR_MOE);
    // Disable both timer counters
    htim_a->Instance->CR1 &= ~TIM_CR1_CEN;
    htim_b->Instance->CR1 &= ~TIM_CR1_CEN;
    // Set first timer to send TRGO on counter enable
    htim_a->Instance->CR2 &= ~TIM_CR2_MMS;
    htim_a->Instance->CR2 |= TIM_TRGO_ENABLE;
    // Set Trigger Source of second timer to the TRGO of the first timer
    htim_b->Instance->SMCR &= ~TIM_SMCR_TS;
    htim_b->Instance->SMCR |= TIM_CLOCKSOURCE_ITRx;
    // Set 2nd timer to start on trigger
    htim_b->Instance->SMCR &= ~TIM_SMCR_SMS;
    htim_b->Instance->SMCR |= TIM_SLAVEMODE_TRIGGER;
    // Dir bit is read only in center aligned mode, so we clear the mode for now
    uint16_t CMS_store_a = htim_a->Instance->CR1 & TIM_CR1_CMS;
    uint16_t CMS_store_b = htim_b->Instance->CR1 & TIM_CR1_CMS;
    htim_a->Instance->CR1 &= ~TIM_CR1_CMS;
    htim_b->Instance->CR1 &= ~TIM_CR1_CMS;
    // Set both timers to up-counting state
    htim_a->Instance->CR1 &= ~TIM_CR1_DIR;
    htim_b->Instance->CR1 &= ~TIM_CR1_DIR;
    // Restore center aligned mode
    htim_a->Instance->CR1 |= CMS_store_a;
    htim_b->Instance->CR1 |= CMS_store_b;
    // set counter offset
    htim_a->Instance->CNT = count_offset;
    htim_b->Instance->CNT = 0;
    // Start Timer a
    htim_a->Instance->CR1 |= (TIM_CR1_CEN);
    // Restore timer configs
    htim_a->Instance->CR2 = CR2_store;
    htim_b->Instance->SMCR = SMCR_store;
    // restore output
    htim_a->Instance->BDTR |= MOE_store_a;
    htim_b->Instance->BDTR |= MOE_store_b;
}

//--------------------------------
// IRQ回调函数 (IRQ Callbacks)
//--------------------------------
/**
 * @brief 步距/方向脉冲回调函数（处理外部步进脉冲输入）
 * @param GPIO_Pin 触发中断的引脚
 * 
 * 当ODrive工作在步进电机模式时，外部控制器通过GPIO发送脉冲信号：
 *   - 脉冲引脚(GPIO_1/ GPIO_3)：每个上升沿触发一次步进
 *   - 方向引脚(GPIO_2/ GPIO_4)：高电平=正向，低电平=反向
 * 
 * 每次步进，位置设定值增加 counts_per_step × 方向系数。
 * 校准完成后自动使能此功能(enable_step_dir = true)。
 */
// step/direction interface
void step_cb(uint16_t GPIO_Pin) {
    GPIO_PinState dir_pin;
    float dir;
    switch (GPIO_Pin) {
    case GPIO_1_Pin:
        //M0 stepped
        if (motors[0].enable_step_dir) {
            dir_pin = HAL_GPIO_ReadPin(GPIO_2_GPIO_Port, GPIO_2_Pin);
            dir = (dir_pin == GPIO_PIN_SET) ? 1.0f : -1.0f;
            motors[0].pos_setpoint += dir * motors[0].counts_per_step;
        }
        break;
    case GPIO_3_Pin:
        //M1 stepped
        if (motors[1].enable_step_dir) {
            dir_pin = HAL_GPIO_ReadPin(GPIO_4_GPIO_Port, GPIO_4_Pin);
            dir = (dir_pin == GPIO_PIN_SET) ? 1.0f : -1.0f;
            motors[1].pos_setpoint += dir * motors[1].counts_per_step;
        }
        break;
    default:
        global_fault(ERROR_UNEXPECTED_STEP_SRC);
        break;
    }
}

/**
 * @brief 编码器Index信号回调函数（Z相脉冲中断处理）
 * @param GPIO_Pin 触发中断的引脚
 * @param motor_index 电机索引（0或1）
 * 
 * 增量式编码器每旋转一圈产生一个Index(Z相)脉冲，用于确定绝对位置参考点。
 * 当编码器经过Index位置时，触发外部中断(EXTI)：
 *   1. 将当前编码器计数值重置为0（建立绝对位置参考）
 *   2. 标记index_found = true
 *   3. 禁用对应的EXTI中断，防止重复触发
 * 
 * 注意：只有在电机校准前尚未找到Index时才会执行重置操作。
 */
// Triggered when an encoder passes over the "Index" pin
// TODO: only arm index edge interrupt when we know encoder has powered up
void enc_index_cb(uint16_t GPIO_Pin, uint8_t motor_index) {
    Motor_t *motor = &motors[motor_index];
    if (!motor->encoder.index_found) {
        setEncoderCount(motor, 0);
        motor->encoder.index_found = true;
    }
    //TODO: Hardcoded EXTI line not portable. Get mapping out of Cubemx by setting EXTI default
    if (GPIO_Pin == M0_ENC_Z_Pin) {
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    } else {
        HAL_NVIC_DisableIRQ(EXTI3_IRQn);
    }
}

/**
 * @brief 直流母线电压ADC采样回调
 * @param hadc ADC句柄
 * @param injected 是否为注入转换（本函数仅用于注入转换）
 * 
 * 读取hadc1的注入转换结果，计算母线电压：
 *   vbus_voltage = ADCValue × 3.3V × VBUS_S_DIVIDER_RATIO / 4096
 * 
 * VBUS_S_DIVIDER_RATIO是母线电压分压比（由硬件电阻分压网络决定）。
 * 母线电压用于：
 *   - SVM调制时的电压归一化
 *   - 欠压保护(brownout)
 *   - 制动电阻控制
 */
void vbus_sense_adc_cb(ADC_HandleTypeDef *hadc, bool injected) {
    static const float voltage_scale = 3.3f * VBUS_S_DIVIDER_RATIO / (float)(1 << 12);
    // Only one conversion in sequence, so only rank1
    uint32_t ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    vbus_voltage = ADCValue * voltage_scale;
}

/**
 * @brief PWM触发ADC采样的回调函数 —— 电流采样的核心枢纽
 * @param hadc 触发中断的ADC句柄（hadc2或hadc3）
 * @param injected 是否为注入转换
 *      - true:  注入转换，由TIM1触发 → 电机0
 *      - false: 规则转换，由TIM8触发 → 电机1
 * 
 * ============================================================================
 * 电流采样时序详解 (PWM-ADC同步机制)
 * ============================================================================
 * 
 * 这是FOC控制中最重要的函数之一，负责在正确的时刻采样电机相电流。
 * 
 * --- 为什么要在特定时刻采样？---
 * 
 * 在SVM空间矢量调制中，每个PWM周期包含8个矢量段：
 *   V0(000) → V1 → V2 → V3 → V7(111) → V3 → V2 → V1 → V0(000)
 * 
 * 在V0(000)矢量期间：所有下管导通，上管关断 → 相电流流经分流电阻 → 可以采样
 * 在V7(111)矢量期间：所有上管导通，下管关断 → 相电流不流经分流电阻 → 采样值为0(DC_CAL)
 * 
 * --- 中心对齐PWM的巧妙设计 ---
 * 
 * 中心对齐PWM在每个周期内有一次向上计数和一次向下计数：
 *   - 向上计数阶段：对应SVM的V0(000)矢量 → 采样得到真实电流值
 *   - 向下计数阶段：对应SVM的V7(111)矢量 → 采样得到零电流(DC_CAL偏移)
 * 
 * 通过检查定时器DIR位判断计数方向，区分真实电流和DC_CAL采样。
 * 
 * --- 双电机复用ADC的时序 ---
 * 
 * ADC2和ADC3同时服务两个电机，通过不同的触发方式和计数方向区分：
 * 
 *   时刻① TIM1向上计数 → ADC2/3注入触发 → 采样M0电流(IphB, IphC)
 *   时刻② TIM1向下计数 → ADC2/3注入触发 → 采样M0 DC_CAL(零电流偏移)
 *   时刻③ TIM8向上计数 → ADC2/3规则触发 → 采样M1电流(IphB, IphC)
 *   时刻④ TIM8向下计数 → ADC2/3规则触发 → 采样M1 DC_CAL(零电流偏移)
 * 
 * 在一个PWM周期内，这4个时刻依次到来，每个时刻触发2个ADC中断(ADC2+ADC3)，
 * 总共8次ADC中断完成两个电机的电流和偏移采样。
 * 
 * --- DC_CAL低通滤波 ---
 * 
 * DC_CAL是电流采样电路的直流偏移（运放失调电压、ADC零点误差等），
 * 需要实时跟踪并减去：
 * 
 *   DC_calib += (current_sample - DC_calib) × calib_filter_k
 * 
 * 其中 calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau
 * calib_tau = 0.2s，是一个低通滤波器的时间常数，
 * 滤波器截止频率 f_c = 1/(2π×τ) ≈ 0.8Hz，只跟踪缓慢变化的直流偏移。
 * 
 * --- 中断处理顺序 ---
 * 
 * ADC2先于ADC3处理（中断优先级或注册顺序）：
 *   1. ADC2中断到达 → 存储IphB → return等待ADC3
 *   2. ADC3中断到达 → 存储IphC → 发送信号给电机线程（电流采样完成）
 * 
 * 当两个相电流都采集完成后，发送M_SIGNAL_PH_CURRENT_MEAS信号唤醒电机控制线程，
 * 触发FOC控制计算。
 * 
 * --- 时序更新 ---
 * 
 * 在采样M0 DC_CAL时，加载M0的下一周期PWM定时(next_timings)
 * 在采样M0电流时，加载M1的下一周期PWM定时(next_timings)
 * 这种交叉更新确保定时更新不会覆盖还未执行的采样。
 */
// This is the callback from the ADC that we expect after the PWM has triggered an ADC conversion.
void pwm_trig_adc_cb(ADC_HandleTypeDef *hadc, bool injected) {
#define calib_tau 0.2f  //@TOTO make more easily configurable
    static const float calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau;

    // Ensure ADCs are expected ones to simplify the logic below
    if (!(hadc == &hadc2 || hadc == &hadc3)) {
        global_fault(ERROR_ADC_FAILED);
        return;
    };

    // Motor 0 is on Timer 1, which triggers ADC 2 and 3 on an injected conversion
    // Motor 1 is on Timer 8, which triggers ADC 2 and 3 on a regular conversion
    // If the corresponding timer is counting up, we just sampled in SVM vector 0, i.e. real current
    // If we are counting down, we just sampled in SVM vector 7, with zero current
    Motor_t *motor = injected ? &motors[0] : &motors[1];
    bool counting_down = motor->motor_timer->Instance->CR1 & TIM_CR1_DIR;

    bool current_meas_not_DC_CAL;
    if (motor == &motors[1] && counting_down) {
        // We are measuring M1 DC_CAL here
        current_meas_not_DC_CAL = false;
        // Load next timings for M0 (only once is sufficient)
        if (hadc == &hadc2) {
            motors[0].motor_timer->Instance->CCR1 = motors[0].next_timings[0];
            motors[0].motor_timer->Instance->CCR2 = motors[0].next_timings[1];
            motors[0].motor_timer->Instance->CCR3 = motors[0].next_timings[2];
        }
        // Check the timing of the sequencing
        check_timing(motor, TIMING_LOG_ADC_CB_M1_DC);

    } else if (motor == &motors[0] && !counting_down) {
        // We are measuring M0 current here
        current_meas_not_DC_CAL = true;
        // Load next timings for M1 (only once is sufficient)
        if (hadc == &hadc2) {
            motors[1].motor_timer->Instance->CCR1 = motors[1].next_timings[0];
            motors[1].motor_timer->Instance->CCR2 = motors[1].next_timings[1];
            motors[1].motor_timer->Instance->CCR3 = motors[1].next_timings[2];
        }
        // Check the timing of the sequencing
        check_timing(motor, TIMING_LOG_ADC_CB_M0_I);

    } else if (motor == &motors[1] && !counting_down) {
        // We are measuring M1 current here
        current_meas_not_DC_CAL = true;
        // Check the timing of the sequencing
        check_timing(motor, TIMING_LOG_ADC_CB_M1_I);

    } else if (motor == &motors[0] && counting_down) {
        // We are measuring M0 DC_CAL here
        current_meas_not_DC_CAL = false;
        // Check the timing of the sequencing
        check_timing(motor, TIMING_LOG_ADC_CB_M0_DC);

    } else {
        global_fault(ERROR_PWM_SRC_FAIL);
        return;
    }

    uint32_t ADCValue;
    if (injected) {
        ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    } else {
        ADCValue = HAL_ADC_GetValue(hadc);
    }
    float current = phase_current_from_adcval(motor, ADCValue);

    if (current_meas_not_DC_CAL) {
        // ADC2 and ADC3 record the phB and phC currents concurrently,
        // and their interrupts should arrive on the same clock cycle.
        // We dispatch the callbacks in order, so ADC2 will always be processed before ADC3.
        // Therefore we store the value from ADC2 and signal the thread that the
        // measurement is ready when we receive the ADC3 measurement

        // return or continue
        if (hadc == &hadc2) {
            motor->current_meas.phB = current - motor->DC_calib.phB;
            return;
        } else {
            motor->current_meas.phC = current - motor->DC_calib.phC;
        }
        // Trigger motor thread
        if (motor->thread_ready) osSignalSet(motor->motor_thread, M_SIGNAL_PH_CURRENT_MEAS);
    } else {
        // DC_CAL measurement
        if (hadc == &hadc2) {
            motor->DC_calib.phB += (current - motor->DC_calib.phB) * calib_filter_k;
        } else {
            motor->DC_calib.phC += (current - motor->DC_calib.phC) * calib_filter_k;
        }
    }
}

//--------------------------------
// 测量与校准 (Measurement and Calibration)
//--------------------------------
/**
 * @brief 测量电机相电阻
 * @param motor 电机对象指针
 * @param test_current 测试电流 [A]
 * @param max_voltage 最大测试电压 [V]
 * @return true=测量成功, false=测量失败
 * 
 * ============================================================================
 * 相电阻测量原理
 * ============================================================================
 * 
 * 测量原理：欧姆定律 R = V / I
 * 
 * 方法：
 *   1. 在电机A相施加直流电压，B相和C相不施加电压
 *      （相当于在A相绕组上施加V_alpha = test_voltage, V_beta = 0）
 * 
 *   2. 使用PI控制器逐渐增加电压，直到电流达到test_current：
 *      V_test[n+1] = V_test[n] + kI × Ts × (I_test - I_alpha)
 * 
 *      其中：
 *        kI = 10.0 (V/s)/A  —— 积分增益
 *        Ts = current_meas_period —— 采样周期
 *        I_alpha = -(IphB + IphC) —— Clarke变换的alpha轴电流
 * 
 *      这本质上是一个纯积分控制器(I控制器)，通过累积误差来调节电压，
 *      使电流逐渐收敛到目标值。
 * 
 *   3. 当电流稳定后，电阻 = 稳定电压 / 目标电流
 *      R = V_test_final / I_test
 * 
 * 为什么只加A相电压？
 *   在三相星形连接中，A相电压 = (IphA) × R。
 *   由于IphA = -(IphB + IphC) = I_alpha，所以可以直接用I_alpha计算。
 * 
 * 安全限制：
 *   - 测试电压限制在max_voltage以内，防止过大电流
 *   - 测试时间固定为3秒（num_test_cycles = 3.0 / CURRENT_MEAS_PERIOD）
 *   - 结果验证：0.01Ω < R < 1.0Ω，超出范围认为测量失败
 * 
 * 时序：
 *   每次循环等待一个电流采样周期，应用测试电压，检查时序是否满足deadline。
 */
// TODO check Ibeta balance to verify good motor connection
bool measure_phase_resistance(Motor_t *motor, float test_current, float max_voltage) {
    static const float kI = 10.0f;                                 // [(V/s)/A]
    static const int num_test_cycles = 3.0f / CURRENT_MEAS_PERIOD; // Test runs for 3s
    float test_voltage = 0.0f;
    for (int i = 0; i < num_test_cycles; ++i) {
        osEvent evt = osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT);
        if (evt.status != osEventSignal) {
            motor->error = ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT;
            return false;
        }
        if (!do_checks(motor)) return false;

        float Ialpha = -(motor->current_meas.phB + motor->current_meas.phC);
        test_voltage += (kI * current_meas_period) * (test_current - Ialpha);
        if (test_voltage > max_voltage) test_voltage = max_voltage;
        if (test_voltage < -max_voltage) test_voltage = -max_voltage;

        // Test voltage along phase A
        queue_voltage_timings(motor, test_voltage, 0.0f);

        // Check we meet deadlines after queueing
        motor->last_cpu_time = check_timing(motor, TIMING_LOG_MEAS_R);
        if (!(motor->last_cpu_time < motor->control_deadline)) {
            motor->error = ERROR_PHASE_RESISTANCE_TIMING;
            return false;
        }
    }

    // De-energize motor
    queue_voltage_timings(motor, 0.0f, 0.0f);

    float R = test_voltage / test_current;
    motor->phase_resistance = R;
    if (fabs(test_voltage) == fabs(max_voltage) || R < 0.01f || R > 1.0f) {
        motor->error = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        return false;
    }
    return true;
}

/**
 * @brief 测量电机相电感
 * @param motor 电机对象指针
 * @param voltage_low 低测试电压 [V]
 * @param voltage_high 高测试电压 [V]
 * @return true=测量成功, false=测量失败
 * 
 * ============================================================================
 * 相电感测量原理
 * ============================================================================
 * 
 * 测量原理：电感电压-电流关系 V = L × di/dt  →  L = V / (di/dt)
 * 
 * 方法：
 *   1. 交替施加两个不同的电压（voltage_low和voltage_high）到A相
 * 
 *   2. 在每个电压下累加电流采样值（共num_cycles=5000个周期）：
 *      Ialphas[0] = Σ(I_alpha @ voltage_low)
 *      Ialphas[1] = Σ(I_alpha @ voltage_high)
 * 
 *   3. 计算平均电流变化率：
 *      ΔI/Δt = (Ialphas[1] - Ialphas[0]) / (Ts × num_cycles)
 * 
 *      其中Ts是采样周期，num_cycles是循环次数。
 *      这实际上是两个电压水平下的平均电流差除以总时间。
 * 
 *   4. 计算电压差：
 *      v_L = 0.5 × (voltage_high - voltage_low)
 * 
 *      乘以0.5的原因：交替施加高低电压，每个电压只作用一半时间。
 * 
 *   5. 最终电感值：
 *      L = v_L / (ΔI/Δt)
 * 
 * 为什么交替施加电压？
 *   如果只施加单一电压，电流会持续上升直到饱和。
 *   交替施加高低电压，可以在一个可控的电流范围内测量电感，
 *   同时避免电流过大。高低电压差产生的净电流变化用于计算电感。
 * 
 * 安全限制：
 *   - 结果验证：1μH < L < 500μH，超出范围认为测量失败
 *   - 每个循环都检查时序deadline
 */
bool measure_phase_inductance(Motor_t *motor, float voltage_low, float voltage_high) {
    float test_voltages[2] = { voltage_low, voltage_high };
    float Ialphas[2] = { 0.0f };
    static const int num_cycles = 5000;

    for (int t = 0; t < num_cycles; ++t) {
        for (int i = 0; i < 2; ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
                motor->error = ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT;
                return false;
            }
            if (!do_checks(motor)) return false;

            Ialphas[i] += -motor->current_meas.phB - motor->current_meas.phC;

            // Test voltage along phase A
            queue_voltage_timings(motor, test_voltages[i], 0.0f);

            // Check we meet deadlines after queueing
            motor->last_cpu_time = check_timing(motor, TIMING_LOG_MEAS_L);
            if (!(motor->last_cpu_time < motor->control_deadline)) {
                motor->error = ERROR_PHASE_INDUCTANCE_TIMING;
                return false;
            }
        }
    }

    // De-energize motor
    queue_voltage_timings(motor, 0.0f, 0.0f);

    float v_L = 0.5f * (voltage_high - voltage_low);
    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (current_meas_period * (float)num_cycles);
    float L = v_L / dI_by_dt;

    motor->phase_inductance = L;
    // TODO arbitrary values set for now
    if (L < 1e-6f || L > 500e-6f) {
        motor->error = ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
        return false;
    }
    return true;
}

/**
 * @brief 编码器偏移校准 —— 通过旋转电压矢量扫描确定编码器零位
 * @param motor 电机对象指针
 * @param voltage_magnitude 扫描电压幅值 [V]
 * @return true=校准成功, false=校准失败
 * 
 * ============================================================================
 * 编码器偏移校准原理
 * ============================================================================
 * 
 * 目的：
 *   增量式编码器只能输出相对位置，不知道绝对零位。
 *   而FOC控制需要知道转子磁极的绝对电角度。
 *   本函数通过旋转电压矢量驱动转子旋转，同时记录编码器读数，
 *   从而确定编码器零点与电机电气零点的偏移量。
 * 
 * 原理：
 *   在静止的alpha-beta坐标系中施加旋转电压矢量：
 *     V_alpha = V_magnitude × cos(θ)
 *     V_beta  = V_magnitude × sin(θ)
 * 
 *   这会产生一个以角速度ω旋转的磁场，转子会跟随磁场旋转。
 *   通过比较转子实际旋转的电角度和编码器计数的变化，可以确定：
 *     1. 编码器CPR是否正确
 *     2. 电机旋转方向与编码器计数方向是否一致
 *     3. 编码器偏移量（零点偏差）
 * 
 * 校准流程：
 * 
 *   第1步：预锁定（1秒）
 *     施加V_alpha = voltage_magnitude, V_beta = 0的直流电压
 *     使转子锁定在电气零点位置，准备开始扫描
 * 
 *   第2步：正向扫描（8π电弧度 = 4圈电角度）
 *     电压矢量从-8π/2旋转到+8π/2
 *     每一步：ph += step_size，step_size = 16π / 2048
 *     每步保持dt_step = 1/500 = 2ms
 *     记录每个位置的编码器读数（累加到encvaluesum）
 * 
 *   第3步：验证CPR和方向
 *     预期编码器变化：expected = scan_range / elec_rad_per_enc
 *       其中 elec_rad_per_enc = pole_pairs × 2π / encoder_cpr
 *     如果实际变化与预期偏差超过encoder_calib_range(2%)，则失败
 *     方向判断：
 *       编码器增加 → motor_dir = 1（电机与编码器同向）
 *       编码器减少 → motor_dir = -1（电机与编码器反向）
 *       变化太小(<8) → 编码器响应异常，失败
 * 
 *   第4步：反向扫描
 *     电压矢量从+8π/2旋转回-8π/2
 *     同样记录编码器读数
 * 
 *   第5步：计算偏移量
 *     encoder_offset = encvaluesum / (num_steps × 2)
 *     正反两次扫描的平均值，消除机械振动和滞后误差
 * 
 * 为什么扫描16π电弧度（8圈电角度）？
 *   扫描多圈可以平均掉机械误差和编码器噪声，提高校准精度。
 *   对于7极对电机，16π电角度 = 16π/(7×2π) ≈ 1.14圈机械角度。
 * 
 * 参数说明：
 *   num_steps = 2048：扫描步数，决定角度分辨率
 *   dt_step = 2ms：每步保持时间，确保转子有足够响应时间
 *   scan_range = 16π：扫描总角度（电弧度）
 *   step_size = scan_range/num_steps ≈ 0.0245 rad/step
 */
// TODO: Do the scan with current, not voltage!
// TODO: add check_timing
bool calib_enc_offset(Motor_t *motor, float voltage_magnitude) {
    static const float start_lock_duration = 1.0f;
    static const int num_steps = 1024 * 2;
    static const float dt_step = 1.0f / 500.0f;
    static const float scan_range = 16.0f * M_PI;
    const float step_size = scan_range / (float)num_steps;  // TODO handle const expressions better (maybe switch to C++ ?)

    // go to motor zero phase for start_lock_duration to get ready to scan
    for (int i = 0; i < start_lock_duration * current_meas_hz; ++i) {
        if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
            motor->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
            return false;
        }
        if (!do_checks(motor)) return false;
        queue_voltage_timings(motor, voltage_magnitude, 0.0f);
    }

    int32_t init_enc_val = (int16_t)motor->encoder.encoder_timer->Instance->CNT;
    int32_t encvaluesum = 0;

    // scan forwards
    for (float ph = -scan_range / 2.0f; ph < scan_range / 2.0f; ph += step_size) {
        for (int i = 0; i < dt_step * (float)current_meas_hz; ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
                motor->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
                return false;
            }
            if (!do_checks(motor)) return false;
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);
        }
        encvaluesum += (int16_t)motor->encoder.encoder_timer->Instance->CNT;
    }

    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = motor->pole_pairs * 2 * M_PI * (1.0f / (float)(motor->encoder.encoder_cpr));
    float expected_encoder_delta = scan_range / elec_rad_per_enc;
    float actual_encoder_delta_abs = fabsf((int16_t)motor->encoder.encoder_timer->Instance->CNT - init_enc_val);
    if (fabsf(actual_encoder_delta_abs - expected_encoder_delta) / expected_encoder_delta > motor->encoder.encoder_calib_range) {
        motor->error = ERROR_ENCODER_CPR_OUT_OF_RANGE;
        return false;
    }
    // check direction
    if ((int16_t)motor->encoder.encoder_timer->Instance->CNT > init_enc_val + 8) {
        // motor same dir as encoder
        motor->encoder.motor_dir = 1;
    } else if ((int16_t)motor->encoder.encoder_timer->Instance->CNT < init_enc_val - 8) {
        // motor opposite dir as encoder
        motor->encoder.motor_dir = -1;
    } else {
        // Encoder response error
        motor->error = ERROR_ENCODER_RESPONSE;
        return false;
    }
    // scan backwards
    for (float ph = scan_range / 2.0f; ph > -scan_range / 2.0f; ph -= step_size) {
        for (int i = 0; i < dt_step * (float)current_meas_hz; ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
                motor->error = ERROR_ENCODER_MEASUREMENT_TIMEOUT;
                return false;
            }
            if (!do_checks(motor)) return false;
            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);
        }
        encvaluesum += (int16_t)motor->encoder.encoder_timer->Instance->CNT;
    }

    int offset = encvaluesum / (num_steps * 2);
    motor->encoder.encoder_offset = offset;
    return true;
}

/**
 * @brief 电机完整校准流程
 * @param motor 电机对象指针
 * @return true=校准全部成功, false=某一步骤失败
 * 
 * ============================================================================
 * 完整校准流程
 * ============================================================================
 * 
 * 校准是电机首次运行前必须执行的步骤，确保控制器准确知道电机的电气参数
 * 和机械零点。校准失败则电机无法启动。
 * 
 * 流程顺序：
 * 
 *   1. 相电阻测量（仅大电流电机）
 *      measure_phase_resistance(calibration_current, R_calib_max_voltage)
 *      → 结果存入 motor->phase_resistance
 * 
 *   2. 相电感测量（仅大电流电机）
 *      measure_phase_inductance(-R_calib_max_voltage, R_calib_max_voltage)
 *      → 结果存入 motor->phase_inductance
 * 
 *   3. 编码器Index搜索（如果需要且未找到）
 *      scan_for_enc_idx() → 旋转电机找到编码器Z相脉冲
 * 
 *   4. 编码器偏移校准（如果不是手动校准过的）
 *      calib_enc_offset(enc_calibration_voltage)
 *      → 结果存入 motor->encoder.encoder_offset 和 motor->encoder.motor_dir
 * 
 *   5. 计算电流环PI增益
 *      电流环带宽 ω_c = 1000 rad/s
 *      电流环传递函数：G(s) = 1/(Ls + R)
 *      
 *      PI控制器设计（极点-零点对消）：
 *        Kp = ω_c × L          —— 比例增益
 *        Ki = (R/L) × Kp       —— 积分增益
 *                             = (R/L) × ω_c × L
 *                             = ω_c × R
 *      
 *      其中 plant_pole = R/L 是电机的电时间常数极点。
 *      积分增益对消了电机电感产生的极点，使闭环传递函数为一阶系统。
 * 
 *   6. 计算编码器PLL增益
 *      PLL带宽 ω_pll = 1000 rad/s
 *      二阶PLL（临界阻尼 ζ=0.5）：
 *        Kp = 2 × ω_pll       —— 比例增益
 *        Ki = 0.25 × Kp²      —— 积分增益 = ω_pll²
 *      
 *      闭环特征方程：s² + Kp·s + Ki = s² + 2ζω_n·s + ω_n²
 *      临界阻尼条件：ζ = 0.5，自然频率 ω_n = ω_pll
 * 
 *   7. 无感观测器PLL增益（暂时与编码器相同）
 * 
 * 云台电机(GIMBAL)模式：
 *   不需要测量电阻和电感，校准电压直接使用calibration_current（单位变为V）。
 * 
 * 校准电压计算：
 *   大电流电机：enc_calibration_voltage = calibration_current × phase_resistance
 *   云台电机：  enc_calibration_voltage = calibration_current（直接作为电压）
 */
bool motor_calibration(Motor_t *motor) {
    motor->error = ERROR_NO_ERROR;

    float R_calib_max_voltage = motor->resistance_calib_max_voltage;
    float enc_calibration_voltage = 0.0f;
    if (motor->motor_type == MOTOR_TYPE_HIGH_CURRENT) {
        if (!measure_phase_resistance(motor, motor->calibration_current, R_calib_max_voltage)) return false;
        enc_calibration_voltage = motor->calibration_current * motor->phase_resistance;

        if (!measure_phase_inductance(motor, -R_calib_max_voltage, R_calib_max_voltage)) return false;
    } else if (motor->motor_type == MOTOR_TYPE_GIMBAL) {
        enc_calibration_voltage = motor->calibration_current;
    } else {
        return false;
    }

    if (motor->rotor_mode == ROTOR_MODE_ENCODER ||
        motor->rotor_mode == ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS) {
        if (motor->encoder.use_index && !motor->encoder.index_found) if (!scan_for_enc_idx(motor,
                                                                                           (float)(motor->encoder.motor_dir) * motor->encoder.idx_search_speed,
                                                                                           enc_calibration_voltage)) return false;
        if (!motor->encoder.manually_calibrated) if (!calib_enc_offset(motor, enc_calibration_voltage)) return false;
    }

    // Calculate current control gains
    float current_control_bandwidth = 1000.0f;  // [rad/s]
    motor->current_control.p_gain = current_control_bandwidth * motor->phase_inductance;
    float plant_pole = motor->phase_resistance / motor->phase_inductance;
    motor->current_control.i_gain = plant_pole * motor->current_control.p_gain;

    // Calculate encoder pll gains
    float encoder_pll_bandwidth = 1000.0f;  // [rad/s]
    motor->encoder.pll_kp = 2.0f * encoder_pll_bandwidth;
    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * motor->encoder.pll_kp < 1.0f)) {
        motor->error = ERROR_CALIBRATION_TIMING;
        return false;
    }
    // Critically damped
    motor->encoder.pll_ki = 0.25f * (motor->encoder.pll_kp * motor->encoder.pll_kp);

    // sensorless pll same as encoder (for now)
    motor->sensorless.pll_kp = motor->encoder.pll_kp;
    motor->sensorless.pll_ki = motor->encoder.pll_ki;

    motor->calibration_ok = true;

    return true;
}

/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 *
 * This holding current is added as a feedforward term in the control loop.
 */
/**
 * @brief 防齿槽转矩(Anti-cogging)校准
 * @param motor 电机对象指针
 * @return true=校准完成, false=校准中或不需要校准
 * 
 * 齿槽转矩(Cogging Torque)是永磁电机固有的转矩波动，
 * 由永磁体与定子齿之间的磁阻变化引起。
 * 在低速运行时，齿槽转矩会导致速度波动和振动。
 * 
 * 抗齿槽校准方法：
 *   逐个遍历编码器位置(0~encoder_cpr-1)，在每个位置：
 *     1. 等待电机稳定（位置误差和速度都小于阈值）
 *     2. 记录速度环积分器电流值 → 这就是克服该位置齿槽转矩所需的电流
 *     3. 存入cogging_map[index]
 * 
 *   校准完成后，cogging_map[]存储了每个编码器位置所需的补偿电流。
 *   在运行时，根据当前位置查表获取补偿值，作为前馈项加入电流指令。
 * 
 * 这是一个逐点进行的非阻塞校准：每次调用只处理一个位置，
 * 返回false表示继续校准，返回true表示校准完成。
 */
bool anti_cogging_calibration(Motor_t *motor) {
    if (motor->anticogging.calib_anticogging && motor->anticogging.cogging_map != NULL) {
        float pos_err = motor->anticogging.index - motor->encoder.pll_pos;
        if (fabsf(pos_err) <= motor->anticogging.calib_pos_threshold &&
            fabsf(motor->encoder.pll_vel) < motor->anticogging.calib_vel_threshold) {
            motor->anticogging.cogging_map[motor->anticogging.index++] = motor->vel_integrator_current;
        }
        if (motor->anticogging.index < motor->encoder.encoder_cpr) {
            set_pos_setpoint(motor, motor->anticogging.index, 0.0f, 0.0f);
            return false;
        } else {
            motor->anticogging.index = 0;
            set_pos_setpoint(motor, 0.0f, 0.0f, 0.0f);  // Send the motor home
            motor->anticogging.use_anticogging = true;  // We're good to go, enable anti-cogging
            motor->anticogging.calib_anticogging = false;
            return true;
        }
    }
    return false;
}

//--------------------------------
// 测试函数 (Test Functions)
//--------------------------------
/**
 * @brief 搜索编码器Index脉冲（Z相脉冲）
 * @param motor 电机对象指针
 * @param omega 旋转速度 [rad/s 电角度]
 * @param voltage_magnitude 搜索电压幅值 [V]
 * @return true=找到Index脉冲, false=时序错误
 * 
 * 施加旋转电压矢量驱动电机缓慢旋转，直到检测到编码器的Index(Z相)脉冲。
 * 用于需要绝对位置参考的编码器模式。
 */
bool scan_for_enc_idx(Motor_t *motor, float omega, float voltage_magnitude) {
    for (;;) {
        for (float ph = 0.0f; ph < 2.0f * M_PI; ph += omega * current_meas_period) {
            osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, osWaitForever);
            if (!do_checks(motor)) return false;

            if (motor->encoder.index_found) return true;

            float v_alpha = voltage_magnitude * arm_cos_f32(ph);
            float v_beta = voltage_magnitude * arm_sin_f32(ph);
            queue_voltage_timings(motor, v_alpha, v_beta);

            // Check we meet deadlines after queueing
            motor->last_cpu_time = check_timing(motor, TIMING_LOG_IDX_SEARCH);
            if (!(motor->last_cpu_time < motor->control_deadline)) {
                motor->error = ERROR_SCAN_MOTOR_TIMING;
                return false;
            }
        }
    }
}

//--------------------------------
// 主电机控制 (Main Motor Control)
//--------------------------------
/**
 * @brief 更新转子位置和速度估计
 * @param motor 电机对象指针
 * 
 * ============================================================================
 * 转子位置观测 —— 编码器PLL与无感磁链观测器
 * ============================================================================
 * 
 * 本函数根据当前的rotor_mode选择相应的位置观测方法：
 *   ROTOR_MODE_ENCODER: 编码器PLL
 *   ROTOR_MODE_SENSORLESS: 无感磁链观测器
 *   ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS: 同时运行两种方法（测试用）
 * 
 * ============================================================================
 * 一、编码器PLL锁相环 (Encoder PLL)
 * ============================================================================
 * 
 * PLL(Phase Locked Loop)的作用是将编码器脉冲信号转换为平滑连续的
 * 位置和速度估计值，同时滤除编码器噪声。
 * 
 * 步骤1：更新编码器内部状态
 *   delta_enc = CNT - encoder_state  // 读取编码器增量（处理溢出回绕）
 *   encoder_state += delta_enc        // 累加到32位状态变量
 * 
 *   为什么要用int16_t转换？
 *   STM32定时器CNT是16位寄存器，直接读取可能漏掉中间计数值。
 *   通过(int16_t)差值计算，自动处理计数器溢出回绕问题。
 * 
 * 步骤2：计算电气角度
 *   corrected_enc = (encoder_state % encoder_cpr) - encoder_offset
 *   corrected_enc *= motor_dir
 *   elec_rad_per_enc = pole_pairs × 2π / encoder_cpr
 *   phase = elec_rad_per_enc × corrected_enc
 * 
 *   其中：
 *     encoder_cpr = 编码器每转计数值 (600线×4倍频 = 2400)
 *     encoder_offset = 校准得到的零点偏移
 *     motor_dir = 方向校正(1或-1)
 *     pole_pairs = 电机极对数(7)
 *     elec_rad_per_enc = 每个编码器计数对应的电角度
 * 
 *   电气角度与机械角度的关系：
 *     θ_electrical = pole_pairs × θ_mechanical
 * 
 * 步骤3：PLL滤波
 *   预测：pll_pos += Ts × pll_vel
 *   误差：delta_pos = encoder_state - floor(pll_pos)
 *   反馈：pll_pos += Ts × pll_kp × delta_pos
 *          pll_vel += Ts × pll_ki × delta_pos
 * 
 *   这是一个标准的二阶PLL：
 *     位置更新：pll_pos = ∫pll_vel + Kp × Δpos
 *     速度更新：pll_vel = ∫Ki × Δpos
 * 
 *   PLL增益（由motor_calibration计算）：
 *     Kp = 2 × ω_pll = 2000 rad/s
 *     Ki = ω_pll² / 4 = 250000 (rad/s²)/rad
 * 
 *   闭环带宽 = 1000 rad/s ≈ 160Hz
 * 
 * ============================================================================
 * 二、无感磁链观测器 (Sensorless Flux Observer)
 * ============================================================================
 * 
 * 基于论文：
 *   "Sensorless Control of Surface-Mount Permanent-Magnet Synchronous Motors
 *    Based on a Nonlinear Observer"
 *   Lee, Hong, Nam, Ortega, Praly, Astolfi
 *   IEEE Transactions on Power Electronics, 2010
 * 
 * 观测器原理：
 *   表贴式永磁同步电机(SPM)的电压方程：
 *     V = R×I + L×dI/dt + λ_pm×ω×[-sin(θ); cos(θ)]
 * 
 *   其中λ_pm是永磁体磁链，最后项是反电动势(Back-EMF)。
 * 
 *   定义磁链状态：
 *     ψ = L×I + λ_pm×[cos(θ); sin(θ)]  —— 总磁链（电感磁链 + 永磁体磁链）
 * 
 *   则电压方程变为：
 *     V = R×I + dψ/dt
 *     → dψ/dt = V - R×I  —— 公式(4)
 * 
 *   积分得到磁链估计：
 *     ψ̂(t) = ∫(V - R×I)dt
 * 
 *   永磁体磁链分量：
 *     η = ψ̂ - L×I  —— 公式(6)
 * 
 *   理想情况下 |η| = λ_pm，由此可以估计转子角度：
 *     θ = atan2(η_β, η_α)
 * 
 * 非线性观测器（公式8）：
 *   纯积分容易漂移，需要加入校正项：
 * 
 *     dψ̂/dt = V - R×I + γ × (λ_pm² - |η|²) × η
 * 
 *   其中γ是观测器增益(observer_gain)。
 *   校正项的作用：
 *     如果|η| > λ_pm → 减小磁链估计
 *     如果|η| < λ_pm → 增大磁链估计
 *   使|η|收敛到λ_pm，保证幅值正确。
 * 
 * 算法步骤：
 * 
 *   1. Clarke变换：I_alpha_beta = [-(phB+phC), 1/√3×(phB-phC)]
 * 
 *   2. 磁链预测（积分电压方程）：
 *      y = -R×I_alpha_beta + V_alpha_beta_memory
 *      flux_state += y × Ts
 *      eta = flux_state - L×I_alpha_beta
 * 
 *   3. 非线性校正：
 *      pm_flux_sqr = λ_pm²
 *      est_pm_flux_sqr = |η|² = η_α² + η_β²
 *      eta_factor = 0.5 × γ / λ_pm² × (pm_flux_sqr - est_pm_flux_sqr)
 *      flux_state += eta_factor × eta × Ts
 * 
 *   4. 更新eta（校正后的永磁体磁链估计）：
 *      eta = flux_state - L×I_alpha_beta
 * 
 *   5. 存储当前V_alpha_beta（用于下一个周期的预测）
 * 
 *   6. PLL跟踪eta的角度：
 *      pll_pos += Ts × pll_vel            // 速度预测
 *      phase = atan2(eta_β, eta_α)        // 从磁链计算角度
 *      delta_phase = wrap_pm_pi(phase - pll_pos)  // 相位误差
 *      pll_pos += Ts × pll_kp × delta_phase
 *      pll_vel += Ts × pll_ki × delta_phase
 * 
 * 注意：V_alpha_beta_memory中存储的是2个周期前的电压值，
 *       因为电流采样与控制之间存在固有的延迟：
 *       当前时刻采样的电流，对应的电压是2个周期前施加的。
 */
void update_rotor(Motor_t *motor) {
    switch (motor->rotor_mode) {
    case ROTOR_MODE_ENCODER:
    case ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS:
        {
            //for convenience
            Encoder_t *encoder = &motor->encoder;

            // update internal encoder state
            int16_t delta_enc = (int16_t)encoder->encoder_timer->Instance->CNT - (int16_t)encoder->encoder_state;
            encoder->encoder_state += (int32_t)delta_enc;

            // compute electrical phase
            int corrected_enc = encoder->encoder_state % motor->encoder.encoder_cpr;
            corrected_enc -= encoder->encoder_offset;
            corrected_enc *= encoder->motor_dir;
            //TODO avoid recomputing elec_rad_per_enc every time
            float elec_rad_per_enc = motor->pole_pairs * 2 * M_PI * (1.0f / (float)(motor->encoder.encoder_cpr));
            float ph = elec_rad_per_enc * (float)corrected_enc;
            // ph = fmodf(ph, 2*M_PI);
            encoder->phase = wrap_pm_pi(ph);

            // run pll (for now pll is in units of encoder counts)
            // TODO pll_pos runs out of precision very quickly here! Perhaps decompose into integer and fractional part?
            // Predict current pos
            encoder->pll_pos += current_meas_period * encoder->pll_vel;
            // discrete phase detector
            float delta_pos = (float)(encoder->encoder_state - (int32_t)floorf(encoder->pll_pos));
            // pll feedback
            encoder->pll_pos += current_meas_period * encoder->pll_kp * delta_pos;
            encoder->pll_vel += current_meas_period * encoder->pll_ki * delta_pos;
        }
        // Drop through to sensorless if also testing
        if (motor->rotor_mode != ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS) break;
    case ROTOR_MODE_SENSORLESS:
        {
            // Algorithm based on paper: Sensorless Control of Surface-Mount Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer
            // http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
            // In particular, equation 8 (and by extension eqn 4 and 6).

            // The V_alpha_beta applied immedietly prior to the current measurement associated with this cycle
            // is the one computed two cycles ago. To get the correct measurement, it was stored twice:
            // once by final_v_alpha/final_v_beta in the current control reporting, and once by V_alpha_beta_memory.

            //for convenience
            Sensorless_t *sensorless = &motor->sensorless;

            // Clarke transform
            float I_alpha_beta[2] = {
                -motor->current_meas.phB - motor->current_meas.phC,
                one_by_sqrt3 * (motor->current_meas.phB - motor->current_meas.phC)
            };

            // alpha-beta vector operations
            float eta[2];
            for (int i = 0; i <= 1; ++i) {
                // y is the total flux-driving voltage (see paper eqn 4)
                float y = -motor->phase_resistance * I_alpha_beta[i] + sensorless->V_alpha_beta_memory[i];
                // flux dynamics (prediction)
                float x_dot = y;
                // integrate prediction to current timestep
                sensorless->flux_state[i] += x_dot * current_meas_period;

                // eta is the estimated permanent magnet flux (see paper eqn 6)
                eta[i] = sensorless->flux_state[i] - motor->phase_inductance * I_alpha_beta[i];
            }

            // Non-linear observer (see paper eqn 8):
            float pm_flux_sqr = sensorless->pm_flux_linkage * sensorless->pm_flux_linkage;
            float est_pm_flux_sqr = eta[0] * eta[0] + eta[1] * eta[1];
            float bandwidth_factor = 1.0f / (sensorless->pm_flux_linkage * sensorless->pm_flux_linkage);
            float eta_factor = 0.5f * (sensorless->observer_gain * bandwidth_factor) * (pm_flux_sqr - est_pm_flux_sqr);

            static float eta_factor_avg_test = 0.0f;
            eta_factor_avg_test += 0.001f * (eta_factor - eta_factor_avg_test);

            // alpha-beta vector operations
            for (int i = 0; i <= 1; ++i) {
                // add observer action to flux estimate dynamics
                float x_dot = eta_factor * eta[i];
                // convert action to discrete-time
                sensorless->flux_state[i] += x_dot * current_meas_period;
                // update new eta
                eta[i] = sensorless->flux_state[i] - motor->phase_inductance * I_alpha_beta[i];
            }

            // Flux state estimation done, store V_alpha_beta for next timestep
            sensorless->V_alpha_beta_memory[0] = motor->current_control.final_v_alpha;
            sensorless->V_alpha_beta_memory[1] = motor->current_control.final_v_beta;

            // PLL
            // predict PLL phase with velocity
            sensorless->pll_pos = wrap_pm_pi(sensorless->pll_pos + current_meas_period * sensorless->pll_vel);
            // update PLL phase with observer permanent magnet phase
            sensorless->phase = fast_atan2(eta[1], eta[0]);
            float delta_phase = wrap_pm_pi(sensorless->phase - sensorless->pll_pos);
            sensorless->pll_pos = wrap_pm_pi(sensorless->pll_pos + current_meas_period * sensorless->pll_kp * delta_phase);
            // update PLL velocity
            sensorless->pll_vel += current_meas_period * sensorless->pll_ki * delta_phase;

            //TODO TEMP TEST HACK
            // static int trigger_ctr = 0;
            // if (++trigger_ctr >= 3*current_meas_hz) {
            //     trigger_ctr = 0;

            //     //Change to sensorless units
            //     motor->vel_gain = 15.0f / 200.0f;
            //     motor->vel_setpoint = 800.0f * motor->encoder.motor_dir;

            //     //Change mode
            //     motor->rotor_mode = ROTOR_MODE_SENSORLESS;
            // }

        }
        break;
    default:
        //TODO error handling
        break;
    }
}

bool using_encoder(Motor_t *motor) {
    if (motor->rotor_mode == ROTOR_MODE_ENCODER ||
        motor->rotor_mode == ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS) return true;
    else return false;
}

bool using_sensorless(Motor_t *motor) {
    if (motor->rotor_mode == ROTOR_MODE_SENSORLESS) return true;
    else return false;
}

float get_rotor_phase(Motor_t *motor) {
    if (using_encoder(motor)) return motor->encoder.phase;
    else if (using_sensorless(motor)) return motor->sensorless.phase;
    else
        //TODO error handling
        return 0.0f;
}

float get_pll_vel(Motor_t *motor) {
    if (using_encoder(motor)) return motor->encoder.pll_vel;
    else if (using_sensorless(motor)) return motor->sensorless.pll_vel;
    else
        //TODO error handling
        return 0.0f;
}

// Function that sets the current encoder count to a desired 32-bit value.
void setEncoderCount(Motor_t *motor, uint32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    motor->encoder.encoder_state = count;
    motor->encoder.encoder_timer->Instance->CNT = count;
    motor->encoder.pll_pos = (float)count;
    __set_PRIMASK(prim);
}

bool spin_up_timestep(Motor_t *motor, float phase, float I_mag) {
    // wait for new timestep
    if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
        motor->error = ERROR_SPIN_UP_TIMEOUT;
        return false;
    }

    if (!do_checks(motor)) return false;
    // run estimator
    if (!loop_updates(motor)) return false;

    // override the phase during spinup
    motor->sensorless.phase = phase;
    // run current control (with the phase override)
    FOC_current(motor, I_mag, 0.0f);

    return true;
}

bool spin_up_sensorless(Motor_t *motor) {
    static const float ramp_up_time = 0.4f;
    static const float ramp_up_distance = 4 * M_PI;
    float ramp_step = current_meas_period / ramp_up_time;

    float phase = 0.0f;
    float vel = ramp_up_distance / ramp_up_time;
    float I_mag = 0.0f;

    // spiral up current
    for (float x = 0.0f; x < 1.0f; x += ramp_step) {
        phase = wrap_pm_pi(ramp_up_distance * x);
        I_mag = motor->sensorless.spin_up_current * x;
        if (!spin_up_timestep(motor, phase, I_mag)) return false;
    }

    // accelerate
    while (vel < motor->sensorless.spin_up_target_vel) {
        vel += motor->sensorless.spin_up_acceleration * current_meas_period;
        phase = wrap_pm_pi(phase + vel * current_meas_period);
        if (!spin_up_timestep(motor, phase, motor->sensorless.spin_up_current)) return false;
    }

    // // test keep spinning
    // while (true) {
    //     phase = wrap_pm_pi(phase + vel * current_meas_period);
    //     if(!spin_up_timestep(motor, phase, motor->sensorless.spin_up_current))
    //         return false;
    // }

    return true;

    // TODO: check pll vel (abs ratio, 0.8)
}

void update_brake_current() {
    float Ibus_sum = 0.0f;
    for (int i = 0; i < num_motors; ++i) {
        Ibus_sum += motors[i].current_control.Ibus;
    }
    // Note: set_brake_current will clip negative values to 0.0f
    set_brake_current(-Ibus_sum);
}

void set_brake_current(float brake_current) {
    if (brake_current < 0.0f) brake_current = 0.0f;
    float brake_duty = brake_current * brake_resistance / vbus_voltage;

    // Duty limit at 90% to allow bootstrap caps to charge
    if (brake_duty > 0.9f) brake_duty = 0.9f;
    int high_on = TIM_APB1_PERIOD_CLOCKS * (1.0f - brake_duty);
    int low_off = high_on - TIM_APB1_DEADTIME_CLOCKS;
    if (low_off < 0) low_off = 0;

    // Safe update of low and high side timings
    // To avoid race condition, first reset timings to safe state
    // ch3 is low side, ch4 is high side
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    htim2.Instance->CCR3 = low_off;
    htim2.Instance->CCR4 = high_on;
}

void queue_modulation_timings(Motor_t *motor, float mod_alpha, float mod_beta) {
    float tA, tB, tC;
    SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
    motor->next_timings[0] = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
    motor->next_timings[1] = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
    motor->next_timings[2] = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
}

void queue_voltage_timings(Motor_t *motor, float v_alpha, float v_beta) {
    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;
    queue_modulation_timings(motor, mod_alpha, mod_beta);
}

/**
 * @brief 电压模式FOC控制（用于云台电机）
 * @param motor 电机对象指针
 * @param v_d d轴电压指令 [V]
 * @param v_q q轴电压指令 [V]
 * @return true=控制成功, false=时序错误
 * 
 * 云台电机不需要电流环，直接给定电压。
 * 将d-q坐标系电压通过Park逆变换转换到alpha-beta静止坐标系，
 * 然后送入SVM生成PWM波形。
 * 
 * Park逆变换：
 *   V_alpha = cos(θ) × V_d - sin(θ) × V_q
 *   V_beta  = cos(θ) × V_q + sin(θ) × V_d
 */
// TODO: This doesn't update brake current
// We should probably make FOC Current call FOC Voltage to avoid duplication.
bool FOC_voltage(Motor_t *motor, float v_d, float v_q) {
    float phase = get_rotor_phase(motor);
    float c = arm_cos_f32(phase);
    float s = arm_sin_f32(phase);
    float v_alpha = c * v_d - s * v_q;
    float v_beta  = c * v_q + s * v_d;
    queue_voltage_timings(motor, v_alpha, v_beta);

    // Check we meet deadlines after queueing
    if (!(check_timing(motor, TIMING_LOG_FOC_VOLTAGE) < motor->control_deadline)) {
        motor->error = ERROR_FOC_VOLTAGE_TIMING;
        return false;
    }
    return true;
}

/**
 * @brief FOC电流环控制 —— 磁场定向控制的核心实现
 * @param motor 电机对象指针
 * @param Id_des d轴期望电流 [A]，通常设为0（表贴式电机不需要励磁电流）
 * @param Iq_des q轴期望电流 [A]，q轴电流正比于转矩
 * @return true=控制成功, false=时序错误
 * 
 * ============================================================================
 * FOC电流控制完整流程
 * ============================================================================
 * 
 * 本函数是FOC控制的内环，每个电流采样周期(通常8kHz~16kHz)执行一次。
 * 
 * 完整的FOC控制链：
 * 
 *   电流采样(IphB, IphC)
 *        │
 *        ▼
 *   ┌──────────────────────────────────┐
 *   │  Clarke变换 (abc → alpha-beta)   │  三相→两相静止坐标系
 *   │  I_alpha = -(IphB + IphC)        │
 *   │  I_beta  = (IphB - IphC)/√3      │
 *   └──────────────────────────────────┘
 *        │
 *        ▼
 *   ┌──────────────────────────────────┐
 *   │  Park变换 (alpha-beta → d-q)     │  静止→旋转坐标系
 *   │  I_d = cos(θ)×I_alpha+sin(θ)×I_beta│
 *   │  I_q = cos(θ)×I_beta -sin(θ)×I_alpha│
 *   └──────────────────────────────────┘
 *        │
 *        ▼
 *   ┌──────────────────────────────────┐
 *   │  PI控制器                        │  d轴和q轴独立控制
 *   │  V_d = Kp×Ierr_d + Ki×∫Ierr_d   │
 *   │  V_q = Kp×Ierr_q + Ki×∫Ierr_q   │
 *   └──────────────────────────────────┘
 *        │
 *        ▼
 *   ┌──────────────────────────────────┐
 *   │  向量限幅 (Anti-windup)          │  防止调制饱和
 *   │  如果|V_dq| > V_max：            │
 *   │    按比例缩小V_d, V_q            │
 *   │    积分器衰减(anti-windup)       │
 *   └──────────────────────────────────┘
 *        │
 *        ▼
 *   ┌──────────────────────────────────┐
 *   │  Park逆变换 (d-q → alpha-beta)   │  旋转→静止坐标系
 *   │  V_alpha = cos(θ)×V_d-sin(θ)×V_q│
 *   │  V_beta  = cos(θ)×V_q+sin(θ)×V_d│
 *   └──────────────────────────────────┘
 *        │
 *        ▼
 *   ┌──────────────────────────────────┐
 *   │  SVM空间矢量调制                 │  电压→PWM占空比
 *   │  计算三相PWM导通时间tA, tB, tC   │
 *   └──────────────────────────────────┘
 *        │
 *        ▼
 *   PWM输出 → 三相MOSFET桥臂 → 电机
 * 
 * ============================================================================
 * 关键算法详解
 * ============================================================================
 * 
 * 1. Clarke变换 (等幅值变换)：
 *    将三相静止坐标系(abc)转换为两相静止坐标系(alpha-beta)
 * 
 *    [I_alpha]   [1    -1/2   -1/2 ] [I_a]
 *    [I_beta ] = [0   √3/2  -√3/2 ] [I_b]
 *    [I_0    ]   [1/2   1/2    1/2 ] [I_c]
 * 
 *    由于只采样了两相电流(IphB, IphC)，利用I_a + I_b + I_c = 0：
 *    I_alpha = I_a = -(I_b + I_c) = -(IphB + IphC)
 *    I_beta  = (I_b - I_c) / √3 = one_by_sqrt3 × (IphB - IphC)
 * 
 * 2. Park变换：
 *    将静止坐标系(alpha-beta)旋转θ角，得到旋转坐标系(d-q)
 * 
 *    [I_d]   [cos(θ)   sin(θ)] [I_alpha]
 *    [I_q] = [-sin(θ)  cos(θ)] [I_beta ]
 * 
 *    其中θ是转子电角度（由编码器PLL或无感观测器提供）
 * 
 *    d轴：与转子磁链方向对齐 → 控制磁通
 *    q轴：与d轴正交 → 控制转矩
 * 
 *    对于表贴式永磁同步电机(SPM)，转子磁链由永磁体提供，
 *    不需要额外的励磁电流，所以Id_des = 0。
 * 
 * 3. PI控制器：
 *    V_d = Vd_integral + Kp × (Id_des - Id)
 *    V_q = Vq_integral + Kp × (Iq_des - Iq)
 * 
 *    积分器更新：
 *    Vd_integral += Ki × (Id_des - Id) × Ts
 *    Vq_integral += Ki × (Iq_des - Iq) × Ts
 * 
 *    PI增益（由校准计算）：
 *    Kp = ω_c × L      （ω_c = 1000 rad/s 电流环带宽）
 *    Ki = (R/L) × Kp   （极点-零点对消设计）
 * 
 * 4. 向量限幅 (Vector Modulation Saturation)：
 *    当SVM调制幅值超过最大允许值时，按比例缩小d-q电压，
 *    同时衰减积分器（Anti-windup防积分饱和）。
 * 
 *    最大调制幅值：M_max = 0.80 × √3/2 ≈ 0.693
 *    （0.80是安全裕度系数，√3/2是SVM的最大线性调制范围）
 * 
 *    如果 √(mod_d² + mod_q²) > M_max：
 *      scale = M_max / √(mod_d² + mod_q²)
 *      mod_d *= scale
 *      mod_q *= scale
 *      integral *= 0.99  （积分衰减，防止windup）
 * 
 * 5. SVM空间矢量调制：
 *    将alpha-beta电压转换为三相PWM占空比。
 *    SVM相比传统SPWM有更高的直流母线利用率（+15.5%）。
 * 
 * 6. 母线电流估计：
 *    Ibus = mod_d × Id + mod_q × Iq
 *    用于制动电阻控制，估算回馈到母线的能量。
 */
bool FOC_current(Motor_t *motor, float Id_des, float Iq_des) {
    Current_control_t *ictrl = &motor->current_control;

    // For Reporting
    ictrl->Iq_setpoint = Iq_des;

    // Clarke transform
    float Ialpha = -motor->current_meas.phB - motor->current_meas.phC;
    float Ibeta = one_by_sqrt3 * (motor->current_meas.phB - motor->current_meas.phC);

    // Park transform
    float phase = get_rotor_phase(motor);
    float c = arm_cos_f32(phase);
    float s = arm_sin_f32(phase);
    float Id = c * Ialpha + s * Ibeta;
    float Iq = c * Ibeta - s * Ialpha;
    ictrl->Iq_measured = Iq;
    ictrl->Id_measured = Id;

    // Current error
    float Ierr_d = Id_des - Id;
    float Ierr_q = Iq_des - Iq;

    // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
    // Apply PI control
    float Vd = ictrl->v_current_control_integral_d + Ierr_d * ictrl->p_gain;
    float Vq = ictrl->v_current_control_integral_q + Ierr_q * ictrl->p_gain;

    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d = V_to_mod * Vd;
    float mod_q = V_to_mod * Vq;

    // Vector modulation saturation, lock integrator if saturated
    // TODO make maximum modulation configurable
    float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        // TODO make decayfactor configurable
        ictrl->v_current_control_integral_d *= 0.99f;
        ictrl->v_current_control_integral_q *= 0.99f;
    } else {
        ictrl->v_current_control_integral_d += Ierr_d * (ictrl->i_gain * current_meas_period);
        ictrl->v_current_control_integral_q += Ierr_q * (ictrl->i_gain * current_meas_period);
    }

    // Compute estimated bus current
    ictrl->Ibus = mod_d * Id + mod_q * Iq;

    // Inverse park transform
    float mod_alpha = c * mod_d - s * mod_q;
    float mod_beta = c * mod_q + s * mod_d;

    // Report final applied voltage in stationary frame (for sensorles estimator)
    ictrl->final_v_alpha = mod_to_V * mod_alpha;
    ictrl->final_v_beta = mod_to_V * mod_beta;

    // Apply SVM
    queue_modulation_timings(motor, mod_alpha, mod_beta);

    // Check we meet deadlines after queueing
    motor->last_cpu_time = check_timing(motor, TIMING_LOG_FOC_CURRENT);
    if (!(motor->last_cpu_time < motor->control_deadline)) {
        motor->error = ERROR_FOC_TIMING;
        return false;
    }

    update_brake_current();
    return true;
}

/**
 * @brief 检查DRV8301驱动器是否有故障
 * @return true=无故障, false=有故障
 * 
 * DRV8301的nFAULT引脚：高电平=正常，低电平=故障。
 * 故障类型包括：过流(OC)、过温(OTD)、欠压锁定(UVLO)等。
 */
//Returns true if everything is OK (no fault)
bool check_DRV_fault(Motor_t *motor) {
    //TODO: make this pin configurable per motor ch
    GPIO_PinState nFAULT_state = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);
    return (nFAULT_state == GPIO_PIN_RESET) ? false : true;
}

/**
 * @brief 检查直流母线是否欠压（Brownout保护）
 * @return true=电压正常, false=欠压
 * 
 * 当母线电压低于dc_bus_brownout_trip_level(默认8V)时触发保护。
 * 欠压可能导致MOSFET门极驱动不足，造成直通或失控。
 */
//Returns true if everything is OK (no fault)
bool check_PSU_brownout(Motor_t *motor) {
    if (vbus_voltage < motor->dc_bus_brownout_trip_level) return false;
    return true;
}

/**
 * @brief 执行所有安全检查（DRV故障 + 欠压保护）
 * @return true=一切正常, false=有故障（motor->error已设置）
 */
// Returns true if everything is ok. Sets motor->error and returns false otherwise.
bool do_checks(Motor_t *motor) {
    if (!check_DRV_fault(motor)) {
        motor->error = ERROR_DRV_FAULT;
        // Update DRV Fault Code
        motor->drv_fault = DRV8301_getFaultType(&motor->gate_driver);
        // Update/Cache all SPI device registers
        DRV_SPI_8301_Vars_t *local_regs = &motor->gate_driver_regs;
        local_regs->RcvCmd = true;
        DRV8301_readData(&motor->gate_driver, local_regs);
        return false;
    }
    if (!check_PSU_brownout(motor)) {
        motor->error = ERROR_DC_BUS_BROWNOUT;
        return false;
    }
    return true;
}

/**
 * @brief 控制循环更新（更新转子位置/速度估计）
 * @return true=成功
 */
bool loop_updates(Motor_t *motor) {
    update_rotor(motor);
    return true;
}

/**
 * @brief 电机主控制循环 —— 三环级联控制（位置→速度→电流）
 * @param motor 电机对象指针
 * 
 * ============================================================================
 * 三环控制架构
 * ============================================================================
 * 
 * 这是电机正常运行时的核心控制循环，每个电流采样周期执行一次。
 * 三个控制环从外到内级联：
 * 
 *   ┌─────────────────────────────────────────────────────────┐
 *   │  位置环 (外环)                                           │
 *   │  vel_des = vel_feedforward + pos_gain × (pos_set - pos)  │
 *   │  输出：期望速度 vel_des                                  │
 *   └─────────────────────────────────────────────────────────┘
 *                          │
 *                          ▼
 *   ┌─────────────────────────────────────────────────────────┐
 *   │  速度环 (中环)                                           │
 *   │  Iq = vel_gain × (vel_des - vel) + vel_integrator × ∫err │
 *   │  输出：期望q轴电流 Iq_des                                │
 *   └─────────────────────────────────────────────────────────┘
 *                          │
 *                          ▼
 *   ┌─────────────────────────────────────────────────────────┐
 *   │  电流环 (内环)                                           │
 *   │  FOC_current(Id_des=0, Iq_des)                           │
 *   │  输出：三相PWM波形                                       │
 *   └─────────────────────────────────────────────────────────┘
 * 
 * ============================================================================
 * 控制模式
 * ============================================================================
 * 
 * control_mode >= CTRL_MODE_POSITION_CONTROL : 位置控制（三环全开）
 * control_mode >= CTRL_MODE_VELOCITY_CONTROL : 速度控制（速度环+电流环）
 * control_mode =  CTRL_MODE_CURRENT_CONTROL  : 电流控制（仅电流环）
 * 
 * ============================================================================
 * 关键机制
 * ============================================================================
 * 
 * 1. 速度限幅：vel_des限制在±vel_limit范围内
 * 
 * 2. 齿槽转矩补偿：
 *    如果启用了anti-cogging，根据当前位置查表获取补偿电流：
 *    Iq += cogging_map[pll_pos % encoder_cpr]
 *    mod()函数处理负数情况，确保索引在[0, encoder_cpr)范围内
 * 
 * 3. 方向校正：
 *    编码器模式下，Iq需要乘以motor_dir（1或-1）
 *    因为编码器和电机的旋转方向可能相反
 * 
 * 4. 电流限幅：
 *    Iq限制在±Ilim范围内，Ilim = min(current_lim, max_allowed_current)
 *    max_allowed_current由硬件参数（分流器电阻、DRV8301增益）决定
 * 
 * 5. 速度积分器Anti-windup：
 *    如果电流被限幅(limited=true)，积分器衰减：integral *= 0.99
 *    如果未被限幅，积分器正常累加：integral += Ki × v_err × Ts
 *    这防止了积分器在限幅时过度累积（windup现象）
 * 
 * 6. 电机制动：
 *    每次FOC计算后调用update_brake_current()
 *    将电机回馈到母线的能量通过制动电阻耗散
 * 
 * 7. 云台电机模式：
 *    使用FOC_voltage()代替FOC_current()
 *    电流指令被直接解释为电压指令，不进行电流闭环
 * 
 * 退出条件：
 *   - enable_control = false（外部禁用）
 *   - 电流采样超时
 *   - DRV故障或欠压
 *   - FOC时序错误
 *   - 无感模式下误用位置控制
 * 
 * 退出后的处理：
 *   - Ibus清零
 *   - 更新制动电阻（关闭）
 *   - PWM输出由调用者关闭(MOE=0)
 */
void control_motor_loop(Motor_t *motor) {
//    while (*(motor->axis_legacy.enable_control)) {
    while (motor->enable_control) {
        if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
            motor->error = ERROR_FOC_MEASUREMENT_TIMEOUT;
            break;
        }

        if (!do_checks(motor)) break;
        if (!loop_updates(motor)) break;

        // Only runs if anticogging.calib_anticogging is true; non-blocking
        anti_cogging_calibration(motor);

        // Position control
        // TODO Decide if we want to use encoder or pll position here
        float vel_des = motor->vel_setpoint;
        if (motor->control_mode >= CTRL_MODE_POSITION_CONTROL) {
            if (motor->rotor_mode == ROTOR_MODE_SENSORLESS) {
                motor->error = ERROR_POS_CTRL_DURING_SENSORLESS;
                break;
            }
            float pos_err = motor->pos_setpoint - motor->encoder.pll_pos;
            vel_des += motor->pos_gain * pos_err;
        }

        // Velocity limiting
        float vel_lim = motor->vel_limit;
        if (vel_des > vel_lim) vel_des = vel_lim;
        if (vel_des < -vel_lim) vel_des = -vel_lim;

        // Velocity control
        float Iq = motor->current_setpoint;

        // Anti-cogging is enabled after calibration
        // We get the current position and apply a current feed-forward
        // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
        if (motor->anticogging.use_anticogging) {
            Iq += motor->anticogging.cogging_map[mod(motor->encoder.pll_pos, motor->encoder.encoder_cpr)];
        }

        float v_err = vel_des - get_pll_vel(motor);
        if (motor->control_mode >= CTRL_MODE_VELOCITY_CONTROL) {
            Iq += motor->vel_gain * v_err;
        }

        // Velocity integral action before limiting
        Iq += motor->vel_integrator_current;

        // Apply motor direction correction
        if (motor->rotor_mode == ROTOR_MODE_ENCODER ||
            motor->rotor_mode == ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS) {
            Iq *= motor->encoder.motor_dir;
        }

        // Current limiting
        float Ilim = MACRO_MIN(motor->current_control.current_lim, motor->current_control.max_allowed_current);
        bool limited = false;
        if (Iq > Ilim) {
            limited = true;
            Iq = Ilim;
        }
        if (Iq < -Ilim) {
            limited = true;
            Iq = -Ilim;
        }

        // Velocity integrator (behaviour dependent on limiting)
        if (motor->control_mode < CTRL_MODE_VELOCITY_CONTROL) {
            // reset integral if not in use
            motor->vel_integrator_current = 0.0f;
        } else {
            if (limited) {
                // TODO make decayfactor configurable
                motor->vel_integrator_current *= 0.99f;
            } else {
                motor->vel_integrator_current += (motor->vel_integrator_gain * current_meas_period) * v_err;
            }
        }

        // Execute current command
        if (motor->motor_type == MOTOR_TYPE_HIGH_CURRENT) {
            if (!FOC_current(motor, 0.0f, Iq)) {
                break; // in case of error exit loop, motor->error has been set by FOC_current
            }
        } else if (motor->motor_type == MOTOR_TYPE_GIMBAL) {
            //In gimbal motor mode, current is reinterptreted as voltage.
            if (!FOC_voltage(motor, 0.0f, Iq)) {
                break; // in case of error exit loop, motor->error has been set by FOC_voltage
            }
        } else {
            motor->error = ERROR_NOT_IMPLEMENTED_MOTOR_TYPE;
            break;
        }

        ++(motor->loop_counter);
    }

    //We are exiting control, reset Ibus, and update brake current
    motor->current_control.Ibus = 0.0f;
    update_brake_current();
}

//--------------------------------
// 电机线程 (Motor Thread)
//--------------------------------
/**
 * @brief 电机控制线程 —— 电机生命周期管理
 * @param argument 电机对象指针（Motor_t*）
 * 
 * ============================================================================
 * 电机线程生命周期
 * ============================================================================
 * 
 * 每个电机有一个独立的RTOS线程，管理从初始化到运行的完整生命周期。
 * 
 * 初始化阶段：
 *   1. 分配防齿槽转矩补偿表（cogging_map，大小=encoder_cpr）
 *   2. 获取线程ID，标记thread_ready = true
 * 
 * 主循环（无限循环）：
 * 
 *   ┌─────────────────────────────────────────────────────┐
 *   │  状态1：校准                                         │
 *   │  ┌───────────────────────────────────────────────┐  │
 *   │  │ if (!do_calibration):                         │  │
 *   │  │   1. 使能PWM输出 (MOE=1)                      │  │
 *   │  │   2. 执行motor_calibration()                  │  │
 *   │  │      - 测量相电阻                              │  │
 *   │  │      - 测量相电感                              │  │
 *   │  │      - 编码器偏移校准                          │  │
 *   │  │      - 计算PI/PLL增益                          │  │
 *   │  │   3. 关闭PWM输出 (MOE=0)                      │  │
 *   │  │   4. do_calibration = true（标记已完成）       │  │
 *   │  └───────────────────────────────────────────────┘  │
 *   └─────────────────────────────────────────────────────┘
 *                        │
 *                        ▼
 *   ┌─────────────────────────────────────────────────────┐
 *   │  状态2：运行                                         │
 *   │  ┌───────────────────────────────────────────────┐  │
 *   │  │ if (calibration_ok && enable_control):        │  │
 *   │  │   1. 使能步进脉冲接口 (enable_step_dir=true)  │  │
 *   │  │   2. 使能PWM输出 (MOE=1)                      │  │
 *   │  │   3. 无感模式：执行spin_up_sensorless()       │  │
 *   │  │      - 螺旋升流：电流从0渐变到spin_up_current  │  │
 *   │  │      - 加速到spin_up_target_vel                │  │
 *   │  │   4. 进入control_motor_loop()                 │  │
 *   │  │      - 三环控制循环                            │  │
 *   │  │      - 直到故障或enable_control=false          │  │
 *   │  │   5. 关闭PWM输出 (MOE=0)                      │  │
 *   │  │   6. 禁用步进脉冲接口                          │  │
 *   │  │   7. 如果因故障退出：                          │  │
 *   │  │      calibration_ok = false                   │  │
 *   │  │      enable_control = false                   │  │
 *   │  └───────────────────────────────────────────────┘  │
 *   └─────────────────────────────────────────────────────┘
 *                        │
 *                        ▼
 *   ┌─────────────────────────────────────────────────────┐
 *   │  状态3：待机                                         │
 *   │  输出零电压，等待100ms，回到循环开头                 │
 *   │  如果calibration_ok=false，需要重新校准              │
 *   └─────────────────────────────────────────────────────┘
 * 
 * ============================================================================
 * 无感启动（Spin-up）详解
 * ============================================================================
 * 
 * 无感模式(ROTOR_MODE_SENSORLESS)下，电机启动时需要一个"预启动"过程：
 * 
 *   问题：无感观测器需要反电动势才能工作，而静止时反电动势为零。
 *   解决：先用开环方式驱动电机到一定速度，再切换到闭环。
 * 
 *   Spin-up分为两个阶段：
 * 
 *   阶段1：螺旋升流（Spiral up current）
 *     时间：ramp_up_time = 0.4s
 *     角度：ramp_up_distance = 4π 电弧度
 *     电流：从0线性增加到spin_up_current(10A)
 * 
 *     相位旋转：ph = 4π × x，x从0到1
 *     电流增加：I_mag = spin_up_current × x
 * 
 *     螺旋升流的目的是让转子在逐渐增加的电流下开始旋转，
 *     避免突然的大电流导致失步。
 * 
 *   阶段2：加速（Accelerate）
 *     初始速度：vel = ramp_up_distance / ramp_up_time = 4π/0.4 ≈ 31.4 rad/s
 *     加速度：spin_up_acceleration = 400 rad/s²
 *     目标速度：spin_up_target_vel = 400 rad/s
 * 
 *     每个周期：
 *       vel += acceleration × Ts
 *       phase += vel × Ts
 * 
 *     加速到目标速度后，进入control_motor_loop()，
 *     此时反电动势足够大，无感观测器可以正常工作。
 * 
 * ============================================================================
 * 辅助函数
 * ============================================================================
 * 
 * using_encoder(motor)      : 判断是否使用编码器模式
 * using_sensorless(motor)   : 判断是否使用无感模式
 * get_rotor_phase(motor)    : 获取当前转子电角度
 * get_pll_vel(motor)        : 获取PLL估计的角速度
 * setEncoderCount(motor, c) : 设置编码器计数值（原子操作，关中断保护）
 * update_brake_current()    : 根据所有电机的Ibus总和更新制动电阻电流
 * set_brake_current(I)      : 设置制动电阻PWM占空比
 *   - 占空比 = I × R_brake / Vbus
 *   - 最大占空比限制在90%（给bootstrap电容充电留出时间）
 *   - 死区时间保护：low_off = high_on - DEADTIME
 * 
 * queue_modulation_timings() : 将alpha-beta调制值通过SVM转换为PWM定时
 *   - SVM(mod_alpha, mod_beta, &tA, &tB, &tC)
 *   - next_timings = [tA, tB, tC] × TIM_PERIOD
 * 
 * queue_voltage_timings()    : 将alpha-beta电压转换为PWM定时
 *   - 电压归一化：mod = V / ((2/3) × Vbus)
 *   - 调用queue_modulation_timings()
 *   - 归一化系数(2/3)来自SVM的Clarke变换系数
 */
void motor_thread(void const *argument) {
    Motor_t *motor = (Motor_t *)argument;

    // Allocate the map for anti-cogging algorithm and initialize all values to 0.0f
    int encoder_cpr = motor->encoder.encoder_cpr;
    motor->anticogging.cogging_map = (float *)malloc(encoder_cpr * sizeof(float));
    if (motor->anticogging.cogging_map != NULL) {
        for (int i = 0; i < encoder_cpr; i++) {
            motor->anticogging.cogging_map[i] = 0.0f;
        }
    }

    motor->motor_thread = osThreadGetId();
    motor->thread_ready = true;

    for (;;) {
        if (motor->do_calibration == false) {
            __HAL_TIM_MOE_ENABLE(motor->motor_timer); // enable pwm outputs
            motor_calibration(motor);
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motor->motor_timer); // disables pwm outputs
            motor->do_calibration = true;
        }

        if (motor->calibration_ok && motor->enable_control) {
            motor->enable_step_dir = true;
            __HAL_TIM_MOE_ENABLE(motor->motor_timer);

            bool spin_up_ok = true;
            if (motor->rotor_mode == ROTOR_MODE_SENSORLESS) spin_up_ok = spin_up_sensorless(motor);
            if (spin_up_ok) control_motor_loop(motor);

            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motor->motor_timer);
            motor->enable_step_dir = false;

            if (motor->enable_control) { // if control is still enabled, we exited because of error
                motor->calibration_ok = false;
                motor->enable_control = false;
            }
        }

        queue_voltage_timings(motor, 0.0f, 0.0f);
        osDelay(100);
    }
    motor->thread_ready = false;
}
