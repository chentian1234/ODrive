/**
 * @file commands_pro.c
 * @brief ODrive 高级二进制命令协议实现
 * 
 * 本文件实现了 ODrive 电机控制器的高级二进制通信协议，是上位机与控制器
 * 之间高效数据交换的核心模块。
 * 
 * 协议特点:
 * 1. 基于 VESC 二进制通信协议扩展，兼容 VESC 工具链
 * 2. 采用命令ID+参数数据的紧凑封装格式，传输效率高
 * 3. 支持电机状态查询、运动控制(占空比/电流/速度/位置)、参数配置等功能
 * 4. 通过回调函数机制实现硬件无关性，可适配 UART/USB/CAN 等多种通信接口
 * 
 * 数据帧格式:
 * [帧头(2字节)] [数据长度(2字节)] [命令ID(1字节)] [参数数据(N字节)] [CRC(2字节)]
 * 
 * 典型使用流程:
 * 1. 初始化时调用 commands_set_send_func() 注册底层发送函数
 * 2. 接收到数据后调用 commands_process_packet() 解析并执行命令
 * 3. 协议通过已注册的回调函数自动发送响应数据
 * 
 * 控制命令使用示例(设置电机速度):
 * 1. 发送 COMM_SET_DUTY + 1.0f 选择 M1 电机
 * 2. 发送 COMM_SET_CURRENT + 5000 (表示 5.0A)
 * 3. 发送 COMM_SET_RPM + 3000 (表示 3000 RPM)
 * 4. 发送 COMM_SET_CURRENT_BRAKE + 2.0f 切换到速度控制模式
 */

#include "commands_pro.h"
#include "packet.h"
#include "buffer.h"
#include "terminal.h"
#include "commands.h"
#include "low_level.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/**
 * @defgroup 私有变量 模块内部私有变量
 * @{
 */

/** 
 * @brief 数据发送缓冲区
 * 
 * 用于构建待发送的响应数据包，大小由 PACKET_MAX_PL_LEN 定义。
 * 该缓冲区为静态变量，避免频繁内存分配。
 */
static uint8_t send_buffer[PACKET_MAX_PL_LEN];

/** @brief 数据包发送回调函数指针 - 当前注册的发送函数 */
static void(*send_func)(unsigned char *data, unsigned int len) = 0;
/** @brief 数据包发送回调函数指针 - 上一次注册的发送函数(用于切换恢复) */
static void(*send_func_last)(unsigned char *data, unsigned int len) = 0;
/** @brief 自定义应用数据接收回调函数指针 */
static void(*appdata_func)(unsigned char *data, unsigned int len) = 0;

/** 
 * @brief 当前目标电机指针
 * 
 * 用于指向当前要控制的电机对象。通过 COMM_SET_DUTY 命令可以选择
 * 控制 M0 或 M1 电机，选择后后续控制命令作用于该电机。
 */
static Motor_t *motor;

/** @brief 位置给定值(单位: 转 或 度，根据配置而定) */
static float pos_setpoint;
/** @brief 速度前馈给定值(单位: RPM) */
static float vel_feed_forward;
/** @brief 电流前馈给定值(单位: A) */
static float current_feed_forward;

/** @} */

/**
 * @brief 设置数据包发送回调函数
 * 
 * 注册底层数据发送函数，当协议需要向外发送响应数据时会调用此回调。
 * 调用者需要提供实际的硬件发送实现(如 UART、USB、CAN 等)。
 * 
 * 设计模式: 回调函数模式(依赖倒置)
 * - 本模块不依赖具体硬件实现，通过回调函数解耦
 * - 便于单元测试和不同平台的移植
 * 
 * @param func 指向数据包发送函数的指针
 *             - data: 待发送的数据缓冲区指针
 *             - len:  待发送数据的长度(字节数)
 * 
 * 使用示例:
 * @code
 *   // UART 发送实现
 *   void uart_send(unsigned char *data, unsigned int len) {
 *       HAL_UART_Transmit(&huart1, data, len, HAL_MAX_DELAY);
 *   }
 *   
 *   // 注册发送函数
 *   commands_set_send_func(uart_send);
 * @endcode
 */
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len)) {
    send_func = func;
}

/**
 * @brief 发送二进制数据包
 * 
 * 通过已注册的发送回调函数将数据发送出去。
 * 内部会先检查 send_func 是否已注册(非 NULL)，未注册则直接返回。
 * 
 * @param data 待发送的数据缓冲区指针
 * @param len  待发送数据的长度(字节数)
 * 
 * @note 调用此函数前必须先调用 commands_set_send_func() 注册发送回调
 * @note 该函数为同步调用，数据发送完成前会阻塞(取决于底层实现)
 */
void commands_send_packet(unsigned char *data, unsigned int len) {
    if (send_func) {
        send_func(data, len);
    }
}

/**
 * @brief 处理接收到的二进制命令数据包
 * 
 * 该函数是二进制命令协议的核心处理入口，负责解析接收到的数据帧，
 * 根据命令类型(COMM_PACKET_ID)执行相应的操作，并通过回调函数返回响应数据。
 * 
 * 数据帧格式:
 * 数据帧经过 packet 层解封装后传入本函数，格式为: [命令ID(1字节)] [参数数据(N字节)]
 * 
 * 命令分发机制:
 * 采用 switch-case 结构进行命令分发，根据 packet_id 跳转到对应的处理分支。
 * 每个 case 分支完成以下工作:
 * 1. 解析参数: 使用 buffer_get_* 系列函数从数据缓冲区中读取参数
 * 2. 执行操作: 调用底层电机控制函数或设置全局变量
 * 3. 返回响应: 构建响应数据并通过 commands_send_packet() 发送
 * 
 * @param data 接收到的数据缓冲区指针(包含命令ID和参数)
 * @param len  接收到的数据长度(字节数)
 * 
 * @note 本函数不处理帧头、长度、CRC校验等，这些由上层 packet 模块处理
 * @note 对于未识别的命令ID，switch 的 default 分支直接返回，不做任何处理
 */
void commands_process_packet(unsigned char *data, unsigned int len) {
    if (!len) {
        return;
    }

    COMM_PACKET_ID packet_id;   // 命令ID，用于识别请求类型
    int32_t ind = 0;            // 缓冲区读写索引

    // 提取命令ID(第一个字节)，并将数据指针后移
    packet_id = data[0];
    data++;
    len--;

    // ============================================================
    // 命令分发处理: 使用 switch-case 结构根据命令ID路由到不同处理分支
    // ============================================================
    switch (packet_id) {
        /* ============================================================
         * 命令: COMM_FW_VERSION - 查询固件版本与设备信息
         * 功能: 返回固件版本号、硬件型号、编译信息、编译时间、芯片唯一ID
         * 请求: 无参数，仅发送命令ID即可
         * 响应: [命令ID(1)] [主版本(1)] [次版本(1)] [硬件名称(N)] [编译类型(N)]
         *       [目标项目(N)] [by(1)] [编译日期(N)] [at(1)] [编译时间(N)]
         *       [0x00分隔符(1)] [芯片ID(12)]
         * ============================================================ */
        case COMM_FW_VERSION:
            ind = 0;
            send_buffer[ind++] = COMM_FW_VERSION;       // 响应命令ID
            send_buffer[ind++] = FW_VERSION_MAJOR;      // 固件主版本号
            send_buffer[ind++] = FW_VERSION_MINOR;      // 固件次版本号

#ifdef HW_NAME
            // 拼接完整的固件版本信息字符串，用于上位机显示设备详情
            strcpy((char*)(send_buffer + ind), HW_NAME);        // 硬件板型名称(如 "34b")
            ind += strlen(HW_NAME);

            strcpy((char*)(send_buffer + ind), __BUILD__);      // 编译工具链("MDK"/"IAR"/"GCC")
            ind += strlen(__BUILD__);

            strcpy((char*)(send_buffer + ind), __FOR__);        // 目标项目标识(如 "ODrive-fw-v0.3.6")
            ind += strlen(__FOR__);

            strcpy((char*)(send_buffer + ind), __BY__);         // 分隔符 " by "
            ind += strlen(__BY__);

            strcpy((char*)(send_buffer + ind), __DATE__);       // 编译日期(如 "Jun  4 2026")
            ind += strlen(__DATE__);

            strcpy((char*)(send_buffer + ind), __AT__);         // 分隔符 " at "
            ind += strlen(__AT__);

            strcpy((char*)(send_buffer + ind), __TIME__);       // 编译时间(如 "14:30:25")
            ind += strlen(__TIME__);

            ind += 1;   // 添加字符串结束符 '\0'

            // 追加 STM32 芯片 96 位唯一 ID(12字节)，用于设备识别和授权验证
            memcpy(send_buffer + ind, STM32_UUID_8, 12);
            ind += 12;
#endif

            commands_send_packet(send_buffer, ind);
            break;

        /* ============================================================
         * 命令: COMM_GET_VALUES - 获取电机实时运行状态数据
         * 功能: 返回电机当前运行的各项关键参数，供上位机显示和监控使用
         * 请求: 无参数，仅发送命令ID即可
         * 
         * 响应数据结构(按顺序，共约 50 字节):
         * +---------+----------+----------+----------+----------+----------+----------+
         * |  偏移   |  字段名   |  类型    |  精度    |  单位    |  说明            |
         * +---------+----------+----------+----------+----------+----------+----------+
         * |    0    | cmdID    |  uint8   |    -     |    -     | 命令ID            |
         * |    1    | T_FET    | float16  |   1e1    |    ℃     | MOS管温度          |
         * |    3    | T_MOTOR  | float16  |   1e1    |    ℃     | 电机温度           |
         * |    5    | I_MOTOR  | float32  |   1e2    |    A     | M0电机母线电流      |
         * |    9    | I_BATT   | float32  |   1e2    |    A     | M1电池母线电流      |
         * |   13    | motor_id | float32  |   1e2    |    A     | motor D轴电流      |
         * |   17    | motor_iq | float32  |   1e2    |    A     | motor Q轴电流(转矩) |
         * |   21    | Duty     | float16  |   1e1    |    -     | 占空比设定值        |
         * |   23    | ERPM     | float32  |   1e0    |  eRPM    | 电气转速(无感PLL速度)|
         * |   27    | V_IN     | float16  |   1e1    |    V     | 母线电压           |
         * |   29    | Ah_DRAW  | float32  |   1e1    |    Ah    | 消耗电量(保留0.0)   |
         * |   33    | Ah_CHRG  | float32  |   1e1    |    Ah    | 充电电量(电流给定)  |
         * |   37    | Wh_GRAW  | float32  |   1e4    |    Wh    | 消耗能量(Id电流)    |
         * |   41    | Wh_CHRG  | float32  |   1e4    |    Wh    | 充电能量(Iq电流)    |
         * |   45    | TAC      |  int32   |    -     |    -     | M0故障码           |
         * |   49    | TAC_ABS  |  int32   |    -     |    -     | M1故障码           |
         * |   53    | Fault    |  uint8   |    -     |    -     | 故障标志(0=正常)    |
         * |   54    | Pos      | float32  |   1e6    |  转/度   | 转子位置(无感PLL)   |
         * +---------+----------+----------+----------+----------+----------+----------+
         * 
         * 说明:
         * - float16: 16位浮点编码(2字节), 实际值 = 编码值 / 精度因子
         * - float32: 32位浮点编码(4字节), 实际值 = 编码值 / 精度因子
         * - 例如: T_FET 精度 1e1，若编码值为 350，则实际温度 = 350/10 = 35.0°C
         * - Ibus: 母线电流，通过相电流计算得出
         * - Id/Iq: FOC 坐标系下的 D/Q 轴电流，Iq 与转矩成正比
         * - sensorless.pll_vel/pos: 无传感器算法估算的速度和位置
         * - exposed_ints[M0/M1_ERROR]: 电机故障状态码
         * ============================================================ */
        case COMM_GET_VALUES:
            ind = 0;
            send_buffer[ind++] = COMM_GET_VALUES;
            buffer_append_float16(send_buffer, *exposed_floats[2], 1e1, &ind);          // [0]  T_FET: MOS管温度(℃)
            buffer_append_float16(send_buffer, *exposed_floats[3], 1e1, &ind);          // [2]  T_MOTOR: 电机温度(℃)
            buffer_append_float32(send_buffer, motors[0].current_control.Ibus, 1e2, &ind); // [4]  I_MOTOR: M0电机母线电流(A)
            buffer_append_float32(send_buffer, motors[1].current_control.Ibus, 1e2, &ind); // [8]  I_BATT: M1电池母线电流(A)
            buffer_append_float32(send_buffer, motors[0].current_control.Id_measured, 1e2, &ind); // [12] motor_id: D轴电流(A)
            buffer_append_float32(send_buffer, motors[0].current_control.Iq_measured, 1e2, &ind); // [16] motor_iq: Q轴电流(A)，与转矩成正比
            buffer_append_float16(send_buffer, motors[0].vel_setpoint, 1e1, &ind);      // [20] Duty: 占空比设定值
            buffer_append_float32(send_buffer, motors[0].sensorless.pll_vel, 1e0, &ind); // [22] ERPM: 电气转速(eRPM)，无感PLL估算值
            buffer_append_float16(send_buffer, *exposed_floats[VBUS_VOLTAGE], 1e1, &ind); // [26] V_IN: 母线输入电压(V)
            buffer_append_float32(send_buffer, 0.0f, 1e1, &ind);                        // [28] Ah_DRAW: 消耗电量(Ah)，保留为0
            buffer_append_float32(send_buffer, motors[0].current_setpoint, 1e1, &ind);  // [32] Ah_CHRG: 充电电量(Ah)
            buffer_append_float32(send_buffer, motors[0].current_control.Id_measured, 1e4, &ind); // [36] Wh_GRAW: 消耗能量(Wh)
            buffer_append_float32(send_buffer, motors[0].current_control.Iq_measured, 1e4, &ind); // [40] Wh_CHRG: 充电能量(Wh)
            buffer_append_int32(send_buffer, *exposed_ints[M0_ERROR], &ind);            // [44] TAC: M0电机故障码
            buffer_append_int32(send_buffer, *exposed_ints[M1_ERROR], &ind);            // [48] TAC_ABS: M1电机故障码
            send_buffer[ind++] = 0;                                                     // [52] Fault: 故障标志位(0=无故障)
            buffer_append_float32(send_buffer, motors[0].sensorless.pll_pos, 1e6, &ind); // [53] pid_pos_mow: 转子位置，无感PLL估算值
            commands_send_packet(send_buffer, ind);
            break;

        /* ============================================================
         * 命令: COMM_SET_DUTY - 选择目标电机(占空比控制命令的扩展用法)
         * 功能: 通过传入不同的参数值来选择要控制的电机(M0 或 M1)
         * 
         * 请求参数: [命令ID(1)] [电机选择(4字节, int32)]
         *   - 参数值 / 100000.0 = 0.0f  → 选择 M0 电机
         *   - 参数值 / 100000.0 = 1.0f  → 选择 M1 电机
         *   - 其他值                   → 无目标电机(选择失败)
         * 
         * 响应: 通过 commands_printf 发送选择结果到上位机
         * 
         * 使用示例:
         *   // 选择 M0 电机: 发送 int32(0 * 100000) = 0
         *   // 选择 M1 电机: 发送 int32(1 * 100000) = 100000
         * 
         * 注意: 此命令仅设置目标电机指针，不发送任何控制信号。
         *       后续的控制命令(COMM_SET_CURRENT/COMM_SET_RPM等)将作用于选中的电机。
         * ============================================================ */
        case COMM_SET_DUTY://D
            ind = 0;
            float m;
//            mc_interface_set_duty((float)buffer_get_int32(data, &ind) / 100000.0f);
            m = (float)buffer_get_int32(data, &ind) / 100000.0f;
            if (m == 0.0f) {
                motor = &motors[0];
                commands_printf("%-32s : oked\n", "Now motor is M0.");
            } else if (m == 1.0f) {
                motor = &motors[1];
                commands_printf("%-32s : oked\n", "Now motor is M1.");
            } else {
                commands_printf("%-32s : failured\n", "Now motor is NONE.");
            }

            break;

        /* ============================================================
         * 命令: COMM_SET_CURRENT - 设置电机电流给定值
         * 功能: 设置电流前馈值(Iq)，用于后续的电流/速度/位置闭环控制
         * 
         * 请求参数: [命令ID(1)] [电流值(4字节, int32)]
         *   - 实际电流 = int32值 / 1000.0 (单位: A)
         *   - 例如: 发送 5000 → 5.000A
         * 
         * 响应: 通过 commands_printf 回显设置的电流值
         * 
         * 使用示例:
         *   // 设置 5A 电流: 发送 int32(5000)
         *   // 设置 -2A 电流: 发送 int32(-2000)
         * 
         * 注意: 此命令仅设置电流前馈变量，不直接执行控制。
         *       需要配合 COMM_SET_CURRENT_BRAKE 命令切换到电流控制模式才生效。
         * ============================================================ */
        case COMM_SET_CURRENT://I
            ind = 0;
//            mc_interface_set_current((float)buffer_get_int32(data, &ind) / 1000.0f);
//            s_current_setpoint(motor, (float)buffer_get_int32(data, &ind) / 1000.0f);
            current_feed_forward = (float)buffer_get_int32(data, &ind) / 1000.0f;
            commands_printf("%-32s : %3.3f\n", "SET_CURRENT", current_feed_forward);

            break;

        /* ============================================================
         * 命令: COMM_SET_CURRENT_BRAKE - 设置电机控制模式(模式切换命令)
         * 功能: 根据传入的模式值，切换到不同的电机控制模式
         * 
         * 请求参数: [命令ID(1)] [模式值(4字节, int32)]
         *   - 模式值 / 1000.0 = 0.0f → 电压控制模式(停止所有电机)
         *   - 模式值 / 1000.0 = 1.0f → 电流控制模式(仅输出给定电流)
         *   - 模式值 / 1000.0 = 2.0f → 速度闭环控制模式(PID速度控制)
         *   - 模式值 / 1000.0 = 3.0f → 位置闭环控制模式(PID位置控制)
         * 
         * 响应: 无直接响应数据
         * 
         * 使用示例(完整控制流程):
         *   1. COMM_SET_DUTY     + 100000    → 选择 M1 电机
         *   2. COMM_SET_CURRENT  + 5000       → 设置电流 5A
         *   3. COMM_SET_RPM      + 3000       → 设置速度 3000 RPM
         *   4. COMM_SET_CURRENT_BRAKE + 2000  → 切换到速度控制模式(启动)
         * 
         * 各模式说明:
         *   模式0: 将所有电机速度设为0，停止运行
         *   模式1: 电流控制，仅输出 current_feed_forward 设定的电流
         *   模式2: 速度控制，以 vel_feed_forward 为目标速度，current_feed_forward 为电流限幅
         *   模式3: 位置控制，以 pos_setpoint 为目标位置，vel_feed_forward 为速度限幅，
         *          current_feed_forward 为电流限幅
         * ============================================================ */
        case COMM_SET_CURRENT_BRAKE://IB
            ind = 0;
            float mode;
//            mc_interface_set_brake_current((float)buffer_get_int32(data, &ind) / 1000.0f);
            mode = (float)buffer_get_int32(data, &ind) / 1000.0f;
            if (mode == 0.0f) {
                // 模式0: 电压控制模式 - 停止所有电机
//                s_mode_setpoint(motor, CTRL_MODE_VOLTAGE_CONTROL);
                for (int i = 0; i < num_motors; i++) {
                    set_vel_setpoint(&motors[i], 0.0f, 0.0f);
                }
            } else if (mode == 1.0f) {
                // 模式1: 电流控制模式 - 仅输出给定电流
//                s_mode_setpoint(motor, CTRL_MODE_CURRENT_CONTROL);
                set_current_setpoint(motor, current_feed_forward);
            } else if (mode == 2.0f) {
                // 模式2: 速度闭环控制模式 - PID速度控制
//                s_mode_setpoint(motor, CTRL_MODE_VELOCITY_CONTROL);
                set_vel_setpoint(motor, vel_feed_forward, current_feed_forward);
            } else if (mode == 3.0f) {
                // 模式3: 位置闭环控制模式 - PID位置控制
//                s_mode_setpoint(motor, CTRL_MODE_POSITION_CONTROL);
                set_pos_setpoint(motor, pos_setpoint, vel_feed_forward, current_feed_forward);
            }

            break;

        /* ============================================================
         * 命令: COMM_SET_RPM - 设置电机速度给定值
         * 功能: 设置速度前馈值，用于速度闭环控制或速度前馈补偿
         * 
         * 请求参数: [命令ID(1)] [速度值(4字节, int32)]
         *   - 实际速度 = int32值 (单位: RPM，无需缩放)
         *   - 正值: 正转; 负值: 反转
         * 
         * 响应: 通过 commands_printf 回显设置的速度值
         * 
         * 使用示例:
         *   // 设置 3000 RPM 正转: 发送 int32(3000)
         *   // 设置 -1500 RPM 反转: 发送 int32(-1500)
         * 
         * 注意: 此命令仅设置速度前馈变量 vel_feed_forward。
         *       需要配合 COMM_SET_CURRENT_BRAKE + 2.0f 切换到速度控制模式才生效。
         * ============================================================ */
        case COMM_SET_RPM://V
            ind = 0;
//            mc_interface_set_pid_speed((float)buffer_get_int32(data, &ind));
//            s_vel_setpoint(motor, (float)buffer_get_int32(data, &ind));
//            set_vel_setpoint(motor, (float)buffer_get_int32(data, &ind), 1.0f);//it is oked.
            vel_feed_forward = (float)buffer_get_int32(data, &ind);
            commands_printf("%-32s : %3.3f\n", "SET_VELOCITY", vel_feed_forward);

            break;

        /* ============================================================
         * 命令: COMM_SET_POS - 设置电机位置给定值
         * 功能: 设置位置前馈值，用于位置闭环控制
         * 
         * 请求参数: [命令ID(1)] [位置值(4字节, int32)]
         *   - 实际位置 = int32值 / 1000000.0 (单位: 转 或 度，根据配置)
         *   - 例如: 发送 1234567 → 1.234567 转
         * 
         * 响应: 通过 commands_printf 回显设置的位置值
         * 
         * 使用示例:
         *   // 设置 2.5 转位置: 发送 int32(2500000)
         *   // 设置 -0.5 转位置: 发送 int32(-500000)
         * 
         * 注意: 此命令仅设置位置前馈变量 pos_setpoint。
         *       需要配合 COMM_SET_CURRENT_BRAKE + 3.0f 切换到位置控制模式才生效。
         * ============================================================ */
        case COMM_SET_POS://P
            ind = 0;
//            mc_interface_set_pid_pos((float)buffer_get_int32(data, &ind) / 1000000.0f);
//            s_pos_setpoint(motor, (float)buffer_get_int32(data, &ind) / 1000000.0f);
            pos_setpoint = (float)buffer_get_int32(data, &ind) / 1000000.0f;
            commands_printf("%-32s : %3.3f\n", "SET_POSITION", pos_setpoint);

            break;

        /* ============================================================
         * 命令: COMM_SET_HANDBRAKE - 设置手刹模式(紧急停止)
         * 功能: 将所有电机的速度设定值和电流限幅设为0，实现紧急停止
         * 
         * 请求参数: 无(或忽略参数)
         * 
         * 响应: 无直接响应数据
         * 
         * 使用示例:
         *   // 发送手刹命令即可，无需额外参数
         *   // 所有电机立即停止运行
         * 
         * 注意: 此命令作用于所有电机(num_motors)，不仅仅是当前选中的目标电机。
         *       适用于紧急停止或安全保护场景。
         * ============================================================ */
        case COMM_SET_HANDBRAKE://HB
            ind = 0;
//            mc_interface_set_handbrake(buffer_get_float32(data, 1e3, &ind));
            for (int i = 0; i < num_motors; i++) {
                set_vel_setpoint(&motors[i], 0.0f, 0.0f);
            }
            break;

        /* ============================================================
         * 命令: COMM_GET_MCCONF / COMM_GET_MCCONF_DEFAULT - 获取电机配置
         * 功能: 返回电机控制器配置参数(当前值或默认值)
         * 响应: 仅返回命令ID(当前为占位实现，未填充实际配置数据)
         * ============================================================ */
        case COMM_GET_MCCONF:
        case COMM_GET_MCCONF_DEFAULT:
            ind = 0;
            send_buffer[ind++] = packet_id;
            commands_send_packet(send_buffer, ind);
            break;

        /* ============================================================
         * 命令: COMM_GET_APPCONF / COMM_GET_APPCONF_DEFAULT - 获取应用配置
         * 功能: 返回应用程序配置参数(当前值或默认值)
         * 响应: 仅返回命令ID(当前为占位实现，未填充实际配置数据)
         * ============================================================ */
        case COMM_GET_APPCONF:
        case COMM_GET_APPCONF_DEFAULT:
            ind = 0;
            send_buffer[ind++] = packet_id;
            commands_send_packet(send_buffer, ind);
            break;

        /* ============================================================
         * 命令: COMM_TERMINAL_CMD - 终端命令执行
         * 功能: 将接收到的字符串作为终端命令执行，支持文本交互方式
         * 请求: [命令ID(1)] [命令字符串(N字节)]
         * 响应: 取决于具体命令(通过 commands_printf 或其他方式返回)
         * 
         * 说明: 此命令允许通过二进制协议通道执行文本命令，
         *       兼容原有的文本终端功能(如 "help", "status" 等)。
         * ============================================================ */
        case COMM_TERMINAL_CMD:
            data[len] = '\0';   // 添加字符串结束符
            terminal_process_string((char*)data);
            break;

        /* ============================================================
         * 命令: COMM_REBOOT - 重启控制器
         * 功能: 通过进入无限循环触发看门狗复位，实现系统重启
         * 请求: 无参数
         * 响应: 无(系统即将重启)
         * 
         * 重启流程:
         * 1. 禁用全局中断 - 阻止任何中断打断重启流程
         * 2. 清除 PendSV 挂起标志 - 清理待处理的中断
         * 3. 禁用所有 NVIC 中断 - 关闭所有中断源
         * 4. 进入无限循环 - 等待看门狗超时触发复位
         * 
         * 注意: 此操作不可逆，执行后控制器将立即重启。
         *       重启前请确保电机已安全停止，避免失控。
         * ============================================================ */
        case COMM_REBOOT:
            // 锁定系统并进入无限循环，由看门狗触发复位
            __disable_irq();
            // 使用 WWDG (窗口看门狗) 复位 MCU

            // 清除待处理的中断标志
            SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;

            // 禁用所有中断
            for (int i = 0; i < 8; i++) {
                NVIC->ICER[i] = NVIC->IABR[i];
            }
            for (;;) {};
            break;

        /* ============================================================
         * 默认分支: 未识别的命令ID
         * 功能: 忽略不支持或未实现的命令，静默处理
         * 说明: 当接收到未知的 packet_id 时，不做任何处理直接返回
         * ============================================================ */
        default:
            break;
    }
}

/**
 * @brief 格式化打印调试信息并发送
 * 
 * 类似 printf 的功能，将格式化字符串通过二进制协议发送出去。
 * 主要用于调试信息的远程输出，上位机接收后可显示在终端中。
 * 
 * 数据格式:
 * [COMM_PRINT(1字节)] [格式化字符串(N字节)]
 * 
 * @param format 格式化字符串(与 printf 格式相同)
 * @param ...    可变参数列表
 * 
 * 使用示例:
 * @code
 *   commands_printf("Motor current: %.2f A\n", current);
 *   commands_printf("Error: %d\n", error_code);
 *   commands_printf("%-32s : %3.3f\n", "SET_VELOCITY", vel_feed_forward);
 * @endcode
 * 
 * @note 缓冲区大小固定为 1023 字节，超出部分将被截断
 * @note 使用 static 缓冲区，函数非线程安全(在单线程嵌入式环境中可接受)
 */
void commands_printf(const char* format, ...) {
    va_list arg;
    va_start(arg, format);
    int len;
    static char print_buffer[1023];

    print_buffer[0] = COMM_PRINT;   // 第一个字节为打印命令ID
    len = vsnprintf(print_buffer+1, 1022, format, arg);  // 格式化字符串到缓冲区
    va_end(arg);

    if (len > 0) {
        commands_send_packet((unsigned char*)print_buffer, (len<1022)? len+1: 1023);
    }
}

/**
 * @brief 发送电机转子位置数据
 * 
 * 用于向上位机实时推送电机的转子位置信息(角度值)。
 * 上位机可用于可视化显示转子位置或进行控制算法调试。
 * 
 * 数据格式:
 * [COMM_ROTOR_POSITION(1字节)] [转子位置(4字节, int32)]
 *   - 位置编码值 = 实际位置 * 100000.0
 *   - 实际位置 = 编码值 / 100000.0 (单位: 转 或 度)
 * 
 * @param rotor_pos 转子位置值
 */
void commands_send_rotor_pos(float rotor_pos) {
    uint8_t buffer[5];
    int32_t index = 0;

    buffer[index++] = COMM_ROTOR_POSITION;
    buffer_append_int32(buffer, (int32_t)(rotor_pos * 100000.0f), &index);

    commands_send_packet(buffer, index);
}

/**
 * @brief 发送实验采样数据
 * 
 * 用于向上位机推送批量实验数据(如波形采集、调试数据等)。
 * 数据以 int32 数组形式发送，每个值经过 10000 倍放大以保持精度。
 * 
 * 数据格式:
 * [COMM_EXPERIMENT_SAMPLE(1字节)] [采样1(4字节)] [采样2(4字节)] ...
 *   - 每个采样值编码 = 实际值 * 10000.0
 * 
 * @param samples 采样数据浮点数组
 * @param len     采样数据个数
 * 
 * @note 数据总量限制为 256 字节以内，超过则直接返回不发送
 *       即: len * 4 + 1 <= 256，最大支持 63 个采样点
 */
void commands_send_experiment_samples(float *samples, int len) {
    if ((len * 4 + 1) > 256) {
        return;
    }

    uint8_t buffer[len * 4 + 1];
    int32_t index = 0;

    buffer[index++] = COMM_EXPERIMENT_SAMPLE;

    for (int i = 0; i < len; i++) {
        buffer_append_int32(buffer, (int32_t)(samples[i] * 10000.0f), &index);
    }

    commands_send_packet(buffer, index);
}