/**
 * @file terminal.c
 * @brief 简易命令行终端实现
 *
 * 该文件实现了 ODrive 固件的命令行终端功能，主要特性包括：
 * - 命令行解析：按空格分割用户输入的字符串为参数列表
 * - 内置命令集：支持电机控制（位置/速度/电流）、参数读写、设备信息查询等
 * - 回调机制：支持最多32个自定义命令回调，可扩展终端功能
 * - 命令分发：通过 if-else 链匹配内置命令，未匹配时查找注册的回调
 */

#include "terminal.h"
#include "commands_pro.h"
#include "utils.h"

#include "low_level.h"
#include "commands.h"

#include "version.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

/* ============================================================
 * 配置参数
 * ============================================================ */

/**
 * @brief 自定义命令回调的最大注册数量
 *
 * 终端最多可注册32个自定义命令回调。当注册数量达到上限后，
 * callback_write 会回绕到0，覆盖最早注册的命令。
 */
#define CALLBACK_LEN                        32

/* ============================================================
 * 私有类型定义
 * ============================================================ */

/**
 * @brief 自定义命令回调结构体
 *
 * 每个注册的自定义命令都存储在此结构中，包含：
 * - command: 命令名称（用于匹配用户输入）
 * - help: 帮助文本（在 help 命令中显示）
 * - arg_names: 参数说明（如 "[arg1] [arg2]"）
 * - cbf: 回调函数指针，命令匹配时调用
 */
typedef struct _terminal_callback_struct {
    const char *command;        /**< 命令名称 */
    const char *help;           /**< 帮助文本 */
    const char *arg_names;      /**< 参数名称说明 */
    void(*cbf)(int argc, const char **argv);  /**< 回调函数指针 */
} terminal_callback_struct;

/* ============================================================
 * 私有变量
 * ============================================================ */

/**
 * @brief 自定义命令回调数组
 *
 * 存储所有已注册的自定义命令回调，最大容量为 CALLBACK_LEN。
 * 使用静态存储，生命周期贯穿整个程序运行期间。
 */
static terminal_callback_struct callbacks[CALLBACK_LEN];

/**
 * @brief 回调数组写指针/计数器
 *
 * 指示下一个可用的回调槽位索引。每次注册新命令时递增，
 * 达到 CALLBACK_LEN 上限后回绕到0。
 */
static int callback_write = 0;

/**
 * @brief 命令接收计数器
 *
 * 每处理一条命令递增，在输出中显示命令序号（如 "In [0] : help"），
 * 方便调试和追踪命令执行历史。
 */
static uint32_t comm_received_cnt = 0;

/* ============================================================
 * 公共函数实现
 * ============================================================ */

/**
 * @brief 处理用户输入的命令行字符串
 *
 * 命令行解析过程：
 * 1. 使用 strtok() 以空格为分隔符，将输入字符串分割为多个参数
 * 2. 参数存储在 argv 数组中，argc 记录参数个数
 * 3. 最多支持 kMaxArgs（64）个参数
 * 4. 根据 argv[0]（命令名）进行命令查找和分发
 *
 * 命令查找和分发逻辑：
 * - 首先匹配一系列 if-else 分支，覆盖所有内置命令
 * - 如果内置命令都不匹配，则遍历 callbacks 数组查找自定义回调
 * - 如果仍未找到，则提示用户命令无效
 *
 * @param str 用户输入的命令行字符串（会被 strtok 修改）
 */
void terminal_process_string(char *str) {
    /* 最大参数数量限制 */
    enum { kMaxArgs = 64 };
    int argc = 0;                    /* 实际参数个数 */
    char *argv[kMaxArgs];            /* 参数指针数组 */

    /* ============================================================
     * 命令行解析：按空格分割参数
     *
     * strtok() 第一次调用时传入原始字符串，后续调用传入 NULL 继续
     * 分割，直到返回 NULL 表示分割完成。
     * ============================================================ */
    char *p2 = strtok(str, " ");     /* 获取第一个参数（命令名） */
    while (p2 && argc < kMaxArgs) {
        argv[argc++] = p2;           /* 存储参数指针，递增计数 */
        p2 = strtok(0, " ");         /* 继续分割下一个参数 */
    }

    /* 打印命令接收信息，包括序号和命令名 */
    commands_printf("\nIn [%u] : %s", comm_received_cnt++, argv[0]);
    commands_printf("----------------------------------------------------------------");

    /* 如果没有解析到任何参数，直接返回 */
    if (argc == 0) {
        commands_printf("No command received\n");
        return;
    }

    /* ============================================================
     * 命令查找和分发逻辑
     *
     * 使用 if-else 链依次匹配内置命令。每个命令分支：
     * 1. 检查参数数量是否符合要求
     * 2. 解析参数值（使用 sscanf）
     * 3. 执行对应的操作
     * ============================================================ */

    /* ----------------------------------------------------------
     * 命令: p (Position Control - 位置控制)
     *
     * 用法: p <motor_number> <pos_setpoint> <vel_feed_forward> <current_feed_forward>
     *
     * 参数说明：
     *   motor_number: 电机编号（0 或 1），必须小于 num_motors
     *   pos_setpoint: 目标位置设定点（弧度）
     *   vel_feed_forward: 速度前馈值（rad/s）
     *   current_feed_forward: 电流前馈值（A）
     *
     * 功能：设置指定电机的位置控制模式设定点
     * ---------------------------------------------------------- */
    if (strcmp(argv[0], "p") == 0) {
        unsigned motor_number;
        float pos_setpoint, vel_feed_forward, current_feed_forward;

        sscanf(argv[1], "%u", &motor_number);
        if (argc == 5 && motor_number < num_motors) {
            sscanf(argv[2], "%f", &pos_setpoint);
            sscanf(argv[3], "%f", &vel_feed_forward);
            sscanf(argv[4], "%f", &current_feed_forward);
            set_pos_setpoint(&motors[motor_number], pos_setpoint, vel_feed_forward, current_feed_forward);
        } else {
            commands_printf("This command requires 5 argument.\n");
        }

    /* ----------------------------------------------------------
     * 命令: v (Velocity Control - 速度控制)
     *
     * 用法: v <motor_number> <vel_feed_forward> <current_feed_forward>
     *
     * 参数说明：
     *   motor_number: 电机编号
     *   vel_feed_forward: 目标速度设定点（rad/s）
     *   current_feed_forward: 电流前馈值（A）
     *
     * 功能：设置指定电机的速度控制模式设定点
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "v") == 0) {
        unsigned motor_number;
        float vel_feed_forward, current_feed_forward;
        sscanf(argv[1], "%u", &motor_number);
        if (argc == 4 && motor_number < num_motors) {
            sscanf(argv[2], "%f", &vel_feed_forward);
            sscanf(argv[3], "%f", &current_feed_forward);
            set_vel_setpoint(&motors[motor_number], vel_feed_forward, current_feed_forward);
        } else {
            commands_printf("This command requires 4 argument.\n");
        }

    /* ----------------------------------------------------------
     * 命令: c (Current Control - 电流控制)
     *
     * 用法: c <motor_number> <current_feed_forward>
     *
     * 参数说明：
     *   motor_number: 电机编号
     *   current_feed_forward: 目标电流设定点（A）
     *
     * 功能：设置指定电机的电流控制模式设定点
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "c") == 0) {
        unsigned motor_number;
        float current_feed_forward;
        sscanf(argv[1], "%u", &motor_number);
        if (argc == 3 && motor_number < num_motors) {
            sscanf(argv[2], "%f", &current_feed_forward);
            set_current_setpoint(&motors[motor_number], current_feed_forward);
        } else {
            commands_printf("This command requires 3 argument.\n");
        }

    /* ----------------------------------------------------------
     * 命令: i (Info - 设备信息)
     *
     * 用法: i
     *
     * 功能：打印芯片相关信息，包括：
     *   - Signature: STM32 芯片签名（用于识别芯片型号）
     *   - Revision: 芯片修订号
     *   - Flash Size: Flash 存储器大小（KiB）
     *   - UUID: 芯片唯一标识符（96位）
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "i") == 0) {// Dump device info
        commands_printf("Signature: %#x\n", STM_ID_GetSignature());
        commands_printf("Revision: %#x\n", STM_ID_GetRevision());
        commands_printf("Flash Size: %#x KiB\n", STM_ID_GetFlashSize());
        commands_printf("UUID: 0x%lx%lx%lx\n", STM_ID_GetUUID(2), STM_ID_GetUUID(1), STM_ID_GetUUID(0));

    /* ----------------------------------------------------------
     * 命令: g (Get - 读取参数值，数字类型索引)
     *
     * 用法: g <type> <index>
     *
     * 参数说明：
     *   type: 数据类型（0=float, 1=int, 2=bool, 3=uint16）
     *   index: 对应类型数组中的索引
     *
     * 功能：读取 exposed_* 全局数组中指定索引的参数值
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "g") == 0) { // GET
        int type = 0;
        int index = 0;
        if (argc == 3) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 0: {
                    commands_printf("%f\n",*exposed_floats[index]);
                    break;
                };
                case 1: {
                    commands_printf("%d\n",*exposed_ints[index]);
                    break;
                };
                case 2: {
                    commands_printf("%d\n",*exposed_bools[index]);
                    break;
                };
                case 3: {
                    commands_printf("%hu\n",*exposed_uint16[index]);
                    break;
                };
            }
        } else {
            commands_printf("This command requires 3 argument.\n");
        }

    /* ----------------------------------------------------------
     * 命令: get (Get - 读取参数值，字符类型索引)
     *
     * 用法: get <type> <index>
     *
     * 参数说明：
     *   type: 数据类型（f=float, i=int, b=bool, u=uint16）
     *   index: 对应类型数组中的索引
     *
     * 功能：与 'g' 命令类似，但使用字符标识类型，更易读
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "get") == 0) { // GET
        char type;
        int index = 0;
        if (argc == 3) {
            sscanf(argv[1], "%s", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 'f': {
                    commands_printf("%f\n",*exposed_floats[index]);
                    break;
                };
                case 'i': {
                    commands_printf("%d\n",*exposed_ints[index]);
                    break;
                };
                case 'b': {
                    commands_printf("%d\n",*exposed_bools[index]);
                    break;
                };
                case 'u': {
                    commands_printf("%hu\n",*exposed_uint16[index]);
                    break;
                };
            }
        } else {
            commands_printf("This command requires 3 argument.\n");
        }

    /* ----------------------------------------------------------
     * 命令: h (Halt - 停止所有电机)
     *
     * 用法: h
     *
     * 功能：遍历所有电机，将速度设定点和电流前馈都设为0，
     *       使所有电机停止运转。紧急情况下使用。
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "h") == 0) { // HALT
        for (int i = 0; i < num_motors; i++) {
            set_vel_setpoint(&motors[i], 0.0f, 0.0f);
        }

    /* ----------------------------------------------------------
     * 命令: s (Set - 设置参数值，数字类型索引)
     *
     * 用法: s <type> <index> <value>
     *
     * 参数说明：
     *   type: 数据类型（0=float, 1=int, 2=bool, 3=uint16）
     *   index: 对应类型数组中的索引
     *   value: 要设置的新值
     *
     * 功能：将值写入 exposed_* 全局数组中指定索引的位置
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "s") == 0) { // SET
        int type = 0;
        int index = 0;
        if (argc == 4) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 0: {
                    sscanf(argv[3], "%f", exposed_floats[index]);
                    break;
                };
                case 1: {
                    sscanf(argv[3], "%d", exposed_ints[index]);
                    break;
                };
                case 2: {
                    int btmp = 0;
                    sscanf(argv[3], "%d", &btmp);
                    *exposed_bools[index] = btmp ? true : false;
                    break;
                };
                case 3: {
                    sscanf(argv[3], "%hu", exposed_uint16[index]);
                    break;
                };
            }
        } else {
            commands_printf("This command requires 4 argument.\n");
        }

    /* ----------------------------------------------------------
     * 命令: set (Set - 设置参数值，字符类型索引)
     *
     * 用法: set <type> <index> <value>
     *
     * 参数说明：
     *   type: 数据类型（f=float, i=int, b=bool, u=uint16）
     *   index: 对应类型数组中的索引
     *   value: 要设置的新值
     *
     * 功能：与 's' 命令类似，但使用字符标识类型，更易读
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "set") == 0) { // SET
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
            commands_printf("This command requires 4 argument.\n");
        }

    /* ----------------------------------------------------------
     * 命令: m (Monitor Setup - 配置监控槽位)
     *
     * 用法: m <type> <index> <slot>
     *
     * 参数说明：
     *   type: 数据类型（0=float, 1=int, 2=bool, 3=uint16）
     *   index: 对应类型数组中的索引
     *   slot: 监控槽位编号
     *
     * 功能：将指定的 exposed 变量绑定到监控槽位，
     *       后续可通过 'o' 命令周期性读取这些变量的值。
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "m") == 0) { // Setup Monitor
        int type = 0;
        int index = 0;
        int slot = 0;
        if (argc == 4) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            sscanf(argv[3], "%u", &slot);
            monitoring_slots[slot].type = type;
            monitoring_slots[slot].index = index;
        } else {
            commands_printf("This command requires 4 argument.\n");
        }

    /* ----------------------------------------------------------
     * 命令: o (Output Monitor - 输出监控数据)
     *
     * 用法: o <limit>
     *
     * 参数说明：
     *   limit: 输出的采样点数量限制
     *
     * 功能：打印已配置的监控槽位的数据，limit 参数控制输出数量
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "o") == 0) { // Output Monitor
        int limit = 0;
        if (argc == 2) {
            sscanf(argv[1], "%u", &limit);
            print_monitoring(limit);
        } else {
            commands_printf("This command requires 2 argument.\n");
        }

    /* ----------------------------------------------------------
     * 命令: t (Anti-Cogging Calibration - 反齿槽转矩校准)
     *
     * 用法: t
     *
     * 功能：对所有电机启动反齿槽转矩校准过程。
     *       仅当电机已分配 cogging_map 且无错误时才会执行。
     *       反齿槽转矩校准用于补偿电机齿槽效应引起的转矩波动。
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "t") == 0) { // Run Anti-Cogging Calibration
        for (int i = 0; i < num_motors; i++) {
            /* 确保齿槽映射表已正确分配，且电机处于无错误状态 */
            if (motors[i].anticogging.cogging_map != NULL && motors[i].error == ERROR_NO_ERROR) {
                motors[i].anticogging.calib_anticogging = true;
            }
        }

    /* ----------------------------------------------------------
     * 命令: getf (Get Floats - 列出所有浮点型参数)
     *
     * 用法: getf
     *
     * 功能：遍历 exposed_floats 数组，打印所有浮点型变量的
     *       名称和当前值，包括：
     *       - 系统参数：VBUS_VOLTAGE（母线电压）、NULLED
     *       - 电机0参数：位置/速度/电流设定点、增益、电感、电阻、
     *         电流测量值、编码器参数、电流控制参数等
     *       - 电机1参数：同上
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "getf") == 0) {
        int index = 0;
        int i = 0;
        commands_printf("[%d]%-32s : %f\n", i++, "VBUS_VOLTAGE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "NULLED", *exposed_floats[index++]);
        commands_printf("----------------------------------------------------------------\n");
        commands_printf("[%d]%-32s : %f\n", i++, "M0_POS_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_POS_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_VEL_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_VEL_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_VEL_INTEGRATOR_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_VEL_INTEGRATOR_CURRENT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_VEL_LIMIT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CALIBRATION_CURRENT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_PHASE_INDUCTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_PHASE_RESISTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_MEAS_PHB", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_MEAS_PHC", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_DC_CALIB_PHB", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_DC_CALIB_PHC", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_SHUNT_CONDUCTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_PHASE_CURRENT_REV_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_CURRENT_LIM", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_P_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_I_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_D", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_Q", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_IBUS", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_ENCODER_PHASE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_ENCODER_PLL_POS", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_ENCODER_PLL_VEL", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_ENCODER_PLL_KP", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_ENCODER_PLL_KI", *exposed_floats[index++]);
        commands_printf("----------------------------------------------------------------\n");
        commands_printf("[%d]%-32s : %f\n", i++, "M1_POS_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_POS_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_VEL_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_VEL_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_VEL_INTEGRATOR_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_VEL_INTEGRATOR_CURRENT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_VEL_LIMIT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CALIBRATION_CURRENT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_PHASE_INDUCTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_PHASE_RESISTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_MEAS_PHB", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_MEAS_PHC", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_DC_CALIB_PHB", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_DC_CALIB_PHC", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_SHUNT_CONDUCTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_PHASE_CURRENT_REV_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_CURRENT_LIM", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_P_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_I_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_D", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_Q", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_IBUS", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_ENCODER_PHASE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_ENCODER_PLL_POS", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_ENCODER_PLL_VEL", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_ENCODER_PLL_KP", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_ENCODER_PLL_KI", *exposed_floats[index++]);

    /* ----------------------------------------------------------
     * 命令: geti (Get Ints - 列出所有整型参数)
     *
     * 用法: geti
     *
     * 功能：打印 exposed_ints 数组中所有整型变量，包括：
     *       - 电机控制模式、编码器偏移、编码器状态、错误代码
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "geti") == 0) {
        int index = 0;
        int i = 0;
        commands_printf("[%d]%-32s : %d\n", i++, "M0_CONTROL_MODE", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_ENCODER_ENCODER_OFFSET", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_ENCODER_ENCODER_STATE", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_ERROR", *exposed_ints[index++]);
        commands_printf("----------------------------------------------------------------\n");
        commands_printf("[%d]%-32s : %d\n", i++, "M1_CONTROL_MODE", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_ENCODER_ENCODER_OFFSET", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_ENCODER_ENCODER_STATE", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_ERROR", *exposed_ints[index++]);

    /* ----------------------------------------------------------
     * 命令: getb (Get Bools - 列出所有布尔型参数)
     *
     * 用法: getb
     *
     * 功能：打印 exposed_bools 数组中所有布尔型变量，包括：
     *       - 线程就绪状态、控制使能、校准触发、校准状态
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "getb") == 0) {
        int index = 0;
        int i = 0;
        commands_printf("[%d]%-32s : %d\n", i++, "M0_THREAD_READY", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_ENABLE_CONTROL", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_DO_CALIBRATION", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_CALIBRATION_OK", *exposed_bools[index++]);
        commands_printf("----------------------------------------------------------------\n");
        commands_printf("[%d]%-32s : %d\n", i++, "M1_THREAD_READY", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_ENABLE_CONTROL", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_DO_CALIBRATION", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_CALIBRATION_OK", *exposed_bools[index++]);

    /* ----------------------------------------------------------
     * 命令: getu (Get UInt16 - 列出所有 uint16 类型参数)
     *
     * 用法: getu
     *
     * 功能：打印 exposed_uint16 数组中所有 uint16 变量，包括：
     *       - 控制截止时间、最后CPU时间
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "getu") == 0) {
        int index = 0;
        int i = 0;
        commands_printf("[%d]%-32s : %hu\n", i++, "M0_CONTROL_DEADLINE", *exposed_uint16[index++]);
        commands_printf("[%d]%-32s : %hu\n", i++, "M0_LAST_CPU_TIME", *exposed_uint16[index++]);
        commands_printf("----------------------------------------------------------------\n");
        commands_printf("[%d]%-32s : %hu\n", i++, "M1_CONTROL_DEADLINE", *exposed_uint16[index++]);
        commands_printf("[%d]%-32s : %hu\n", i++, "M1_LAST_CPU_TIME", *exposed_uint16[index++]);

    /* ----------------------------------------------------------
     * 命令: hw_status (Hardware Status - 硬件状态)
     *
     * 用法: hw_status
     *
     * 功能：打印硬件和固件相关信息：
     *       - 固件版本号（MAJOR.MINOR.PATCH）
     *       - 硬件名称（如果定义了 HW_NAME）
     *       - 构建信息（编译日期和目标平台）
     *       - 固件更新日志时间
     *       - STM32 芯片 UUID（12字节，十六进制格式）
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "hw_status") == 0) {
        commands_printf("Firmware: %d.%d.%d", ODRIVE_FW_VERSION_MAJOR, ODRIVE_FW_VERSION_MINOR, ODRIVE_FW_VERSION_PATCH);
#ifdef HW_NAME
        commands_printf("Hardware: %s", HW_NAME);
#endif
        commands_printf("Firmware version: %s %s", __BUILD__, __FOR__);
        commands_printf("Firmware changelog time: %s", FW_CHANGELOG_TIME);

        commands_printf("UUID: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                        STM32_UUID_8[0], STM32_UUID_8[1], STM32_UUID_8[2], STM32_UUID_8[3],
                        STM32_UUID_8[4], STM32_UUID_8[5], STM32_UUID_8[6], STM32_UUID_8[7],
                        STM32_UUID_8[8], STM32_UUID_8[9], STM32_UUID_8[10], STM32_UUID_8[11]);

    /* ----------------------------------------------------------
     * 命令: help (帮助信息)
     *
     * 用法: help
     *
     * 功能：显示所有可用命令的帮助信息，包括：
     *       1. 内置命令（help、hw_status）
     *       2. 所有已注册的自定义命令回调
     *          - 显示命令名和参数说明（如果有）
     *          - 显示帮助文本（如果有）
     * ---------------------------------------------------------- */
    } else if (strcmp(argv[0], "help") == 0) {
        commands_printf("Valid commands are:");
        commands_printf("help");
        commands_printf("  Show this help");

        commands_printf("hw_status");
        commands_printf("  Print some hardware status information.");

        /* 遍历所有已注册的自定义命令回调，显示其帮助信息 */
        for (int i = 0; i < callback_write; i++) {
            if (callbacks[i].arg_names) {
                commands_printf("%s %s", callbacks[i].command, callbacks[i].arg_names);
            } else {
                commands_printf(callbacks[i].command);
            }

            if (callbacks[i].help) {
                commands_printf("  %s", callbacks[i].help);
            } else {
                commands_printf("  There is no help available for this command.");
            }
        }

        commands_printf(" ");

    /* ----------------------------------------------------------
     * 未匹配的命令：在自定义回调中查找
     *
     * 如果输入的命令不匹配任何内置命令，则遍历 callbacks 数组
     * 查找已注册的自定义命令回调。
     *
     * 回调机制说明：
     * - callbacks 数组最多存储 CALLBACK_LEN（32）个回调
     * - 使用 strcmp 比较命令名，找到第一个匹配的回调
     * - 调用匹配回调的 cbf 函数，传入 argc 和 argv
     * - 如果遍历完所有回调都未找到匹配，提示命令无效
     * ---------------------------------------------------------- */
    } else {
        bool found = false;
        for (int i = 0; i < callback_write; i++) {
            if (strcmp(argv[0], callbacks[i].command) == 0) {
                callbacks[i].cbf(argc, (const char**)argv);  /* 调用匹配的回调函数 */
                found = true;
                break;
            }
        }

        if (!found) {
            commands_printf("Invalid command: %s\n"
                            "type help to list all available commands\n", argv[0]);
        }
    }
}


/**
 * @brief 注册自定义命令回调函数
 *
 * 向终端注册一个新的命令回调。注册后，当用户输入的命令不匹配
 * 任何内置命令时，系统会查找已注册的回调并调用匹配的回调函数。
 *
 * 回调注册逻辑：
 * 1. 首先检查是否已存在相同 command 指针地址的回调（防止重复注册）
 * 2. 然后使用 strcmp 比较命令名字符串（检测同名不同地址的情况）
 * 3. 如果找到已存在的回调，则覆盖旧的回调（更新为新的回调函数）
 * 4. 如果是新命令，则在下一个可用槽位注册，并递增 callback_write
 * 5. 当 callback_write 达到 CALLBACK_LEN 上限时，回绕到0
 *
 * 回调机制特点：
 * - 最多支持32个自定义命令（CALLBACK_LEN）
 * - 支持覆盖已注册的命令（同名命令重新注册）
 * - 达到上限后回绕，覆盖最早的命令（循环缓冲区行为）
 * - 线程安全：无锁实现，假设在单线程环境中调用
 *
 * @param command   命令名称字符串指针（如 "my_cmd"）
 * @param help      帮助文本，可为 NULL
 * @param arg_names 参数名称说明，可为 NULL（如 "[arg1] [arg2]"）
 * @param cbf       回调函数指针，签名: void(int argc, const char **argv)
 *                  - argc: 参数个数（包含命令名本身）
 *                  - argv: 参数数组，argv[0] 为命令名
 */
void terminal_register_command_callback(
    const char* command,
    const char *help,
    const char *arg_names,
    void(*cbf)(int argc, const char **argv)) {

    int callback_num = callback_write;   /* 默认使用下一个可用槽位 */

    /* ============================================================
     * 检查命令是否已注册（去重/覆盖逻辑）
     *
     * 使用两种方式检查：
     * 1. 指针地址比较：检测完全相同的 command 字符串指针
     * 2. 字符串内容比较：检测命令名相同但指针不同的情况
     * ============================================================ */
    for (int i = 0; i < callback_write; i++) {
        /* 首先比较指针地址，因为相同的回调可能被多次注册 */
        if (callbacks[i].command == command) {
            callback_num = i;    /* 找到相同指针，复用该槽位 */
            break;
        }

        /* 使用字符串内容比较，检测同名命令 */
        if (strcmp(callbacks[i].command, command) == 0) {
            callback_num = i;    /* 找到同名命令，复用该槽位 */
            break;
        }
    }

    /* 写入/更新回调结构体 */
    callbacks[callback_num].command = command;
    callbacks[callback_num].help = help;
    callbacks[callback_num].arg_names = arg_names;
    callbacks[callback_num].cbf = cbf;

    /* 如果是新注册的命令（而非覆盖），递增写指针 */
    if (callback_num == callback_write) {
        callback_write++;
        /* 达到上限时回绕，实现循环缓冲区 */
        if (callback_write >= CALLBACK_LEN) {
            callback_write = 0;
        }
    }
}
