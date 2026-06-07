/*
 * ============================================================================
 * 文件名: version.h
 *
 * 文件用途:
 *   本文件定义ODrive固件版本号、硬件版本标识和编译环境信息。
 *   版本号在系统启动时通过串口/USB输出，用于固件识别和版本管理。
 *   支持Keil MDK、IAR、GCC三种编译工具的自动识别。
 *
 * 作者: ODrive Robotics
 * 版本: v0.3.6
 * ============================================================================
 */

#define ODRIVE_FW_VERSION_MAJOR   0    /* 主版本号：重大架构变更时增加 */
#define ODRIVE_FW_VERSION_MINOR   3    /* 次版本号：新功能添加时增加 */
#define ODRIVE_FW_VERSION_PATCH   6    /* 修订号：Bug修复时增加 */

#define HW_NAME					"3.4b" /* 硬件版本号：对应ODrive V3.4b电路板 */

/* 编译工具链自动识别 */
#if defined ARM_MDK
#define __BUILD__					" MDK "   /* Keil MDK (ARM Compiler) */
#elif defined ARM_IAR
#define __BUILD__					" IAR "   /* IAR EWARM */
#else
#define __BUILD__					" GCC "   /* GNU Arm Embedded Toolchain */
#endif

#define __BY__   " by "
#define __AT__   " at "

#define ARM_APP_NEW               /* 定义此宏表示当前编译的是应用程序固件 */

#if 1
#if defined ARM_APP_NEW
#define __FOR__					"app2.0"  /* 应用程序版本2.0 */
#elif defined ARM_BOOT_NEW
#define __FOR__					"boot1.0" /* Bootloader版本1.0 */
#else
#define __FOR__					"none1.0" /* 未定义版本 */
#endif
#define FW_CHANGELOG_TIME      "23/05/2018 11:22:59"  /* 最后修改时间 */
/*
添加:
修改:
新增:
删除:
修复:
优化:
破坏:
*/
#elif 1
#else
#endif
