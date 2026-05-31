/**
 * @brief Minimal runtime symbols for Keil C-only build (scheme A).
 * Full ODrive firmware provides these in MotorControl/main.cpp and communication modules.
 */

#include <stdbool.h>
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOSConfig.h"
#include "gpio.h"
#include "dma.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"

/* FreeRTOS heap (see configAPPLICATION_ALLOCATED_HEAP in FreeRTOSConfig.h) */
uint8_t ucHeap[configTOTAL_HEAP_SIZE];

/* USB descriptor serial string (normally set in communication.cpp / main.cpp) */
char serial_number_str[13] = "000000000000";

/* USB event queue (normally created in MotorControl/main.cpp) */
osMessageQId usb_event_queue;

void usb_rx_process_packet(uint8_t *buf, uint32_t len, uint8_t endpoint_pair)
{
    (void)buf;
    (void)len;
    (void)endpoint_pair;
}

void system_init(void)
{
}

bool board_init(void)
{
    return true;
}

void start_timers(void)
{
}

extern void SystemClock_Config(void);
extern void MX_FREERTOS_Init(void);

static void odrive_create_usb_event_queue(void)
{
    osMessageQDef(usb_event_queue, 7, uint32_t);
    usb_event_queue = osMessageCreate(osMessageQ(usb_event_queue), NULL);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_TIM1_Init();
    MX_TIM8_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_SPI3_Init();
    MX_ADC3_Init();
    MX_TIM2_Init();
    MX_UART4_Init();
    MX_TIM5_Init();
    MX_TIM13_Init();

    odrive_create_usb_event_queue();
    MX_FREERTOS_Init();

    osKernelStart();

    while (1) {
    }
}
