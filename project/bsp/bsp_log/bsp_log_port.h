#ifndef BSP_LOG_PORT_H
#define BSP_LOG_PORT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_LOG_PLATFORM  "STM32F407"

static inline uint32_t bsp_log_get_tick_ms(void)
{
    return HAL_GetTick();
}

#include "SEGGER_RTT.h"

#define BSP_LOG_OUTPUT(buf)  SEGGER_RTT_WriteString(0, buf)

#define BSP_LOG_LOCK()    SEGGER_RTT_LOCK()
#define BSP_LOG_UNLOCK()  SEGGER_RTT_UNLOCK()

#ifdef __cplusplus
}
#endif

#endif
