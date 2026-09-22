/**
 * @file    dwt_protect.cpp
 * @brief   DWT 守护模块实现（C → C++）
 * @note    原 C 版 RTOS 逻辑不变，删除裸机 TIM 占位分支。
 */
#include "dwt_protect.h"
#include "bsp_dwt.h"

static osTimerId_t dwt_timer = nullptr;

static void DWT_DaemonCallback(void* argument) {
    (void)argument;
    DWT_SysTimeUpdate();
    // 单次定时器，每次超时后重新启动
    if (dwt_timer != nullptr)
        osTimerStart(dwt_timer, DWT_DAEMON_PERIOD_MS);
}

void DWT_DaemonInit() {
    dwt_timer = osTimerNew(DWT_DaemonCallback, osTimerOnce, nullptr, nullptr);
    if (dwt_timer != nullptr)
        osTimerStart(dwt_timer, DWT_DAEMON_PERIOD_MS);
}
