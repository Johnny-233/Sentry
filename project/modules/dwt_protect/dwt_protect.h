/**
 * @file    dwt_protect.h
 * @brief   DWT 守护模块（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 dwt_protect 迁移，逻辑不变。本项目使用 FreeRTOS（CMSIS-RTOS2），
 *          保留 RTOS 分支，删除裸机 TIM 占位分支。
 */
#pragma once

#include "cmsis_os2.h"

#define DWT_DAEMON_PERIOD_MS  10000   // 更新时间间隔，单位 ms

void DWT_DaemonInit(void);
