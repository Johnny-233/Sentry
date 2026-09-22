/**
 * @file    remote_config.h
 * @brief   遥控器机型编译期选型与期望串口参数
 * @note    板上只接一种接收端：只定义一个 REMOTE_DEVICE_*。
 *          换机型后请同步改 CubeMX 对应 UART 的 Baud/WordLength/Parity。
 */
#pragma once

#include "usart.h"
#include <stdint.h>

/* ========== 机型选择（互斥）==========
 * 当前用**新遥控 VT13/VT03（图传链路，带键鼠）**：huart3 @ 921600 8N1。
 * 旧的 DBUS/Dt7 逻辑仍然完整保留（remote.h 的 DT7 分支 + dt7 驱动都在，robot_cmd.cpp 里
 * 的 DT7 分支也还在），只是暂时用不上。要切回旧遥控：把下面两行对调注释，并把
 * Src/usart.c 里 huart3 改回 100000 / 9B / EVEN —— Remote::init() 会校验串口参数，
 * 不匹配会直接拒绝初始化（日志 "UART params mismatch"）。 */
#define REMOTE_DEVICE_VT13
// #define REMOTE_DEVICE_DT7
// #define REMOTE_DEVICE_HT10A

#if (defined(REMOTE_DEVICE_VT13) + defined(REMOTE_DEVICE_DT7) + defined(REMOTE_DEVICE_HT10A)) != 1
#error "remote_config.h: define exactly one of REMOTE_DEVICE_VT13 / REMOTE_DEVICE_DT7 / REMOTE_DEVICE_HT10A"
#endif

/* ========== 期望串口参数（须与 CubeMX 一致）========== */
#if defined(REMOTE_DEVICE_VT13)
#define REMOTE_UART_BAUD        921600u
#define REMOTE_UART_WORDLENGTH  UART_WORDLENGTH_8B
#define REMOTE_UART_PARITY      UART_PARITY_NONE
#elif defined(REMOTE_DEVICE_DT7)
#define REMOTE_UART_BAUD        100000u
#define REMOTE_UART_WORDLENGTH  UART_WORDLENGTH_9B
#define REMOTE_UART_PARITY      UART_PARITY_EVEN
#elif defined(REMOTE_DEVICE_HT10A)
#define REMOTE_UART_BAUD        100000u
#define REMOTE_UART_WORDLENGTH  UART_WORDLENGTH_9B
#define REMOTE_UART_PARITY      UART_PARITY_EVEN
#endif

static_assert(REMOTE_UART_BAUD > 0u, "REMOTE_UART_BAUD must be positive");
