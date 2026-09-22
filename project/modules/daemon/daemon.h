/**
 * @file    daemon.h
 * @brief   守护进程 / 设备离线看门狗（C → C++）
 * @note    从 C 版 daemon 迁移：struct Daemon_Instance → class Daemon，
 *          Daemon_Register → init、Daemon_Reset → reset，逻辑不变，禁堆。
 *          本项目固定 TIM_DAEMON_SUPPORT=1，故删除 RTOS 分支与死注册表。
 */
#pragma once

#include "bsp_tim.h"
#include <stdint.h>

class Daemon {
public:
    /// 初始化配置
    struct Config {
        TIM::Config tim_config;         ///< 时基定时器（建议 1ms 周期）
        uint32_t cycle;                 ///< 超时周期数（单位与定时器周期一致）
        void (*daemon_callback)(void*); ///< 离线回调
        void* device;                   ///< 回调 device
    };

    /// 注册/初始化（替代 Daemon_Register，禁堆）
    void init(const Config& config);

    /// 喂狗（替代 Daemon_Reset）
    void reset();

    // —— 跨模块读取/写入 ——
    uint8_t online_ = 0;   ///< 1=在线，0=离线

private:
    static void timerCallback(void* arg);   ///< 替代 Timer_Callback

    TIM tim_;                                ///< 时基定时器
    uint32_t cycle_ = 0;                     ///< 超时周期数
    uint32_t count_ = 0;                     ///< 剩余周期计数
    void (*daemon_callback_)(void*) = nullptr; ///< 离线回调
    void* device_ = nullptr;                 ///< 回调 device
};
