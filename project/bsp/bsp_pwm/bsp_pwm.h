/**
 * @file    bsp_pwm.h
 * @brief   PWM 输出（C → C++）
 * @note    从 C 版 bsp_pwm 迁移：struct PWM_Instance → class PWM，Register → init，
 *          逻辑不变，禁堆（原 malloc 实例改为栈/全局对象）。
 */
#pragma once

#include "tim.h"
#include <stdint.h>

class PWM {
public:
    /// 初始化配置
    struct Config {
        TIM_HandleTypeDef* htim;    ///< TIM 句柄
        uint32_t channel;           ///< 通道
    };

    void init(const Config& config);     ///< 替代 PWM_Register（禁堆）
    void start();                        ///< 启动 PWM
    void stop();                         ///< 停止 PWM
    void setDutyRatio(float dutyratio);  ///< 设置占空比 [0,1]
    void setPeriod(float period);        ///< 设置周期 [s]

private:
    TIM_HandleTypeDef* htim_;   ///< TIM 句柄
    uint32_t channel_;          ///< 通道

    static uint32_t cpu_freq_hz_;   ///< CPU 频率，setPeriod 中需要
};
