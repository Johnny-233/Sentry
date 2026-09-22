/**
 * @file    rgb_led.h
 * @brief   RGB LED 模块（C → C++）
 * @note    从 C 版 rgb_led 迁移：struct RGB_Instance → class RGB，
 *          RGB_Register → init、RGB_Init → initDefault、RGB_Set → set，逻辑不变，禁堆。
 */
#pragma once

#include "bsp_pwm.h"

class RGB {
public:
    /// 初始化配置
    struct Config {
        TIM_HandleTypeDef* htim;   ///< TIM 句柄
        uint32_t ch_r;             ///< 红通道
        uint32_t ch_g;             ///< 绿通道
        uint32_t ch_b;             ///< 蓝通道
    };

    void init(const Config& config);   ///< 替代 RGB_Register（禁堆）
    void initDefault();                ///< 替代 RGB_Init（硬编码 htim5 + 三通道）
    void set(uint16_t r, uint16_t g, uint16_t b);  ///< 替代 RGB_Set

private:
    PWM r_;   ///< 红通道 PWM
    PWM g_;   ///< 绿通道 PWM
    PWM b_;   ///< 蓝通道 PWM
};
