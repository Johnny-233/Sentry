/**
 * @file    rgb_led.cpp
 * @brief   RGB LED 模块实现（C → C++）
 * @note    原 C 版逻辑不变，仅去 malloc、去 NULL 检查。
 */
#include "rgb_led.h"

void RGB::init(const Config& config) {
    PWM::Config cfg_r = { .htim = config.htim, .channel = config.ch_r };
    PWM::Config cfg_g = { .htim = config.htim, .channel = config.ch_g };
    PWM::Config cfg_b = { .htim = config.htim, .channel = config.ch_b };

    r_.init(cfg_r);
    g_.init(cfg_g);
    b_.init(cfg_b);
}

void RGB::initDefault() {
    Config config = {
        .htim = &htim5,
        .ch_r = TIM_CHANNEL_3,   // PH12
        .ch_g = TIM_CHANNEL_2,   // PH11
        .ch_b = TIM_CHANNEL_1,   // PH10
    };
    init(config);
}

void RGB::set(uint16_t r, uint16_t g, uint16_t b) {
    r_.setDutyRatio((float)r / 999.0f);
    g_.setDutyRatio((float)g / 999.0f);
    b_.setDutyRatio((float)b / 999.0f);
}
