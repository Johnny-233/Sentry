/**
 * @file    servo.h
 * @brief   舵机（C → C++）
 * @note    从 C 版 servo 迁移：struct Servo_Instance → class Servo，
 *          ServoRegister → init、ServoMove → move，逻辑不变，禁堆。
 *          原 C 的死注册表与未使用的 min_duty/max_duty 宏一并删除。
 */
#pragma once

#include "bsp_pwm.h"

class Servo {
public:
    /// 初始化配置
    struct Config {
        PWM::Config servo_pwm_config;   // PWM 配置
        float init_angle;               // 初始角度
    };

    void init(const Config& config);    // 替代 ServoRegister
    void move(float angle);             // 替代 ServoMove

private:
    static float angleToDuty(float angle);  // 替代 AngleToDuty

    PWM servo_pwm_;                     // PWM 实例
};
