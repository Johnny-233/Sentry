/**
 * @file    servo.cpp
 * @brief   舵机实现（C → C++）
 * @note    原 C 版逻辑不变；malloc 实例改为应用层全局对象，死注册表删除（禁堆）。
 */
#include "servo.h"

float Servo::angleToDuty(float angle)
{
    // 限制角度在范围内
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    // 角度 -> 占空比
    float ratio = (angle - 0) / (180 - 0);
    return 5.0 + ratio * (10.0 - 5.0);  // 0.5ms ~ 2.5ms 对应 5% ~ 10% 占空比
}

void Servo::init(const Config& config)
{
    // 注册 PWM 实例
    servo_pwm_.init(config.servo_pwm_config);

    float init_duty = angleToDuty(config.init_angle);
    servo_pwm_.setDutyRatio(init_duty);
}

void Servo::move(float angle)
{
    servo_pwm_.setDutyRatio(angleToDuty(angle));
}
