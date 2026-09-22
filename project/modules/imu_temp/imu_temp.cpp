/**
 * @file    imu_temp.cpp
 * @brief   IMU 温控模块实现（C → C++）
 */
#include "imu_temp.h"

void IMUTemp::init(const Config& config) {
    target_temp_ = IMU_TEMP_TARGET;

    PID::Config pid_cfg = {
        .kp = 2500.0f,
        .ki = 20.0f,
        .kd = 0.0f,
        .mode = PID::Mode::Position,
        .features = PID::FeatureIntegralLimit | PID::FeatureOutputLimit | PID::FeatureTrapezoidIntegral,
        .integral_limit = 30.0f,
        .output_min = 0.0f,
        .output_max = 2000.0f,
    };
    pid_.init(pid_cfg);
    pid_.setSetpoint(IMU_TEMP_TARGET);

    PWM::Config pwm_cfg = {
        .htim = config.htim,
        .channel = config.channel,
    };
    pwm_.init(pwm_cfg);
}

void IMUTemp::update(float temperature) {
    if (temperature > IMU_TEMP_SAFETY_MAX || temperature < 0.0f) {
        pwm_.setDutyRatio(0.0f);
        pid_.resetIntegral();
        return;
    }

    pid_.update(temperature);
    pwm_.setDutyRatio(pid_.output_ / 4999.0f);
}

bool IMUTemp::isReady() {
    float err = pid_.setpoint_ - pid_.last_feedback_;
    return (err > -IMU_TEMP_TOLERANCE && err < IMU_TEMP_TOLERANCE);
}

void IMUTemp::safetyOff() {
    pwm_.setDutyRatio(0.0f);
}
