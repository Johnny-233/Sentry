/**
 * @file    imu_temp.h
 * @brief   IMU 温控模块（C → C++）
 * @note    从 C 版 imu_temp 迁移：struct IMU_Temp_Instance → class IMUTemp，
 *          IMU_Temp_Init → init、IMU_Temp_Update → update、IsReady → isReady、
 *          SafetyOff → safetyOff，逻辑不变，禁堆。PID + PWM 内嵌为成员。
 */
#pragma once

#include "bsp_pwm.h"
#include "pid.h"

#define IMU_TEMP_TARGET     40.0f
#define IMU_TEMP_TOLERANCE  1.0f
#define IMU_TEMP_SAFETY_MAX 50.0f

class IMUTemp {
public:
    /// 初始化配置
    struct Config {
        TIM_HandleTypeDef* htim;    ///< 加热用 PWM 定时器
        uint32_t channel;           ///< 加热用 PWM 通道
    };

    void init(const Config& config);  ///< 替代 IMU_Temp_Init（禁堆）
    void update(float temperature);   ///< 替代 IMU_Temp_Update
    bool isReady();                   ///< 替代 IMU_Temp_IsReady
    void safetyOff();                 ///< 替代 IMU_Temp_SafetyOff

private:
    PID pid_;                              ///< 温度环 PID
    PWM pwm_;                              ///< 加热 PWM
    float target_temp_ = IMU_TEMP_TARGET;  ///< 目标温度
};
