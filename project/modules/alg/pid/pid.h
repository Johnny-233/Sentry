/**
 * @file    pid.h
 * @brief   通用 PID 控制器（C → C++）
 * @note    从 C 版 pid 迁移：struct PID_Instance → class PID，PID_Init → init、
 *          PID_Update → update 等，逻辑不变，禁堆。
 */
#pragma once

#include <stdint.h>

class PID {
public:
    /// PID 模式
    enum class Mode : uint8_t { Position = 0, Delta = 1 };

    /// 功能开关（位标志，C 风格枚举，值可直接 | 组合）
    enum Feature : uint16_t {
        FeatureNone                    = 0,
        FeatureIntegralLimit           = 1u << 0,  // 积分限幅 (增量式不需要)
        FeatureDerivativeLimit         = 1u << 1,  // 微分限幅
        FeatureOutputLimit             = 1u << 2,  // 输出限幅
        FeatureDeadZone                = 1u << 3,  // 死区处理
        FeatureFilter                  = 1u << 4,  // 输入滤波
        FeatureVariableGain            = 1u << 5,  // 变增益 PID
        FeatureDerivativeOnMeasurement = 1u << 6,  // 微分先行
        FeatureFeedforward             = 1u << 7,  // 前馈控制
        FeatureTrapezoidIntegral       = 1u << 8,  // 梯形积分（位置式）
    };

    /// 初始化配置
    struct Config {
        float kp;                  // 比例系数
        float ki;                  // 积分系数
        float kd;                  // 微分系数
        Mode mode;                 // PID 模式
        uint16_t features;        // 功能开关（位标志，直接 | 组合）
        float integral_limit;      // 积分限幅值
        float derivative_limit;    // 微分限幅值
        float output_min;          // 输出最小值
        float output_max;          // 输出最大值
        float dead_zone;           // 死区范围
        float feedforward_gain;    // 前馈增益
        float filter_alpha;        // 滤波系数 (0~1)，越小滤波越强
        float kp_extra;            // 变增益比例系数
        float kd_extra;            // 变增益微分系数
        uint8_t period;            // 采样周期 ms
    };

    void init(const Config& config);                       ///< 替代 PID_Init（禁堆）
    void reset();                                          ///< 替代 PID_Reset
    void setIntegral(float value);                         ///< 替代 PID_Set_Integral
    void resetIntegral();                                  ///< 替代 PID_Reset_Integral
    void setParameters(float kp, float ki, float kd);      ///< 替代 PID_Set_Parameters
    void setSetpoint(float setpoint);                      ///< 替代 PID_Set_Setpoint
    void setFeedforward(float feedforward);                ///< 替代 PID_Set_Feedforward
    Feature getFeatures(Feature feature);                  ///< 替代 PID_Get_Features
    void setFeatures(Feature feature);                     ///< 替代 PID_Set_Features
    void clearFeatures(Feature feature);                   ///< 替代 PID_Clear_Features
    void backCalcAntiWindup(float limited_output);         ///< 反算抗饱和回灌：外部限幅后回灌真实输出
    void update(float feedback);                           ///< 替代 PID_Update

    // —— 跨模块直接读的状态 ——
    float setpoint_ = 0.0f;       ///< 目标值
    float output_ = 0.0f;         ///< PID 输出
    float integral_ = 0.0f;       ///< 积分项（位置式）
    float last_feedback_ = 0.0f;  ///< 上次反馈值

private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float integral_limit_ = 0.0f;
    float derivative_limit_ = 0.0f;
    float output_min_ = 0.0f;
    float output_max_ = 0.0f;
    float dead_zone_ = 0.0f;
    float feedforward_gain_ = 0.0f;
    float filter_alpha_ = 0.0f;
    float kp_extra_ = 0.0f;
    float kd_extra_ = 0.0f;
    float feedforward_ = 0.0f;
    float last_feedforward_ = 0.0f;
    float last_error_ = 0.0f;
    float last_2_error_ = 0.0f;
    float last_2_feedback_ = 0.0f;
    Mode mode_ = Mode::Position;
    uint16_t features_ = FeatureNone;
    uint8_t period_ = 1;
};
