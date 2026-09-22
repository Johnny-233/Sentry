/**
 * @file    pid.cpp
 * @brief   通用 PID 控制器实现（C → C++）
 * @note    从 C 版 pid 迁移，逻辑不变，禁堆。
 */
#include "pid.h"
#include <math.h>

void PID::init(const Config& config) {
    kp_ = config.kp;
    ki_ = config.ki;
    kd_ = config.kd;
    mode_ = config.mode;
    features_ = config.features;
    period_ = (config.period == 0) ? 1 : config.period;

    integral_limit_ = config.integral_limit;
    derivative_limit_ = config.derivative_limit;
    dead_zone_ = config.dead_zone;
    output_min_ = config.output_min;
    output_max_ = config.output_max;
    feedforward_gain_ = (config.feedforward_gain == 0.0f) ? 1.0f : config.feedforward_gain;

    filter_alpha_ = config.filter_alpha;
    if (filter_alpha_ <= 0.0f)
        filter_alpha_ = 0.3f;
    else if (filter_alpha_ > 1.0f)
        filter_alpha_ = 1.0f;
    kp_extra_ = config.kp_extra;
    kd_extra_ = config.kd_extra;
}

void PID::reset() {
    integral_ = 0.0f;
    last_error_ = 0.0f;
    last_feedback_ = 0.0f;
    last_2_error_ = 0.0f;
    last_2_feedback_ = 0.0f;
    output_ = 0.0f;
}

void PID::setIntegral(float value) {
    integral_ = value;
}

void PID::resetIntegral() {
    setIntegral(0.0f);
}

void PID::setParameters(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setSetpoint(float setpoint) {
    setpoint_ = setpoint;
}

void PID::setFeedforward(float feedforward) {
    feedforward_ = feedforward;
}

PID::Feature PID::getFeatures(Feature feature) {
    return static_cast<Feature>(features_ & feature);
}

void PID::setFeatures(Feature feature) {
    features_ = features_ | feature;
}

void PID::clearFeatures(Feature feature) {
    features_ = features_ & ~feature;
}

void PID::update(float feedback) {
    // 一阶低通滤波输入
    if ((features_ & FeatureFilter) != FeatureNone)
        feedback = filter_alpha_ * feedback + (1.0f - filter_alpha_) * last_feedback_;
    float error = setpoint_ - feedback;

    // 死区处理
    if ((features_ & FeatureDeadZone) != FeatureNone)
        if (error > -dead_zone_ && error < dead_zone_)
            error = 0.0f;

    // 变增益处理（线性增益）
    float kp = kp_;
    float kd = kd_;
    if ((features_ & FeatureVariableGain) != FeatureNone) {
        kp = kp_ + (error > 0.0f ? error : -error) * kp_extra_;
        kd = kd_ + (error > 0.0f ? error : -error) * kd_extra_;
    }

    if (mode_ == Mode::Delta) {
        // 增量式
        integral_ = error * period_;
        // 微分先行
        float derivative;
        if ((features_ & FeatureDerivativeOnMeasurement) != FeatureNone)
            derivative = -(feedback - 2*last_feedback_ + last_2_feedback_) / period_;
        else
            derivative = (error - 2*last_error_ + last_2_error_) / period_;

        // 微分限幅
        if ((features_ & FeatureDerivativeLimit) != FeatureNone) {
            if (derivative > derivative_limit_) derivative = derivative_limit_;
            else if (derivative < -derivative_limit_) derivative = -derivative_limit_;
        }

        // 输出计算
        output_ += kp * (error - last_error_) + ki_ * integral_ + kd * derivative;

        // 前馈处理
        if ((features_ & FeatureFeedforward) != FeatureNone)
            output_ += (feedforward_ - last_feedforward_) * feedforward_gain_;

        last_2_error_ = last_error_;
        last_2_feedback_ = last_feedback_;
        last_feedforward_ = feedforward_;
    } else {
        // 位置式
        // 抗积分饱和
        if (!((features_ & FeatureOutputLimit) != FeatureNone &&
              ((output_ >= output_max_ && error > 0.0f) ||
               (output_ <= output_min_ && error < 0.0f)))) {
            // 梯形积分：用前后误差均值近似面积，否则矩形
            if ((features_ & FeatureTrapezoidIntegral) != FeatureNone)
                integral_ += (error + last_error_) * period_ * 0.5f;
            else
                integral_ += error * period_;
        }
        // 积分限幅
        if ((features_ & FeatureIntegralLimit) != FeatureNone) {
            if (integral_ > integral_limit_) integral_ = integral_limit_;
            else if (integral_ < -integral_limit_) integral_ = -integral_limit_;
        }

        // 微分先行
        float derivative;
        if ((features_ & FeatureDerivativeOnMeasurement) != FeatureNone)
            derivative = -(feedback - last_feedback_) / period_;
        else
            derivative = (error - last_error_) / period_;

        // 微分限幅
        if ((features_ & FeatureDerivativeLimit) != FeatureNone) {
            if (derivative > derivative_limit_) derivative = derivative_limit_;
            else if (derivative < -derivative_limit_) derivative = -derivative_limit_;
        }

        // 输出计算
        output_ = kp * error + ki_ * integral_ + kd * derivative;

        // 前馈处理
        if ((features_ & FeatureFeedforward) != FeatureNone)
            output_ += feedforward_ * feedforward_gain_;
    }

    // 输出限幅
    if ((features_ & FeatureOutputLimit) != FeatureNone) {
        if (output_ > output_max_) output_ = output_max_;
        else if (output_ < output_min_) output_ = output_min_;
    }

    last_error_ = error;
    last_feedback_ = feedback;
}

void PID::backCalcAntiWindup(float limited_output) {
    // 反算抗饱和回灌：外部限幅后回灌真实输出，仅当新输出绝对值更小（被限得更狠）才生效
    if (fabsf(limited_output) >= fabsf(output_))
        return;

    if (mode_ == Mode::Position) {
        // 位置式：砍积分去对齐输出（输出差量全部由积分项吸收）
        if (ki_ != 0.0f)
            integral_ -= (output_ - limited_output) / ki_;
    }
    // 增量式：直接砍增量值（输出回退到被限后的值）
    output_ = limited_output;
}
