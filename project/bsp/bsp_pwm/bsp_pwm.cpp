/**
 * @file    bsp_pwm.cpp
 * @brief   PWM 输出实现
 */
#include "bsp_pwm.h"

uint32_t PWM::cpu_freq_hz_ = 0;

void PWM::init(const Config& config) {
    htim_ = config.htim;
    channel_ = config.channel;

    if (cpu_freq_hz_ == 0)   // 对应原版 !idx：仅首次计算
        cpu_freq_hz_ = HAL_RCC_GetSysClockFreq();

    // 启动 PWM
    HAL_TIM_PWM_Start(htim_, channel_);
}

void PWM::start() {
    HAL_TIM_PWM_Start(htim_, channel_);
}

void PWM::stop() {
    HAL_TIM_PWM_Stop(htim_, channel_);
}

void PWM::setDutyRatio(float dutyratio) {
    if (dutyratio < 0.0f) dutyratio = 0.0f;
    if (dutyratio > 1.0f) dutyratio = 1.0f;
    __HAL_TIM_SetCompare(htim_, channel_, dutyratio * (htim_->Instance->ARR));
}

void PWM::setPeriod(float period) {
    uint32_t new_arr = period * cpu_freq_hz_ / (htim_->Init.Prescaler + 1);
    uint32_t compare = (uint16_t)__HAL_TIM_GetCompare(htim_, channel_) * new_arr / __HAL_TIM_GetAutoreload(htim_);

    __HAL_TIM_SetAutoreload(htim_, new_arr);
    __HAL_TIM_SetCompare(htim_, channel_, compare);
}
