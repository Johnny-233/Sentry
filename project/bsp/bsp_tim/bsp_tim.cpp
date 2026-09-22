/**
 * @file    bsp_tim.cpp
 * @brief   TIM 时基中断实现
 */
#include "bsp_tim.h"

TIM* TIM::instances_[TIM_MX_DEVICE_NUM] = {nullptr};
uint8_t TIM::idx_ = 0;

void TIM::init(const Config& config) {
    if (idx_ >= TIM_MX_DEVICE_NUM)
        return;

    htim_ = config.htim;
    instances_[idx_++] = this;

    startIT();
}

void TIM::startIT() {
    HAL_TIM_Base_Start_IT(htim_);
}

void TIM::stopIT() {
    HAL_TIM_Base_Stop_IT(htim_);
}

void TIM::setCallback(Callback callback, void* device) {
    callback_ = callback;
    device_ = device;
}

void TIM::periodElapsedCallback(TIM_HandleTypeDef* htim) {
    for (uint8_t i = 0; i < idx_; i++) {
        TIM* tim = instances_[i];
        if (tim->callback_ != nullptr && tim->htim_ == htim)
            tim->callback_(tim->device_);
    }
}

extern "C" void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    TIM::periodElapsedCallback(htim);
}
