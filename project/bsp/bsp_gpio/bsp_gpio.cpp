/**
 * @file    bsp_gpio.cpp
 * @brief   GPIO 实现
 */
#include "bsp_gpio.h"

GPIO* GPIO::instances_[GPIO_MX_DEVICE_NUM] = {nullptr};
uint8_t GPIO::idx_ = 0;

void GPIO::init(const Config& config) {
    if (idx_ >= GPIO_MX_DEVICE_NUM)
        return;

    gpio_x_ = config.gpio_x;
    pin_ = config.pin;
    pin_state_ = config.pin_state;
    exti_mode_ = config.exti_mode;
    instances_[idx_++] = this;
}

GPIO_PinState GPIO::readPin() {
    return HAL_GPIO_ReadPin(gpio_x_, pin_);
}

void GPIO::writePin(GPIO_PinState state) {
    HAL_GPIO_WritePin(gpio_x_, pin_, state);
}

void GPIO::togglePin() {
    HAL_GPIO_TogglePin(gpio_x_, pin_);
}

void GPIO::setCallback(Callback callback) {
    callback_ = callback;
}

void GPIO::extiCallback(uint16_t pin) {
    for (size_t i = 0; i < idx_; i++) {
        GPIO* gpio = instances_[i];
        if (gpio->pin_ == pin && gpio->callback_ != nullptr) {
            gpio->callback_(gpio);
            return;
        }
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    GPIO::extiCallback(GPIO_Pin);
}
