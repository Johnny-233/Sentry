/**
 * @file    bsp_gpio.h
 * @brief   GPIO（C → C++）
 * @note    从 C 版 bsp_gpio 迁移：struct GPIO_Instance → class GPIO，GPIO_Register → init、
 *          GPIO_ReadPin/WritePin/TogglePin → readPin/writePin/togglePin，逻辑不变，禁堆。
 */
#pragma once

#include "gpio.h"
#include <stdint.h>

#define GPIO_MX_DEVICE_NUM 20

class GPIO {
public:
    /// EXTI 触发模式
    enum class ExtiMode : uint8_t {
        None = 0,       ///< 无
        Rising,         ///< 上升沿
        Falling,        ///< 下降沿
        RisingFalling,  ///< 双边沿
    };

    using Callback = void (*)(GPIO*);   ///< EXTI 回调，参数为 GPIO 实例

    /// 初始化配置
    struct Config {
        GPIO_TypeDef* gpio_x;     ///< GPIO 端口
        uint16_t pin;             ///< 引脚
        GPIO_PinState pin_state;  ///< 初始电平
        ExtiMode exti_mode;       ///< EXTI 模式
    };

    void init(const Config& config);      ///< 替代 GPIO_Register（禁堆）
    GPIO_PinState readPin();              ///< 替代 GPIO_ReadPin
    void writePin(GPIO_PinState state);   ///< 替代 GPIO_WritePin
    void togglePin();                     ///< 替代 GPIO_TogglePin
    void setCallback(Callback callback);  ///< 设置 EXTI 回调

    static void extiCallback(uint16_t pin);  ///< EXTI 分发

private:
    GPIO_TypeDef* gpio_x_ = nullptr;    ///< GPIO 端口
    uint16_t pin_ = 0;                  ///< 引脚
    GPIO_PinState pin_state_;           ///< 初始电平
    ExtiMode exti_mode_;                ///< EXTI 模式
    Callback callback_ = nullptr;       ///< EXTI 回调

    static GPIO* instances_[GPIO_MX_DEVICE_NUM];  ///< 实例注册表
    static uint8_t idx_;                          ///< 已注册数
};
