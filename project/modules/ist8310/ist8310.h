/**
 * @file    ist8310.h
 * @brief   IST8310 磁力计（C → C++）
 * @note    从 C 版 ist8310 迁移：struct IST8310_Instance → class IST8310，
 *          IST8310_Register → init、IST8310_Acquire → acquire，逻辑不变，禁堆。
 *          WHO_AM_I 校验失败以 valid() 软错误状态体现（对齐 BMI088）。
 */
#pragma once

#include "bsp_i2c.h"
#include "bsp_gpio.h"
#include "ist8310_reg.h"
#include <stdint.h>

class IST8310 {
public:
    /// 磁力计数据
    struct Data {
        float mag[3];   // µT (微特斯拉)
    };

    /// 初始化配置
    struct Config {
        HardI2C::Config i2c_config;   ///< 硬件 I2C 配置
        GPIO::Config rst_config;      ///< 复位引脚配置
    };

    void init(const Config& config);          ///< 替代 IST8310_Register（禁堆）
    bool valid() const { return valid_; }     ///< WHO_AM_I 校验是否通过
    uint8_t acquire(Data& data);              ///< 替代 IST8310_Acquire（0 成功 / 1 失败）

private:
    void reset();                              ///< 替代 IST8310_Reset
    uint8_t readReg(uint8_t reg);              ///< 替代 IST8310_ReadReg
    void writeReg(uint8_t reg, uint8_t data);  ///< 替代 IST8310_WriteReg

    HardI2C i2c_;        ///< 硬件 I2C
    GPIO rst_;           ///< 复位引脚
    bool valid_ = false; ///< WHO_AM_I 校验通过标志
};
