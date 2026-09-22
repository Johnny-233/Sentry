/**
 * @file    bsp_adc.h
 * @brief   ADC（C → C++）
 * @note    从 C 版 bsp_adc 迁移：struct ADC_Instance → class ADC，ADCRegister → init、
 *          ADCGetRaw/ADCGetVoltage → getRaw/getVoltage，逻辑不变，禁堆。
 */
#pragma once

#include <stdint.h>
#include "adc.h"

#define ADC_MX_DEVICE_NUM 8  // 最大支持的ADC实例数量

#undef ADC  // stm32f407xx.h 遗留 `#define ADC ADC123_COMMON`，与类名冲突

class ADC {
public:
    /// 初始化配置
    struct Config {
        ADC_HandleTypeDef* adc_handle;  ///< ADC 句柄
        float vref;                     ///< 参考电压 V
    };

    void init(const Config& config);  ///< 替代 ADCRegister（禁堆）
    uint16_t getRaw();                ///< 替代 ADCGetRaw
    float getVoltage();               ///< 替代 ADCGetVoltage

private:
    uint16_t fullScale(uint32_t resolution);  ///< 替代 ADC_FullScale，按位宽推导满量程

    ADC_HandleTypeDef* adc_handle_ = nullptr;  ///< ADC 句柄
    float vref_ = 0.0f;                        ///< 参考电压 V
    uint16_t full_scale_ = 0;                  ///< 满量程原始值
};
