/**
 * @file    bsp_adc.cpp
 * @brief   ADC 实现
 */
#include "bsp_adc.h"

#define ADC_TIMEOUT_MS 10  // 手动读取模式下的转换超时,单位 ms

// 根据 Init.Resolution 的位宽编码推导满量程原始值
// (ADC_RESOLUTION_12B=0, 10B/8B/6B 是 CR1 的位编码而非位数,故用 switch 而非算术)
uint16_t ADC::fullScale(uint32_t resolution) {
    switch (resolution) {
        case ADC_RESOLUTION_12B: return 4095;
        case ADC_RESOLUTION_10B: return 1023;
        case ADC_RESOLUTION_8B:  return 255;
        case ADC_RESOLUTION_6B:  return 63;
        default:                 return 4095;
    }
}

void ADC::init(const Config& config) {
    if (config.adc_handle == nullptr)
        return;

    adc_handle_ = config.adc_handle;
    vref_ = config.vref;
    full_scale_ = fullScale(config.adc_handle->Init.Resolution);
}

uint16_t ADC::getRaw() {
    if (adc_handle_ == nullptr)
        return 0;

    HAL_ADC_Start(adc_handle_);
    HAL_ADC_PollForConversion(adc_handle_, ADC_TIMEOUT_MS);
    uint16_t raw = HAL_ADC_GetValue(adc_handle_);
    HAL_ADC_Stop(adc_handle_);

    return raw;
}

float ADC::getVoltage() {
    if (adc_handle_ == nullptr)
        return 0.0f;

    return (float)getRaw() * vref_ / (float)full_scale_;
}
