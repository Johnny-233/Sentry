/**
 * @file    power.cpp
 * @brief   功率模块实现（C → C++）
 * @note    原 C 版逻辑不变；去 NULL 检查。
 */
#include "power.h"
#include "bsp_adc.h"

static ADC power_adc;

void Power_Init(ADC_HandleTypeDef *hadc)
{
    ADC::Config cfg = {
        .adc_handle = hadc,
        .vref = 3.3f, // ADC 参考电压 VDDA
    };
    power_adc.init(cfg);
}

float Power_GetBusVoltage(void)
{
    return power_adc.getVoltage() * POWER_VBAT_DIVIDER;
}
