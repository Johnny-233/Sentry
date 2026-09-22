/**
 * @file    power.h
 * @brief   功率模块（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 power 迁移，逻辑不变。去掉 extern "C"、改用 #pragma once（对齐 crc.h）。
 */
#pragma once

#include "adc.h" // 提供 ADC_HandleTypeDef

// ========== RM 场景专用模块 ==========
// C 板母线电压检测, 硬件: VCC_BAT 经 100k/10k 分压后接 PF10 = ADC3_IN8。
// 分压比 / 参考电压等 RM 场景特有的常量在此写死, 不做通用化。
// 注意: 分压网络源阻抗约 9kΩ, ADC3 通道采样时间建议配 480 cycles。
#define POWER_VBAT_DIVIDER 11.0f // 分压比 R1=100k / R2=10k, 电池电压 = ADC 引脚电压 × 11

void Power_Init(ADC_HandleTypeDef *hadc);
float Power_GetBusVoltage(void);
