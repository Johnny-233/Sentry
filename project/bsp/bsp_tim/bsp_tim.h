/**
 * @file    bsp_tim.h
 * @brief   TIM 时基中断（C → C++）
 * @note    从 C 版 bsp_tim 迁移：struct TIM_Instance → class TIM，Register → init，
 *          逻辑不变，禁堆。USER_TIM_PeriodElapsedCallback 被 main.c（C）调用，
 *          故保留 extern "C"（边界②），类声明用 #ifdef __cplusplus 隔离。
 */
#pragma once

#include "tim.h"
#include <stdint.h>

#define TIM_MX_DEVICE_NUM 25

#ifdef __cplusplus
class TIM {
public:
    using Callback = void (*)(void*);

    /// 初始化配置
    struct Config {
        TIM_HandleTypeDef* htim;    ///< TIM 句柄
    };

    void init(const Config& config);    ///< 替代 TIM_Register（禁堆）
    void startIT();                     ///< 启动中断
    void stopIT();                      ///< 停止中断
    void setCallback(Callback callback, void* device);  ///< 设置周期中断回调
    static void periodElapsedCallback(TIM_HandleTypeDef* htim);  ///< 中断分发

private:
    TIM_HandleTypeDef* htim_ = nullptr;   ///< TIM 句柄
    Callback callback_ = nullptr;         ///< 周期中断回调
    void* device_ = nullptr;              ///< 回调 device

    static TIM* instances_[TIM_MX_DEVICE_NUM];  ///< 实例注册表
    static uint8_t idx_;                        ///< 已注册数
};
#endif  // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif
// 由 main.c（C 文件）的 HAL_TIM_PeriodElapsedCallback 调用
void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
#ifdef __cplusplus
}
#endif
