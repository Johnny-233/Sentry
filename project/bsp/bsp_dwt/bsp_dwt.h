/**
 * @file    bsp_dwt.h
 * @brief   DWT 时钟模块（C → C++：无实例结构体，自由函数置于全局命名空间）
 * @note    原 C 版逻辑不变；去掉 extern "C"、改用 #pragma once。
 */
#pragma once
#include <stdint.h>

extern uint8_t DWT_Init_flag;   ///< DWT 是否已初始化

// 函数占用时间检测宏
#define TIME_ELAPSE(dt, code)                    \
    do                                           \
    {                                            \
        float tstart = DWT_GetTimeline_s();      \
        code;                                    \
        dt = DWT_GetTimeline_s() - tstart;       \
    } while (0)

void DWT_Init(void);                              // DWT 初始化
float DWT_GetDeltaT(uint32_t* cnt_last);          // 获取时间差 [s]
double DWT_GetDeltaT64(uint32_t* cnt_last);       // 精确时间差 [s]
float DWT_GetTimeline_s(void);                    // 当前秒
float DWT_GetTimeline_ms(void);                   // 当前毫秒
uint64_t DWT_GetTimeline_us(void);                // 当前微秒
void DWT_Delay_Tick(uint64_t delay);              // 忙等待 tick
void DWT_Delay_us(float delay);                   // 忙等待 us
void DWT_Delay_ms(float delay);                   // 忙等待 ms
void DWT_SysTimeUpdate(void);                     // DWT 更新
