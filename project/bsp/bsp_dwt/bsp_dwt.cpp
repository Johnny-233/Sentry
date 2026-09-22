/**
 * @file    bsp_dwt.cpp
 * @brief   DWT 时钟模块实现（C → C++）
 * @note    使用 DWT 寄存器实现高精度时间测量与延时，适用于 STM32 系列。
 */
#include "bsp_dwt.h"
#include "main.h"

// DWT 时刻（可放进全局给外部调用）
static uint64_t CYCCNT64;
// 提前计算好的换算系数
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
// DWT 计数器溢出次数
static uint32_t CYCCNT_RountCount;
// DWT 是否初始化
uint8_t DWT_Init_flag = 0;

static void DWT_CNT_Update(void) {
    static uint32_t CYCCNT_LAST = 0;
    // 互斥锁
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker) {
        bit_locker = 1;

        // 更新 DWT 计数器溢出次数，并更新 DWT 时刻
        uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;
        CYCCNT_LAST = cnt_now;

        bit_locker = 0;
    }
}

void DWT_SysTimeUpdate(void) {
    // 更新 DWT 时刻
    DWT_CNT_Update();
    CYCCNT64 = (uint64_t)CYCCNT_RountCount << 32 | DWT->CYCCNT;
}

void DWT_Init(void) {
    // 计算 CPU 频率，并计算换算系数
    CPU_FREQ_Hz = HAL_RCC_GetSysClockFreq();
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;

    // 寄存器初始化，使能并清零 DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = (uint32_t)0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // 初始化 DWT 时刻和溢出次数
    CYCCNT_RountCount = 0;
    DWT_SysTimeUpdate();
    DWT_Init_flag = 1;
}

float DWT_GetDeltaT(uint32_t* cnt_last) {
    uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    // 利用回绕机制计算时间差
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

double DWT_GetDeltaT64(uint32_t* cnt_last) {
    uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    // 利用回绕机制计算时间差
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

float DWT_GetTimeline_s(void) {
    DWT_SysTimeUpdate();
    return CYCCNT64 / (float)CPU_FREQ_Hz;
}

float DWT_GetTimeline_ms(void) {
    DWT_SysTimeUpdate();
    return CYCCNT64 / (float)CPU_FREQ_Hz_ms;
}

uint64_t DWT_GetTimeline_us(void) {
    DWT_SysTimeUpdate();
    return CYCCNT64 / (uint64_t)CPU_FREQ_Hz_us;
}

void DWT_Delay_Tick(uint64_t delay) {
    uint64_t tickstart = DWT->CYCCNT;

    // 等到延时结束
    while ((DWT->CYCCNT - tickstart) < delay);
}

void DWT_Delay_us(float delay) {
    DWT_Delay_Tick(delay * CPU_FREQ_Hz_us);
}

void DWT_Delay_ms(float delay) {
    DWT_Delay_Tick(delay * CPU_FREQ_Hz_ms);
}
