/**
 * @file    super_cap.h
 * @brief   超级电容（C → C++）
 * @note    从 C 版 Modules/super_cap 迁移：SuperCapInstance → class SuperCap，
 *          SuperCapInit → init、SuperCapSend → send，收包解析与发送逻辑不变，禁堆、无 extern "C"。
 *          旧实现的单例静态指针 super_cap_instance 删除：回调通过 CAN::setCallback 的 device 参数拿到对象。
 *          接收 id / 发送 id 由 Config 配置（旧 chassis.c：hcan1、rx 0x311、tx 0x310）。
 */
#pragma once

#include <stdint.h>
#include "bsp_can.h"

class SuperCap {
public:
    /// 初始化配置
    struct Config {
        CAN_HandleTypeDef* can_handle; // CAN 句柄
        uint32_t rx_id;                // 接收 id（超电反馈，旧 chassis.c 用 0x311）
        uint32_t tx_id;                // 发送 id（功率限制指令，旧 chassis.c 用 0x310）
    };

    /// 超级电容反馈信息（收包解析结果）
    struct Msg {
        uint16_t vol;     // 电压
        uint16_t current; // 电流
        uint16_t power;   // 功率
    };

    void init(const Config& config);                    // 替代 SuperCapInit
    void send(uint8_t* data, uint16_t len);             // 替代 SuperCapSend

    // —— 跨模块读取的状态（公开直接读）——
    Msg cap_msg_ = {}; // 超级电容信息

private:
    static void rxCallback(void* device); // 替代 SuperCapRxCallback

    CAN can_; // CAN 实例
};
