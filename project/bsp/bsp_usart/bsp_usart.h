/**
 * @file    bsp_usart.h
 * @brief   USART 串口（C → C++）
 * @note    从 C 版 bsp_usart 迁移：struct USART_Instance → class USART，
 *          USART_Register → init、USARTSend → send，逻辑不变，禁堆。
 */
#pragma once

#include "usart.h"
#include <stdint.h>

#define DEVICE_USART_CNT 5     // 最大支持串口实例
#define USART_RXBUFF_LIMIT 256 // 默认256就好

#if USART_RXBUFF_LIMIT > 256 || USART_RXBUFF_LIMIT < 1
#error "USART_RXBUFF_LIMIT must be between 1 and 256"
#endif

class USART {
public:
    using Callback = void (*)(void*, uint8_t);  // 参数：用户数据指针 + 接收数据长度

    /// 初始化配置
    struct Config {
        UART_HandleTypeDef* usart_handle;   ///< HAL USART 句柄
    };

    void init(const Config& config);    ///< 替代 USART_Register（禁堆）
    void send(uint8_t* send_buf, uint16_t send_size);  ///< 替代 USARTSend
    void setCallback(Callback callback, void* device); ///< 设置接收回调

    /// 真正重启接收: 先 AbortReceive 清掉 HAL 的 BUSY/锁死状态, 再重开 DMA+IDLE。
    /// (只调 serviceInit() 在 HAL 认为还 BUSY 时会直接返回 HAL_BUSY, 等于没救)
    void restart();

    // —— 跨模块直接读写的状态（对齐 C 版字段）——
    uint8_t recv_buff_[USART_RXBUFF_LIMIT]; ///< 接收缓冲区

    static void rxEventCallback(UART_HandleTypeDef* huart, uint16_t size);  ///< 接收完成分发
    static void errorCallback(UART_HandleTypeDef* huart);                   ///< 错误分发

private:
    void serviceInit();  ///< 替代 USART_ServiceInit（启动接收中断）

    UART_HandleTypeDef* usart_handle_ = nullptr;  ///< HAL USART 句柄
    Callback callback_ = nullptr;                 ///< 接收完成回调
    void* device_ = nullptr;                      ///< 回调 device

    static USART* instances_[DEVICE_USART_CNT];   ///< 实例注册表
    static uint8_t idx_;                          ///< 已注册数
};
