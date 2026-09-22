/**
 * @file    serial.h
 * @brief   串口管理模块（C → C++）
 * @note    从 C 版 serial 迁移：struct Serial_Instance → class Serial，
 *          Serial_Register → init、Serial_Send → send，逻辑不变，禁堆。
 *          本项目所有调用方 recv_size=1，多块缓冲（需 malloc）已删除，仅保留单块零拷贝模式。
 */
#pragma once

#include "bsp_usart.h"
#include "daemon.h"
#include <stdint.h>

#define SERIAL_RXBUFF_LIMIT 256

#if SERIAL_RXBUFF_LIMIT < 1 || SERIAL_RXBUFF_LIMIT > 256
#error "SERIAL_RXBUFF_LIMIT: invalid value, must be between 1 and 256"
#endif

#if SERIAL_RXBUFF_LIMIT != 256
#warning "SERIAL_RXBUFF_LIMIT is not 256, recommend using 256 as single block buffer size"
#endif

class Serial {
public:
    /// 初始化配置
    struct Config {
        UART_HandleTypeDef* usart_handle;    ///< HAL USART 句柄
        TIM_HandleTypeDef* htim;             ///< 缓冲区满看门狗时基（1ms）
        void (*rx_callback)(uint16_t len);   ///< 数据接收完成回调
    };

    void init(const Config& config);                       ///< 替代 Serial_Register（禁堆）
    void send(uint8_t* send_buf, uint16_t send_size);      ///< 替代 Serial_Send

    /// 重启底层接收(掉线时用): 等价于旧 C 版的 USARTServiceInit 兜底
    void restart();

    // —— 跨模块读取（application 在 rx_callback 后读取）——
    uint8_t recv_buf_[SERIAL_RXBUFF_LIMIT];  ///< 接收数据缓冲

private:
    static void usartRxCallback(void* device, uint8_t recv_pos);  ///< 替代 Serial_USART_RX_Callback
    static void fullTimeout(void* device);                        ///< 替代 Serial_FullTimeout
    void copyToRecvBuf(uint16_t pos);                             ///< 替代 Serial_CopyToRecvBuf

    USART usart_;                         ///< 底层 USART（parse_buf 即 usart_.recv_buff_，零拷贝）
    Daemon full_daemon_;                  ///< 缓冲区满看门狗
    uint16_t last_pos_parse_ = 0;         ///< parse_buf 内绝对位置
    void (*rx_callback_)(uint16_t) = nullptr;  ///< 数据接收完成回调
};
