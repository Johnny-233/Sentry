/**
 * @file    serial.cpp
 * @brief   串口管理模块实现（C → C++）
 * @note    原 C 版单块（recv_size=1）逻辑不变，删除多块缓冲与 malloc。
 */
#include "serial.h"

#include <string.h>

void Serial::copyToRecvBuf(uint16_t pos) {
    uint16_t len;

    if (pos > last_pos_parse_) {
        // 连续：从 last_pos 搬到 pos
        len = pos - last_pos_parse_;
        memcpy(recv_buf_, usart_.recv_buff_ + last_pos_parse_, len);
    } else {
        // 回绕：两段
        uint16_t len1 = SERIAL_RXBUFF_LIMIT - last_pos_parse_;
        uint16_t len2 = pos;
        len = len1 + len2;
        memcpy(recv_buf_, usart_.recv_buff_ + last_pos_parse_, len1);
        memcpy(recv_buf_ + len1, usart_.recv_buff_, len2);
    }

    if (rx_callback_ != nullptr)
        rx_callback_(len);

    last_pos_parse_ = pos;
}

void Serial::fullTimeout(void* device) {
    Serial* instance = static_cast<Serial*>(device);
    // 单块模式下 parse_idx 恒为 0
    instance->copyToRecvBuf(0);
}

void Serial::usartRxCallback(void* device, uint8_t recv_pos) {
    Serial* instance = static_cast<Serial*>(device);

    // 单块模式下 parse_buf 直接指向 usart_.recv_buff_，无需额外搬运

    if (recv_pos % SERIAL_RXBUFF_LIMIT == 0) {
        // 全满中断：喂狗（单块模式下 parse_idx 恒为 0，切换块为空操作）
        instance->full_daemon_.reset();
    } else {
        // 空闲中断：搬运到 recv_buf，关闭全满看门狗
        instance->full_daemon_.online_ = 0;
        instance->copyToRecvBuf(recv_pos);
    }
}

void Serial::init(const Config& config) {
    // 先设回调再启动 DMA（对齐 C 版：回调在注册配置里先设好）
    usart_.setCallback(usartRxCallback, this);
    usart_.init({ .usart_handle = config.usart_handle });
    rx_callback_ = config.rx_callback;

    // 注册缓冲区满看门狗
    Daemon::Config full_daemon_config = {
        .tim_config = { .htim = config.htim },
        .cycle = 2,
        .daemon_callback = fullTimeout,
        .device = this,
    };
    full_daemon_.init(full_daemon_config);
}

void Serial::restart() {
    last_pos_parse_ = 0;
    usart_.restart();
    full_daemon_.reset();
}

void Serial::send(uint8_t* send_buf, uint16_t send_size) {
    usart_.send(send_buf, send_size);
}
