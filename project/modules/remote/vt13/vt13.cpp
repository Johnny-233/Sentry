/**
 * @file    vt13.cpp
 * @brief   VT13/VT03 图传遥控协议驱动实现
 */
#include "vt13.h"

#if defined(REMOTE_DEVICE_VT13)

#include "daemon.h"
#include "serial.h"
#include "crc_rm.h"
#include <string.h>

namespace remote_vt13 {

static Serial serial_;
static Daemon daemon_;
static remote_frame_t frame_;

uint8_t data_flag;
const remote_frame_t* const data = &frame_;

static void serialCallback(uint16_t len)
{
    if (len != REMOTE_VT13_FRAME_LEN) return;

    remote_frame_t serial_frame;
    memcpy(&serial_frame, serial_.recv_buf_, sizeof(remote_frame_t));

    if (serial_frame.sof_1 != 0xA9 || serial_frame.sof_2 != 0x53) return;
    if (!CRC16_Verify(serial_.recv_buf_, REMOTE_VT13_FRAME_LEN)) return;

    memcpy(&frame_, serial_.recv_buf_, REMOTE_VT13_FRAME_LEN);

    data_flag = 1;
    daemon_.reset();
}

static void lostCallback(void* device)
{
    (void)device;
    /* 遥控掉线: 旧 C 固件在这里调 USARTServiceInit() 重启串口服务 ——
       因为 HAL 开 DMA 接收后有概率 __HAL_LOCK 死锁, 之后再也进不了接收中断,
       表现为"帧停在最后一帧、遥控彻底失灵"。这里做同样的事(abort + 重开 DMA)。 */
    serial_.restart();
}

void init(UART_HandleTypeDef* huart)
{
    Daemon::Config daemon_config = {
        .tim_config = { .htim = &htim5 },
        .cycle = 100,
        .daemon_callback = lostCallback,
        .device = nullptr,
    };
    daemon_.init(daemon_config);

    Serial::Config serial_config = {
        .usart_handle = huart,
        .htim = &htim5,
        .rx_callback = serialCallback,
    };
    serial_.init(serial_config);
}

uint8_t online()
{
    return daemon_.online_;
}

void restart()
{
    serial_.restart();
}

}  // namespace remote_vt13

#endif  // REMOTE_DEVICE_VT13
