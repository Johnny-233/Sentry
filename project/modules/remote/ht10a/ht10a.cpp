/**
 * @file    ht10a.cpp
 * @brief   HT-10A 遥控器 SBUS 协议驱动实现
 */
#include "ht10a.h"

#if defined(REMOTE_DEVICE_HT10A)

#include "daemon.h"
#include "serial.h"
#include <string.h>

namespace remote_ht10a {

static Serial serial_;
static Daemon daemon_;
static ht10a_rc_t rc_;

uint8_t data_flag;
const ht10a_rc_t* const data = &rc_;

static void sbusParse(const uint8_t* buf)
{
    if (buf[0] != 0x0F || buf[24] != 0x00)
        return;

    rc_.channels[0]  = ((uint16_t)buf[1] | ((uint16_t)buf[2] << 8)) & 0x07FF;
    rc_.channels[1]  = (((uint16_t)buf[2] >> 3) | ((uint16_t)buf[3] << 5)) & 0x07FF;
    rc_.channels[2]  = (((uint16_t)buf[3] >> 6) | ((uint16_t)buf[4] << 2) |
                        ((uint16_t)buf[5] << 10)) & 0x07FF;
    rc_.channels[3]  = (((uint16_t)buf[5] >> 1) | ((uint16_t)buf[6] << 7)) & 0x07FF;
    rc_.channels[4]  = (((uint16_t)buf[6] >> 4) | ((uint16_t)buf[7] << 4)) & 0x07FF;
    rc_.channels[5]  = (((uint16_t)buf[7] >> 7) | ((uint16_t)buf[8] << 1) |
                        ((uint16_t)buf[9] << 9)) & 0x07FF;
    rc_.channels[6]  = (((uint16_t)buf[9] >> 2) | ((uint16_t)buf[10] << 6)) & 0x07FF;
    rc_.channels[7]  = (((uint16_t)buf[10] >> 5) | ((uint16_t)buf[11] << 3)) & 0x07FF;
    rc_.channels[8]  = ((uint16_t)buf[12] | ((uint16_t)buf[13] << 8)) & 0x07FF;
    rc_.channels[9]  = (((uint16_t)buf[13] >> 3) | ((uint16_t)buf[14] << 5)) & 0x07FF;
    rc_.channels[10] = (((uint16_t)buf[14] >> 6) | ((uint16_t)buf[15] << 2) |
                        ((uint16_t)buf[16] << 10)) & 0x07FF;
    rc_.channels[11] = (((uint16_t)buf[16] >> 1) | ((uint16_t)buf[17] << 7)) & 0x07FF;
    rc_.channels[12] = (((uint16_t)buf[17] >> 4) | ((uint16_t)buf[18] << 4)) & 0x07FF;
    rc_.channels[13] = (((uint16_t)buf[18] >> 7) | ((uint16_t)buf[19] << 1) |
                        ((uint16_t)buf[20] << 9)) & 0x07FF;
    rc_.channels[14] = (((uint16_t)buf[20] >> 2) | ((uint16_t)buf[21] << 6)) & 0x07FF;
    rc_.channels[15] = (((uint16_t)buf[21] >> 5) | ((uint16_t)buf[22] << 3)) & 0x07FF;

    rc_.rocker_right_x       = (int16_t)rc_.channels[0] - REMOTE_HT10A_CH_MID;
    rc_.rocker_right_y       = (int16_t)rc_.channels[1] - REMOTE_HT10A_CH_MID;
    rc_.rocker_left_y        = (int16_t)rc_.channels[2] - REMOTE_HT10A_CH_MID;
    rc_.rocker_left_x        = (int16_t)rc_.channels[3] - REMOTE_HT10A_CH_MID;
    rc_.switch_right         = (uint8_t)(rc_.channels[4] - REMOTE_HT10A_SW_OFFSET);
    rc_.switch_center_left   = (int16_t)rc_.channels[5];
    rc_.switch_center_right  = (int16_t)rc_.channels[6];
    rc_.switch_left          = (uint8_t)(rc_.channels[7] - REMOTE_HT10A_SW_OFFSET);

    data_flag = 1;
    daemon_.reset();
}

static void serialCallback(uint16_t len)
{
    if (len != REMOTE_HT10A_FRAME_LEN) return;
    sbusParse(serial_.recv_buf_);
}

static void lostCallback(void* device)
{
    (void)device;
    memset(&rc_, 0, sizeof(rc_));
    data_flag = 0;
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

}  // namespace remote_ht10a

#endif  // REMOTE_DEVICE_HT10A
