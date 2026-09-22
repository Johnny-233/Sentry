/**
 * @file    dt7.cpp
 * @brief   DT7 遥控器接收器协议驱动实现
 */
#include "dt7.h"

#if defined(REMOTE_DEVICE_DT7)

#include "daemon.h"
#include "serial.h"
#include <stdlib.h>
#include <string.h>

namespace remote_dt7 {

static Serial serial_;
static Daemon daemon_;
static dt7_rc_t rc_;

uint8_t data_flag;
const dt7_rc_t* const data = &rc_;

static void rectifyJoystick()
{
    int16_t* channels[5] = {
        &rc_.rocker_left_x,
        &rc_.rocker_left_y,
        &rc_.rocker_right_x,
        &rc_.rocker_right_y,
        &rc_.dial,
    };
    for (uint8_t i = 0; i < 5; ++i) {
        if (abs(*channels[i]) > 660)
            *channels[i] = 0;
    }
}

static void sbusToRc(const uint8_t* sbus_buf)
{
    rc_.rocker_right_x =
        ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - REMOTE_DT7_CH_MID;
    rc_.rocker_right_y =
        (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - REMOTE_DT7_CH_MID;
    rc_.rocker_left_x =
        (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) -
        REMOTE_DT7_CH_MID;
    rc_.rocker_left_y =
        (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - REMOTE_DT7_CH_MID;
    rc_.dial =
        ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - REMOTE_DT7_CH_MID;

    rectifyJoystick();

    rc_.switch_right = ((sbus_buf[5] >> 4) & 0x0003);
    rc_.switch_left  = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
}

static void serialCallback(uint16_t len)
{
    if (len != REMOTE_DT7_FRAME_LEN) return;

    sbusToRc(serial_.recv_buf_);
    data_flag = 1;
    daemon_.reset();
}

static void lostCallback(void* device)
{
    (void)device;
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

}  // namespace remote_dt7

#endif  // REMOTE_DEVICE_DT7
