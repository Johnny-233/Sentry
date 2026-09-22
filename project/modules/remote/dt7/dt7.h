/**
 * @file    dt7.h
 * @brief   DT7 遥控器接收器协议驱动（摇杆/双开关/拨轮，无键鼠）
 */
#pragma once

#include "remote_config.h"

#if defined(REMOTE_DEVICE_DT7)

#include "bsp_usart.h"
#include <stdint.h>

#define REMOTE_DT7_FRAME_LEN 18

#define REMOTE_DT7_CH_MID 1024
#define REMOTE_DT7_CH_MIN 364
#define REMOTE_DT7_CH_MAX 1684

#define REMOTE_DT7_SW_UP   1
#define REMOTE_DT7_SW_DOWN 2
#define REMOTE_DT7_SW_MID  3

typedef struct {
    int16_t rocker_right_x;  ///< 右摇杆水平（已减 mid，约 ±660）
    int16_t rocker_right_y;  ///< 右摇杆竖直
    int16_t rocker_left_x;   ///< 左摇杆水平
    int16_t rocker_left_y;   ///< 左摇杆竖直
    int16_t dial;            ///< 拨轮
    uint8_t switch_left;     ///< 左开关：UP/MID/DOWN
    uint8_t switch_right;    ///< 右开关：UP/MID/DOWN
} dt7_rc_t;

namespace remote_dt7 {

void init(UART_HandleTypeDef* huart);
uint8_t online();
void restart();

extern uint8_t data_flag;
extern const dt7_rc_t* const data;

}  // namespace remote_dt7

#endif  // REMOTE_DEVICE_DT7
