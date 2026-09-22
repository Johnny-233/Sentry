/**
 * @file    ht10a.h
 * @brief   HT-10A 遥控器 SBUS 协议驱动（16 通道原值 + 摇杆/开关具名字段）
 */
#pragma once

#include "remote_config.h"

#if defined(REMOTE_DEVICE_HT10A)

#include "bsp_usart.h"
#include <stdint.h>

#define REMOTE_HT10A_FRAME_LEN 25

#define REMOTE_HT10A_CH_MID    992
#define REMOTE_HT10A_SW_OFFSET 192

#define REMOTE_HT10A_SW_UP   0
#define REMOTE_HT10A_SW_MID  32
#define REMOTE_HT10A_SW_DOWN 64

typedef struct {
    uint16_t channels[16];        ///< SBUS 11bit 原值，不减中位
    int16_t  rocker_right_x;      ///< CH1 - 992
    int16_t  rocker_right_y;      ///< CH2 - 992
    int16_t  rocker_left_y;       ///< CH3 - 992
    int16_t  rocker_left_x;       ///< CH4 - 992
    uint8_t  switch_right;        ///< CH5 - 192，实测 0/32/64
    int16_t  switch_center_left;  ///< CH6 原值
    int16_t  switch_center_right; ///< CH7 原值
    uint8_t  switch_left;         ///< CH8 - 192，实测 0/32/64
} ht10a_rc_t;

namespace remote_ht10a {

void init(UART_HandleTypeDef* huart);
uint8_t online();
void restart();

extern uint8_t data_flag;
extern const ht10a_rc_t* const data;

}  // namespace remote_ht10a

#endif  // REMOTE_DEVICE_HT10A
