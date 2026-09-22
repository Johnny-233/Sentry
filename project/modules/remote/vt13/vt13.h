/**
 * @file    vt13.h
 * @brief   VT13/VT03 图传遥控协议驱动
 */
#pragma once

#include "remote_config.h"

#if defined(REMOTE_DEVICE_VT13)

#include "bsp_usart.h"
#include <stdint.h>

#define REMOTE_VT13_FRAME_LEN 21

#pragma pack(1)

typedef struct {
    uint8_t  sof_1;
    uint8_t  sof_2;
    uint64_t ch_0      : 11;
    uint64_t ch_1      : 11;
    uint64_t ch_2      : 11;
    uint64_t ch_3      : 11;
    uint64_t mode_sw   :  2;
    uint64_t pause     :  1;
    uint64_t fn_1      :  1;
    uint64_t fn_2      :  1;
    uint64_t wheel     : 11;
    uint64_t trigger   :  1;
    int16_t  mouse_x;
    int16_t  mouse_y;
    int16_t  mouse_z;
    uint8_t  mouse_left  : 2;
    uint8_t  mouse_right : 2;
    uint8_t  mouse_middle: 2;
    uint16_t key;
    uint16_t crc16;
} remote_frame_t;

#pragma pack()

static_assert(sizeof(remote_frame_t) == REMOTE_VT13_FRAME_LEN, "remote_frame_t size mismatch");

namespace remote_vt13 {

void init(UART_HandleTypeDef* huart);
uint8_t online();
void restart();   // force-restart the UART reception (HAL DMA-RX wedge recovery)

extern uint8_t data_flag;
extern const remote_frame_t* const data;

}  // namespace remote_vt13

#endif  // REMOTE_DEVICE_VT13
