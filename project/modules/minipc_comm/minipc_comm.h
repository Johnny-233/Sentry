/**
 * @file    minipc_comm.h
 * @brief   miniPC 通信模块（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 minipc_comm 迁移，逻辑不变。去掉 extern "C"、改用 #pragma once（对齐 crc.h）。
 */
#pragma once

#include <stdint.h>
#include "bsp_usart.h"

#define MINIPC_RX_FRAME_LEN  15    // minipc → STM32
#define MINIPC_TX_FRAME_LEN  20    // STM32 → minipc

#pragma pack(1)

// 接收帧 (minipc → STM32): header(1) + yaw/pitch(8) + can_fire(4) + crc(2) = 15
typedef struct {
    uint8_t  header;          // 0x5A
    float    yaw;
    float    pitch;
    int32_t  can_fire;
    uint16_t crc16;
} minipc_rx_frame_t;

// 发送帧 (STM32 → minipc): header(1) + color(1) + roll/pitch/yaw/speed(16) + crc(2) = 20
typedef struct {
    uint8_t  header;          // 0xA5
    uint8_t  detect_color;
    float    roll;
    float    pitch;
    float    yaw;
    float    bullet_speed;
    uint16_t crc16;
} minipc_tx_frame_t;

#pragma pack()

extern uint8_t minipc_data_flag;

minipc_rx_frame_t *Minipc_Init(UART_HandleTypeDef *huart);
minipc_rx_frame_t *Minipc_GetData();
uint8_t Minipc_Online();
void Minipc_Send(float yaw, float pitch, float roll, float bullet_speed, uint8_t color);
