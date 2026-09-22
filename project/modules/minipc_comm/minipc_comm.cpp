/**
 * @file    minipc_comm.cpp
 * @brief   miniPC 通信模块实现（C → C++）
 * @note    原 C 版逻辑不变；字节缓冲强转结构体指针统一改 memcpy（§9），去 NULL 检查。
 */
#include "minipc_comm.h"
#include "serial.h"
#include "daemon.h"
#include "crc_rm.h"
#include <string.h>

static Serial minipc_serial;
static Daemon minipc_daemon;
static minipc_rx_frame_t minipc_rx_data;
static minipc_tx_frame_t tx;

uint8_t minipc_data_flag;

static void Minipc_SerialCallback(uint16_t len)
{
    if (len != MINIPC_RX_FRAME_LEN) return;

    minipc_rx_frame_t serial_frame;
    memcpy(&serial_frame, minipc_serial.recv_buf_, sizeof(minipc_rx_frame_t));

    if (serial_frame.header != 0x5A) return;
    if (!CRC16_Verify(minipc_serial.recv_buf_, MINIPC_RX_FRAME_LEN)) return;

    memcpy(&minipc_rx_data, minipc_serial.recv_buf_, MINIPC_RX_FRAME_LEN);
    minipc_data_flag = 1;
    minipc_daemon.reset();
}

static void Minipc_Lost_Control(void *device)
{
    (void)device;
}

uint8_t Minipc_Online()
{
    return minipc_daemon.online_;
}

minipc_rx_frame_t *Minipc_Init(UART_HandleTypeDef *huart)
{
    Daemon::Config daemon_config = {
        .tim_config = { .htim = &htim5 },
        .cycle = 100,
        .daemon_callback = Minipc_Lost_Control,
        .device = nullptr,
    };
    minipc_daemon.init(daemon_config);

    Serial::Config serial_config = {
        .usart_handle = huart,
        .htim = &htim5,
        .rx_callback = Minipc_SerialCallback,
    };
    minipc_serial.init(serial_config);

    return &minipc_rx_data;
}

minipc_rx_frame_t *Minipc_GetData()
{
    return &minipc_rx_data;
}

void Minipc_Send(float yaw, float pitch, float roll, float bullet_speed, uint8_t color)
{
    tx.header       = 0xA5;
    tx.detect_color = color;
    tx.roll         = roll;
    tx.pitch        = pitch;
    tx.yaw          = yaw;
    tx.bullet_speed = bullet_speed;

    uint16_t crc = CRC16_Calculate(reinterpret_cast<const uint8_t *>(&tx), 18);
    tx.crc16 = crc;

    minipc_serial.send(reinterpret_cast<uint8_t *>(&tx), MINIPC_TX_FRAME_LEN);
}
