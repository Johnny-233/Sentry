/**
 * @file    bsp_spi.cpp
 * @brief   SPI 软件片选主机实现
 */
#include "bsp_spi.h"

void SPI::init(const Config& config) {
    hspi_ = config.hspi;
    cs_port_ = config.cs_port;
    cs_pin_ = config.cs_pin;

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

/**
 * @brief 轮询模式 SPI 传输
 * @note  完全阻塞，使用 HAL_SPI_TransmitReceive 实现，
 *        自动拉低/拉高片选，调用前确保 SPI 空闲即可
 */
void SPI::transfer(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t len) {
    if (len == 0) return;

    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi_, tx_buf, rx_buf, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}
