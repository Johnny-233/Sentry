/**
 * @file    bsp_i2c.cpp
 * @brief   硬件 I2C 实现
 */
#include "bsp_i2c.h"

void HardI2C::init(const Config& config) {
    hi2c_ = config.hi2c;
    dev_addr_ = config.dev_addr;
}

void HardI2C::write(uint8_t* data, uint16_t length) {
    if (hi2c_ == nullptr) return;

    HAL_I2C_Master_Transmit(hi2c_, dev_addr_, data, length, 100);
}

void HardI2C::read(uint8_t* data, uint16_t length) {
    if (hi2c_ == nullptr) return;

    HAL_I2C_Master_Receive(hi2c_, dev_addr_, data, length, 100);
}

void HardI2C::memWrite(uint8_t reg_addr, uint8_t* data, uint16_t length) {
    if (hi2c_ == nullptr) return;

    HAL_I2C_Mem_Write(hi2c_, dev_addr_, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length, 100);
}

void HardI2C::memRead(uint8_t reg_addr, uint8_t* data, uint16_t length) {
    if (hi2c_ == nullptr) return;

    HAL_I2C_Mem_Read(hi2c_, dev_addr_, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length, 100);
}
