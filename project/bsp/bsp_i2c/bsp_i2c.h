/**
 * @file    bsp_i2c.h
 * @brief   硬件 I2C（C → C++）
 * @note    从 C 版 bsp_i2c 迁移：struct HARD_I2C_Instance → class HardI2C，
 *          HARD_I2C_Init → init、HARD_I2C_Write/Read/Mem_* → write/read/memWrite/memRead，
 *          逻辑不变，禁堆。
 */
#pragma once

#include "i2c.h"
#include <stdint.h>

#define I2C_ACK  0
#define I2C_NACK 1

class HardI2C {
public:
    /// 初始化配置
    struct Config {
        I2C_HandleTypeDef* hi2c;    ///< I2C 句柄
        uint8_t dev_addr;           ///< 设备地址
    };

    void init(const Config& config);  ///< 替代 HARD_I2C_Init（禁堆）
    void write(uint8_t* data, uint16_t length);                       ///< 替代 HARD_I2C_Write
    void read(uint8_t* data, uint16_t length);                        ///< 替代 HARD_I2C_Read
    void memWrite(uint8_t reg_addr, uint8_t* data, uint16_t length);  ///< 替代 HARD_I2C_Mem_Write
    void memRead(uint8_t reg_addr, uint8_t* data, uint16_t length);   ///< 替代 HARD_I2C_Mem_Read

private:
    I2C_HandleTypeDef* hi2c_ = nullptr;  ///< I2C 句柄
    uint8_t dev_addr_ = 0;               ///< 设备地址
};
