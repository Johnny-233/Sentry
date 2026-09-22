/**
 * @file    bsp_soft_i2c.h
 * @brief   软件 I2C（C → C++）
 * @note    从 C 版 bsp_soft_i2c 迁移：struct SOFT_I2C_Instance → class SoftI2C，
 *          SOFT_I2C_Init → init、SOFT_I2C_Write/Read/Mem_* → write/read/memWrite/memRead，
 *          逻辑不变，禁堆。阻塞式软件模拟 I2C。
 */
#pragma once

#include "gpio.h"
#include <stdint.h>

#define SOFT_I2C_ACK  0   /*!< 应答信号 */
#define SOFT_I2C_NACK 1   /*!< 非应答信号 */

class SoftI2C {
public:
    /// 初始化配置
    struct Config {
        GPIO_TypeDef* port;     ///< GPIO 端口
        uint32_t i2c_tick;      ///< I2C 时钟节拍
        uint16_t scl;           ///< SCL 引脚
        uint16_t sda;           ///< SDA 引脚
        uint8_t dev_addr;       ///< 设备地址
    };

    void init(const Config& config);  ///< 替代 SOFT_I2C_Init（禁堆）
    void write(uint8_t* data, uint16_t length);                       ///< 替代 SOFT_I2C_Write
    void read(uint8_t* data, uint16_t length);                        ///< 替代 SOFT_I2C_Read
    void memWrite(uint8_t reg_addr, uint8_t* data, uint16_t length);  ///< 替代 SOFT_I2C_Mem_Write
    void memRead(uint8_t reg_addr, uint8_t* data, uint16_t length);   ///< 替代 SOFT_I2C_Mem_Read

private:
    void start();                    ///< 起始信号
    void stop();                     ///< 停止信号
    void writeBit(uint8_t bit);      ///< 写 1 位
    uint8_t readBit();               ///< 读 1 位
    uint8_t writeByte(uint8_t byte); ///< 写 1 字节，返回 ACK/NACK
    uint8_t readByte(uint8_t ack);   ///< 读 1 字节，ack=1 表示发送 NACK

    GPIO_TypeDef* port_ = nullptr;  ///< GPIO 端口
    uint32_t i2c_tick_ = 0;         ///< I2C 时钟节拍
    uint16_t scl_ = 0;              ///< SCL 引脚
    uint16_t sda_ = 0;              ///< SDA 引脚
    uint8_t dev_addr_ = 0;          ///< 设备地址
};
