/**
 * @file    bsp_soft_i2c.cpp
 * @brief   软件 I2C 实现
 */
#include "bsp_soft_i2c.h"
#include "bsp_dwt.h"

void SoftI2C::init(const Config& config) {
    port_ = config.port;
    scl_ = config.scl;
    sda_ = config.sda;
    i2c_tick_ = config.i2c_tick;
    dev_addr_ = config.dev_addr;

    // 将SCL和SDA线设置为高电平（空闲状态）
    HAL_GPIO_WritePin(port_, scl_, GPIO_PIN_SET);
    HAL_GPIO_WritePin(port_, sda_, GPIO_PIN_SET);
}

void SoftI2C::start() {
    // 拉高SCL和SDA线，再拉低SDA线，再拉低SCL线，完成起始信号发送
    HAL_GPIO_WritePin(port_, scl_, GPIO_PIN_SET);
    HAL_GPIO_WritePin(port_, sda_, GPIO_PIN_SET);
    DWT_Delay_us(i2c_tick_);
    HAL_GPIO_WritePin(port_, sda_, GPIO_PIN_RESET);
    DWT_Delay_us(i2c_tick_);
    HAL_GPIO_WritePin(port_, scl_, GPIO_PIN_RESET);
}

void SoftI2C::stop() {
    // 拉低SDA线和SCL线，再拉高SCL线，再拉高SDA线，完成停止信号发送
    HAL_GPIO_WritePin(port_, sda_, GPIO_PIN_RESET);
    DWT_Delay_us(i2c_tick_);
    HAL_GPIO_WritePin(port_, scl_, GPIO_PIN_SET);
    DWT_Delay_us(i2c_tick_);
    HAL_GPIO_WritePin(port_, sda_, GPIO_PIN_SET);
}

void SoftI2C::writeBit(uint8_t bit) {
    // 发送单个位，通过设置SDA线的电平，然后拉高SCL线，再拉低SCL线完成发送
    HAL_GPIO_WritePin(port_, sda_, static_cast<GPIO_PinState>(bit));
    DWT_Delay_us(i2c_tick_);
    HAL_GPIO_WritePin(port_, scl_, GPIO_PIN_SET);
    DWT_Delay_us(i2c_tick_);
    HAL_GPIO_WritePin(port_, scl_, GPIO_PIN_RESET);
}

uint8_t SoftI2C::readBit() {
    // 拉高SCL线再拉低，读取SDA线电平
    HAL_GPIO_WritePin(port_, sda_, GPIO_PIN_SET); // 释放SDA线，准备读取
    DWT_Delay_us(i2c_tick_);
    HAL_GPIO_WritePin(port_, scl_, GPIO_PIN_SET);
    DWT_Delay_us(i2c_tick_);
    uint8_t bit = static_cast<uint8_t>(HAL_GPIO_ReadPin(port_, sda_));
    HAL_GPIO_WritePin(port_, scl_, GPIO_PIN_RESET);

    // 返回读取到的位值
    return bit;
}

uint8_t SoftI2C::writeByte(uint8_t byte) {
    // 由高位到低位逐位发送数据
    for (uint8_t i = 0; i < 8; i++)
        writeBit(byte >> (7 - i) & 0x01);
    // 读取从机应答位Ack/Nack
    return readBit();
}

uint8_t SoftI2C::readByte(uint8_t ack) {
    // 由高位到低位逐位读取数据
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++)
        byte = (byte << 1) + readBit();
    // 发送ACK/NACK
    writeBit(ack);
    return byte;
}

void SoftI2C::write(uint8_t* data, uint16_t length) {
    // 发送起始信号
    start();
    // 发送设备地址（写操作），检查应答
    if (writeByte(dev_addr_)) {
        stop();
        return;
    }
    // 逐字节发送数据，并检查每个字节的应答
    for (uint16_t i = 0; i < length; i++) {
        if (writeByte(data[i])) {
            stop();
            return;
        }
    }
    // 发送停止信号
    stop();
}

void SoftI2C::read(uint8_t* data, uint16_t length) {
    // 发送起始信号
    start();
    // 发送设备地址（读操作），检查应答
    if (writeByte(dev_addr_ | 0x01)) {
        stop();
        return;
    }
    // 逐字节读取数据，最后一个字节发送NACK，其余发送ACK
    for (uint16_t i = 0; i < length; i++) {
        if (i == length - 1)
            data[i] = readByte(SOFT_I2C_NACK);
        else
            data[i] = readByte(SOFT_I2C_ACK);
    }
    // 发送停止信号
    stop();
}

void SoftI2C::memWrite(uint8_t reg_addr, uint8_t* data, uint16_t length) {
    // 发送起始信号
    start();
    // 发送设备地址（写操作），检查应答
    if (writeByte(dev_addr_)) {
        stop();
        return;
    }
    // 发送寄存器地址，检查应答
    if (writeByte(reg_addr)) {
        stop();
        return;
    }
    // 逐字节发送数据，并检查每个字节的应答
    for (uint16_t i = 0; i < length; i++) {
        if (writeByte(data[i])) {
            stop();
            return;
        }
    }
    // 发送停止信号
    stop();
}

void SoftI2C::memRead(uint8_t reg_addr, uint8_t* data, uint16_t length) {
    // 发送起始信号
    start();
    // 发送设备地址（写操作），检查应答
    if (writeByte(dev_addr_)) {
        stop();
        return;
    }
    // 发送寄存器地址，检查应答
    if (writeByte(reg_addr)) {
        stop();
        return;
    }

    // 重新发送起始信号
    start();
    // 发送设备地址（读操作），检查应答
    if (writeByte(dev_addr_ | 0x01)) {
        stop();
        return;
    }
    // 逐字节读取数据，最后一个字节发送NACK，其余发送ACK
    for (uint16_t i = 0; i < length; i++) {
        if (i == length - 1)
            data[i] = readByte(SOFT_I2C_NACK);
        else
            data[i] = readByte(SOFT_I2C_ACK);
    }
    // 发送停止信号
    stop();
}
