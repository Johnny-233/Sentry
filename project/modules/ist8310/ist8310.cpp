/**
 * @file    ist8310.cpp
 * @brief   IST8310 磁力计实现（C → C++）
 * @note    原 C 版逻辑不变，仅去 malloc、去 NULL 检查。
 */
#include "ist8310.h"

void IST8310::reset() {
    // 拉低复位引脚 50ms, 再拉高 50ms
    rst_.writePin(GPIO_PIN_RESET);
    HAL_Delay(50);
    rst_.writePin(GPIO_PIN_SET);
    HAL_Delay(50);
}

uint8_t IST8310::readReg(uint8_t reg) {
    uint8_t data = 0;
    i2c_.memRead(reg, &data, 1);
    return data;
}

void IST8310::writeReg(uint8_t reg, uint8_t data) {
    i2c_.memWrite(reg, &data, 1);
}

void IST8310::init(const Config& config) {
    i2c_.init(config.i2c_config);
    rst_.init(config.rst_config);

    // 硬件复位
    reset();

    // WHO_AM_I 校验
    uint8_t id = readReg(IST8310_REG_WHO_AM_I);
    if (id != IST8310_WHO_AM_I_VALUE)
        return;

    // 配置: 200Hz ODR, 使能中断
    writeReg(IST8310_REG_CNTL2, IST8310_INT_ENABLE);
    writeReg(IST8310_REG_CNTL1, IST8310_ODR_200HZ);

    valid_ = true;
}

uint8_t IST8310::acquire(Data& data) {
    uint8_t buf[6];
    i2c_.memRead(IST8310_REG_DATA_OUT, buf, 6);

    // 解析 XYZ 数据 (小端序)
    int16_t raw[3];
    raw[0] = (int16_t)(buf[1] << 8 | buf[0]);  // X
    raw[1] = (int16_t)(buf[3] << 8 | buf[2]);  // Y
    raw[2] = (int16_t)(buf[5] << 8 | buf[4]);  // Z

    // 转换为 µT
    for (uint8_t i = 0; i < 3; i++)
        data.mag[i] = (float)raw[i] * IST8310_MAG_SEN;

    return 0;
}
