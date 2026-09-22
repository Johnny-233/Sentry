/**
 * @file    bmi088.cpp
 * @brief   BMI088 六轴 IMU 驱动实现（C → C++）
 */
#include "bmi088.h"
#include <string.h>

/*---------- SPI 读写（BMI088 Accel 读需要 dummy byte） ----------*/

void BMI088::accReadReg(uint8_t reg, uint8_t* buf, uint8_t len) {
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};
    tx[0] = 0x80 | reg;
    spi_acc_.transfer(tx, rx, len + 2);
    memcpy(buf, rx + 2, len);
}

void BMI088::accWriteReg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg, data};
    uint8_t rx[2] = {0};
    spi_acc_.transfer(tx, rx, 2);
}

void BMI088::gyroReadReg(uint8_t reg, uint8_t* buf, uint8_t len) {
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};
    tx[0] = 0x80 | reg;
    spi_gyro_.transfer(tx, rx, len + 1);
    memcpy(buf, rx + 1, len);
}

void BMI088::gyroWriteReg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg, data};
    uint8_t rx[2] = {0};
    spi_gyro_.transfer(tx, rx, 2);
}

/*---------- 量程设置 ----------*/

void BMI088::setAccelRange(AccRange range) {
    if (range > AccRange::G24) return;
    accel_range_ = range;
    accWriteReg(BMI088_ACC_RANGE, static_cast<uint8_t>(range));
    DWT_Delay_ms(1);
}

void BMI088::setGyroRange(GyroRange range) {
    // 保持原版判断：>Dps125(=0) 即返回（量程实际已在 gyroInit 中按 gyro_range_ 写入）
    if (range > GyroRange::Dps125) return;
    gyro_range_ = range;
    gyroWriteReg(BMI088_GYRO_RANGE, 4 - static_cast<uint8_t>(range));
    DWT_Delay_ms(1);
}

/*---------- 初始化 ----------*/

uint8_t BMI088::accInit() {
    uint8_t id = 0;

    // BMI088 Accel 上电后默认 I2C 模式，需要 CSB1 上升沿切换到 SPI 模式。
    // 执行一次 SPI 读操作（拉低 CS 再拉高），产生上升沿触发模式切换。
    // 返回值无效，仅用于触发切换。
    accReadReg(BMI088_ACC_CHIP_ID, &id, 1);
    DWT_Delay_ms(1);

    // 软复位，将 Accel 所有寄存器恢复默认值
    accWriteReg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    DWT_Delay_ms(80);

    // 软复位后读取 CHIP_ID 验证 SPI 通信正常
    accReadReg(BMI088_ACC_CHIP_ID, &id, 1);
    DWT_Delay_ms(1);
    accReadReg(BMI088_ACC_CHIP_ID, &id, 1);
    DWT_Delay_ms(1);
    if (id != BMI088_ACC_CHIP_ID_VALUE) return 1;

    // 使能 Accel 测量
    accWriteReg(BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE);
    DWT_Delay_ms(1);
    accWriteReg(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE);
    DWT_Delay_ms(1);

    // 配置 ODR 800Hz，正常带宽
    // ACC_CONF bits[7:4]=acc_bwp=0x0A(Normal), bits[3:0]=acc_odr=0x0B(800Hz) → 0xAB
    accWriteReg(BMI088_ACC_CONF, BMI088_ACC_BWP_NORMAL | BMI088_ACC_ODR_800_HZ);
    DWT_Delay_ms(1);

    return 0;
}

uint8_t BMI088::gyroInit() {
    uint8_t id = 0;

    gyroReadReg(BMI088_GYRO_CHIP_ID, &id, 1);
    DWT_Delay_ms(1);

    // 软复位，将 Gyro 所有寄存器恢复默认值
    gyroWriteReg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    DWT_Delay_ms(80);

    // 读取 CHIP_ID 验证 SPI 通信正常（Gyro 由 PS 引脚直接选择 SPI 模式，无需切换）
    gyroReadReg(BMI088_GYRO_CHIP_ID, &id, 1);
    DWT_Delay_ms(1);
    if (id != BMI088_GYRO_CHIP_ID_VALUE) return 1;

    // 配置量程 (枚举值与寄存器编码反向: 0x00=2000, 0x04=125)
    gyroWriteReg(BMI088_GYRO_RANGE, 4 - static_cast<uint8_t>(gyro_range_));
    DWT_Delay_ms(1);

    // 配置带宽 230Hz，加 ODR 使能位（0x80）
    gyroWriteReg(BMI088_GYRO_BANDWIDTH, BMI088_GYRO_BW_230_HZ | 0x80);
    DWT_Delay_ms(1);

    // 设置正常模式，使能数据就绪中断
    gyroWriteReg(BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE);
    DWT_Delay_ms(1);
    gyroWriteReg(BMI088_GYRO_CTRL, BMI088_GYRO_DRDY_ON);
    DWT_Delay_ms(1);

    return 0;
}

void BMI088::init(const Config& config) {
    spi_acc_.init(config.spi_acc_config);
    spi_gyro_.init(config.spi_gyro_config);

    accel_range_ = config.accel_range;
    gyro_range_ = config.gyro_range;

    // 椭球拟合校准参数
    use_ellipsoid_cal_ = 0;
    memset(accel_offset_, 0, sizeof(accel_offset_));
    memset(accel_M_, 0, sizeof(accel_M_));
    // 检查是否提供了校准参数 (M[0] != 0 表示有有效校准)
    if (config.accel_M[0] != 0.0f) {
        use_ellipsoid_cal_ = 1;
        memcpy(accel_offset_, config.accel_offset, sizeof(accel_offset_));
        memcpy(accel_M_, config.accel_M, sizeof(accel_M_));
    }

    if (accInit() != 0) return;
    if (gyroInit() != 0) return;

    setAccelRange(config.accel_range);
    setGyroRange(config.gyro_range);

    valid_ = true;
}

/*---------- 公有 API ----------*/

void BMI088::readAccel() {
    uint8_t buf[6] = {0};
    accReadReg(BMI088_ACCEL_XOUT_L, buf, 6);

    float accel_scale = (1 << static_cast<uint32_t>(accel_range_)) * BMI088_ACCEL_3G_SEN;
    float raw_x = (int16_t)(buf[1] << 8 | buf[0]) * accel_scale;
    float raw_y = (int16_t)(buf[3] << 8 | buf[2]) * accel_scale;
    float raw_z = (int16_t)(buf[5] << 8 | buf[4]) * accel_scale;

    if (use_ellipsoid_cal_) {
        // 椭球拟合校准: corrected = M × (raw - offset)
        float dx = raw_x - accel_offset_[0];
        float dy = raw_y - accel_offset_[1];
        float dz = raw_z - accel_offset_[2];
        accel.x = accel_M_[0]*dx + accel_M_[1]*dy + accel_M_[2]*dz;
        accel.y = accel_M_[3]*dx + accel_M_[4]*dy + accel_M_[5]*dz;
        accel.z = accel_M_[6]*dx + accel_M_[7]*dy + accel_M_[8]*dz;
    } else {
        // 不校准，直接使用原始值
        accel.x = raw_x;
        accel.y = raw_y;
        accel.z = raw_z;
    }
}

void BMI088::readGyro() {
    uint8_t buf[6] = {0};
    gyroReadReg(BMI088_GYRO_X_L, buf, 6);

    float gyro_scale = (1 << static_cast<uint32_t>(gyro_range_)) * BMI088_GYRO_125_SEN;
    gyro.x = (int16_t)(buf[1] << 8 | buf[0]) * gyro_scale - gyro_offset_[0];
    gyro.y = (int16_t)(buf[3] << 8 | buf[2]) * gyro_scale - gyro_offset_[1];
    gyro.z = (int16_t)(buf[5] << 8 | buf[4]) * gyro_scale - gyro_offset_[2];
}

void BMI088::readTemp() {
    uint8_t buf[2] = {0};
    accReadReg(BMI088_TEMP_M, buf, 2);
    int16_t raw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (raw > 1023) raw -= 2048;
    temperature = (float)raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI088::readAll() {
    readAccel();
    readGyro();
    readTemp();
}

void BMI088::calibrate() {
    const uint16_t cali_times = 6000;
    float gyro_sum[3] = {0};
    float gyro_max[3], gyro_min[3];
    float gyro_diff[3];
    uint32_t start_tick = DWT_GetTimeline_us();
    uint8_t pass = 0;

    do {
        gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0;
        gyro_max[0] = gyro_max[1] = gyro_max[2] = 0;
        gyro_min[0] = gyro_min[1] = gyro_min[2] = 0;

        for (uint16_t i = 0; i < cali_times; i++) {
            readGyro();

            if (i == 0) {
                gyro_max[0] = gyro_min[0] = gyro.x;
                gyro_max[1] = gyro_min[1] = gyro.y;
                gyro_max[2] = gyro_min[2] = gyro.z;
            } else {
                if (gyro.x > gyro_max[0]) gyro_max[0] = gyro.x;
                if (gyro.x < gyro_min[0]) gyro_min[0] = gyro.x;
                if (gyro.y > gyro_max[1]) gyro_max[1] = gyro.y;
                if (gyro.y < gyro_min[1]) gyro_min[1] = gyro.y;
                if (gyro.z > gyro_max[2]) gyro_max[2] = gyro.z;
                if (gyro.z < gyro_min[2]) gyro_min[2] = gyro.z;
            }

            gyro_sum[0] += gyro.x;
            gyro_sum[1] += gyro.y;
            gyro_sum[2] += gyro.z;

            DWT_Delay_ms(1);
        }

        // stability checks (gyro should be stable when stationary)
        gyro_diff[0] = gyro_max[0] - gyro_min[0];
        gyro_diff[1] = gyro_max[1] - gyro_min[1];
        gyro_diff[2] = gyro_max[2] - gyro_min[2];

        pass = (gyro_diff[0] < 0.15f) &&
               (gyro_diff[1] < 0.15f) &&
               (gyro_diff[2] < 0.15f);

    } while (!pass && (DWT_GetTimeline_us() - start_tick) < 12000000);  // 12s timeout

    // set gyro calibration results
    for (uint8_t i = 0; i < 3; i++)
        gyro_offset_[i] = gyro_sum[i] / cali_times;
}
