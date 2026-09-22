/**
 * @file    bmi088.h
 * @brief   BMI088 六轴 IMU（加速度计 + 陀螺仪）驱动（C → C++）
 * @note    从 C 版 bmi088 迁移：struct BMI088_Instance → class BMI088，Register → init、
 *          BMI088_Read_X/Set_X → readX/setX，逻辑不变，禁堆。
 *          加速度计/陀螺仪分两个 SPI 从机（各占一个软件片选）。
 */
#pragma once

#include "bsp_spi.h"
#include "bsp_dwt.h"
#include <stdint.h>

/*---------- 加速度计寄存器 ----------*/

#define BMI088_ACC_CHIP_ID          0x00
#define BMI088_ACC_CHIP_ID_VALUE    0x1E

#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03

#define BMI088_ACCEL_XOUT_L         0x12
#define BMI088_ACCEL_XOUT_M         0x13
#define BMI088_ACCEL_YOUT_L         0x14
#define BMI088_ACCEL_YOUT_M         0x15
#define BMI088_ACCEL_ZOUT_L         0x16
#define BMI088_ACCEL_ZOUT_M         0x17

#define BMI088_TEMP_M               0x22
#define BMI088_TEMP_L               0x23

#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41

#define BMI088_INT1_IO_CTRL         0x53
#define BMI088_INT_MAP_DATA         0x58

#define BMI088_ACC_SELF_TEST        0x6D

#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRL         0x7D
#define BMI088_ACC_SOFTRESET        0x7E
#define BMI088_ACC_SOFTRESET_VALUE  0xB6

/*---------- 陀螺仪寄存器 ----------*/

#define BMI088_GYRO_CHIP_ID         0x00
#define BMI088_GYRO_CHIP_ID_VALUE   0x0F

#define BMI088_GYRO_X_L             0x02
#define BMI088_GYRO_X_H             0x03
#define BMI088_GYRO_Y_L             0x04
#define BMI088_GYRO_Y_H             0x05
#define BMI088_GYRO_Z_L             0x06
#define BMI088_GYRO_Z_H             0x07

#define BMI088_GYRO_INT_STAT_1      0x0A
#define BMI088_GYRO_RANGE           0x0F
#define BMI088_GYRO_BANDWIDTH       0x10
#define BMI088_GYRO_LPM1            0x11
#define BMI088_GYRO_SOFTRESET       0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6
#define BMI088_GYRO_CTRL            0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP  0x18
#define BMI088_GYRO_SELF_TEST       0x3C

/*---------- 配置值 ----------*/

// 加速度计量程
#define BMI088_ACC_RANGE_3G         0x00
#define BMI088_ACC_RANGE_6G         0x01
#define BMI088_ACC_RANGE_12G        0x02
#define BMI088_ACC_RANGE_24G        0x03

// 加速度计 ODR
#define BMI088_ACC_ODR_12_5_HZ      0x05
#define BMI088_ACC_ODR_25_HZ        0x06
#define BMI088_ACC_ODR_50_HZ        0x07
#define BMI088_ACC_ODR_100_HZ       0x08
#define BMI088_ACC_ODR_200_HZ       0x09
#define BMI088_ACC_ODR_400_HZ       0x0A
#define BMI088_ACC_ODR_800_HZ       0x0B
#define BMI088_ACC_ODR_1600_HZ      0x0C

// 加速度计带宽 (ACC_CONF bits[7:4], 合法值必须左移4位)
#define BMI088_ACC_BWP_OSR4         (0x08 << 4) // 0x80
#define BMI088_ACC_BWP_OSR2         (0x09 << 4) // 0x90
#define BMI088_ACC_BWP_NORMAL       (0x0A << 4) // 0xA0

// 陀螺仪量程
#define BMI088_GYRO_RANGE_2000      0x00
#define BMI088_GYRO_RANGE_1000      0x01
#define BMI088_GYRO_RANGE_500       0x02
#define BMI088_GYRO_RANGE_250       0x03
#define BMI088_GYRO_RANGE_125       0x04

// 陀螺仪带宽
#define BMI088_GYRO_BW_532_HZ       0x00
#define BMI088_GYRO_BW_230_HZ       0x01
#define BMI088_GYRO_BW_116_HZ       0x02
#define BMI088_GYRO_BW_47_HZ        0x03
#define BMI088_GYRO_BW_23_HZ        0x04
#define BMI088_GYRO_BW_12_HZ        0x05
#define BMI088_GYRO_BW_64_HZ        0x06
#define BMI088_GYRO_BW_32_HZ        0x07

// 电源模式
#define BMI088_ACC_PWR_ACTIVE       0x00
#define BMI088_ACC_PWR_SUSPEND      0x03
#define BMI088_ACC_ENABLE           0x04
#define BMI088_ACC_DISABLE          0x00

#define BMI088_GYRO_NORMAL_MODE     0x00
#define BMI088_GYRO_SUSPEND_MODE    0x80
#define BMI088_GYRO_DRDY_ON        0x80

/*---------- 灵敏度系数 ----------*/

#define BMI088_ACCEL_3G_SEN         0.0008974358974f
#define BMI088_ACCEL_6G_SEN         0.00179443359375f
#define BMI088_ACCEL_12G_SEN        0.0035888671875f
#define BMI088_ACCEL_24G_SEN        0.007177734375f

#define BMI088_GYRO_2000_SEN        0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN        0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN         0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN         0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN         0.000066579027251980956150958662738366f

#define BMI088_TEMP_FACTOR          0.125f
#define BMI088_TEMP_OFFSET          23.0f

class BMI088 {
public:
    /// 加速度计量程
    enum class AccRange : uint8_t { G3 = 0, G6 = 1, G12 = 2, G24 = 3 };
    /// 陀螺仪量程（枚举值与寄存器编码反向：0=2000dps ... 4=125dps）
    enum class GyroRange : uint8_t { Dps125 = 0, Dps250 = 1, Dps500 = 2, Dps1000 = 3, Dps2000 = 4 };

    /// 初始化配置
    struct Config {
        SPI::Config spi_acc_config;   ///< 加速度计 SPI 从机配置
        SPI::Config spi_gyro_config;  ///< 陀螺仪 SPI 从机配置
        AccRange accel_range;         ///< 加速度计量程
        GyroRange gyro_range;         ///< 陀螺仪量程

        // 椭球拟合校准参数 (离线生成，不填则不校准)
        float accel_offset[3];        ///< 加速度偏移
        float accel_M[9];             ///< 3×3 修正矩阵（M[0]!=0 表示启用）
    };

    /// 三维向量
    struct Vector3 {
        float x, y, z;
    };

    // —— 输出数据（跨模块读取）——
    Vector3 accel = {};       ///< 加速度 [m/s²]
    Vector3 gyro = {};        ///< 角速度 [rad/s]
    float temperature = 0.0f; ///< 温度 [°C]

    void init(const Config& config);       ///< 替代 BMI088_Register（禁堆）
    bool valid() const { return valid_; }  ///< 芯片 ID 校验是否通过

    void setAccelRange(AccRange range);    ///< 替代 BMI088_Set_Accel_Range
    void setGyroRange(GyroRange range);    ///< 替代 BMI088_Set_Gyro_Range
    void readAccel();                      ///< 替代 BMI088_Read_Accel
    void readGyro();                       ///< 替代 BMI088_Read_Gyro
    void readTemp();                       ///< 替代 BMI088_Read_Temp
    void readAll();                        ///< 替代 BMI088_Read_All
    void calibrate();                      ///< 替代 BMI088_Calibrate

private:
    // SPI 读写（BMI088 Accel 读需要 dummy byte）
    void accReadReg(uint8_t reg, uint8_t* buf, uint8_t len);
    void accWriteReg(uint8_t reg, uint8_t data);
    void gyroReadReg(uint8_t reg, uint8_t* buf, uint8_t len);
    void gyroWriteReg(uint8_t reg, uint8_t data);

    uint8_t accInit();   ///< 加速度计初始化（软复位 + 芯片 ID 校验 + 配置）
    uint8_t gyroInit();  ///< 陀螺仪初始化

    SPI spi_acc_;          ///< 加速度计 SPI 从机
    SPI spi_gyro_;         ///< 陀螺仪 SPI 从机
    AccRange accel_range_ = AccRange::G3;       ///< 加速度计量程
    GyroRange gyro_range_ = GyroRange::Dps125;  ///< 陀螺仪量程
    float gyro_offset_[3] = {};                 ///< 陀螺仪零偏（运行时校准）
    uint8_t use_ellipsoid_cal_ = 0;             ///< 是否使用椭球拟合校准
    float accel_offset_[3] = {};                ///< 加速度偏移
    float accel_M_[9] = {};                     ///< 3×3 修正矩阵
    bool valid_ = false;                        ///< 初始化成功标志
};
