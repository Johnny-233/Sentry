/**
 * @file    mahony.h
 * @brief   Mahony AHRS 姿态解算（C → C++）
 * @note    从 C 版 mahony 迁移：struct → class，Register/Reset/Update → init/reset/update，
 *          逻辑不变，禁堆（原 malloc 实例改为栈/全局对象）。
 */
#pragma once

class Mahony {
public:
    /// 输入：IMU 原始数据（加速度 [m/s²]，陀螺仪 [deg/s]）
    struct IMU_Raw {
        float ax, ay, az;
        float gx, gy, gz;
    };

    /// 输出：姿态角 [deg]
    struct Attitude {
        float roll, pitch, yaw;
    };

    /// 初始化配置
    struct Config {
        float kp;   ///< 比例增益
        float ki;   ///< 积分增益
        float dt;   ///< 采样周期 [s]
    };

    void init(const Config& config);  ///< 替代 Mahony_Init（无堆）
    void update(const IMU_Raw& imu);  ///< 替代 Mahony_Update
    void reset();                     ///< 替代 Mahony_Reset

    // —— 状态数据公开，对齐 C 版字段 ——
    float q0_, q1_, q2_, q3_;                 ///< 四元数
    float roll_, pitch_, yaw_;                ///< 欧拉角 [deg]
    float dt_;                                ///< 采样周期 [s]
    bool  initialized_;                       ///< 是否已解算
    float kp_, ki_;                           ///< 增益
    float ex_int_, ey_int_, ez_int_;          ///< 积分误差
    Attitude attitude_;                       ///< 姿态角输出 [deg]
};
