/**
 * @file    mahony.cpp
 * @brief   Mahony AHRS 姿态解算实现（逻辑对齐 C 原版，仅换 class + 命名）
 */
#include "mahony.h"
#include <math.h>

#ifndef M_RAD_TO_DEG
#define M_RAD_TO_DEG 57.29577951308232f
#endif
#ifndef M_DEG_TO_RAD
#define M_DEG_TO_RAD 0.017453292519943295f
#endif

void Mahony::init(const Config& config) {
    q0_ = 1.0f;
    q1_ = q2_ = q3_ = 0.0f;
    roll_ = pitch_ = yaw_ = 0.0f;
    ex_int_ = ey_int_ = ez_int_ = 0.0f;
    attitude_.roll = attitude_.pitch = attitude_.yaw = 0.0f;
    initialized_ = false;

    dt_ = config.dt > 0 ? config.dt : 0.005f;
    kp_ = config.kp > 0 ? config.kp : 1.0f;     // 默认降低一点 Kp 增加稳定性
    ki_ = config.ki >= 0 ? config.ki : 0.001f;
}

void Mahony::reset() {
    q0_ = 1.0f;
    q1_ = q2_ = q3_ = 0.0f;
    ex_int_ = ey_int_ = ez_int_ = 0.0f;
    initialized_ = false;
}

/**
 * @brief 官方推荐的 Mahony AHRS 更新算法
 * 针对水平旋转 Yaw 跳变问题进行了优化
 */
void Mahony::update(const IMU_Raw& imu) {
    float ax = imu.ax;
    float ay = imu.ay;
    float az = imu.az;
    float gx = imu.gx * M_DEG_TO_RAD;
    float gy = imu.gy * M_DEG_TO_RAD;
    float gz = imu.gz * M_DEG_TO_RAD;

    float recip_norm;
    float vx, vy, vz;
    float ex = 0, ey = 0, ez = 0;
    float qa, qb, qc;

    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm > 0.9f && acc_norm < 1.1f) {
        // 只有在可信时才做归一化和误差计算
        recip_norm = 1.0f / acc_norm;
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        // 估计方向的重力分量
        vx = 2.0f * (q1_ * q3_ - q0_ * q2_);
        vy = 2.0f * (q0_ * q1_ + q2_ * q3_);
        vz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;

        // 计算误差
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);
    }

    // 积分误差比例增益
    ex_int_ += ex * ki_;
    ey_int_ += ey * ki_;
    ez_int_ += ez * ki_;

    // 限制积分误差，防止积分饱和
    ex_int_ = fmaxf(-0.1f, fminf(0.1f, ex_int_));
    ey_int_ = fmaxf(-0.1f, fminf(0.1f, ey_int_));
    ez_int_ = fmaxf(-0.1f, fminf(0.1f, ez_int_));

    // 应用反馈校正
    gx += kp_ * ex + ex_int_;
    gy += kp_ * ey + ey_int_;
    gz += kp_ * ez + ez_int_;

    // 四元数微分方程
    qa = q0_;
    qb = q1_;
    qc = q2_;

    q0_ += (-qb * gx - qc * gy - q3_ * gz) * (0.5f * dt_);
    q1_ += (qa * gx + qc * gz - q3_ * gy) * (0.5f * dt_);
    q2_ += (qa * gy - qb * gz + q3_ * gx) * (0.5f * dt_);
    q3_ += (qa * gz + qb * gy - qc * gx) * (0.5f * dt_);

    // 归一化四元数
    recip_norm = sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    if (recip_norm > 0.001f) {
        recip_norm = 1.0f / recip_norm;
        q0_ *= recip_norm;
        q1_ *= recip_norm;
        q2_ *= recip_norm;
        q3_ *= recip_norm;
    }

    // 计算欧拉角
    roll_ = atan2f(2.0f * (q0_ * q1_ + q2_ * q3_),
                        1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_)) * M_RAD_TO_DEG;
    pitch_ = asinf(2.0f * (q0_ * q2_ - q3_ * q1_)) * M_RAD_TO_DEG;
    yaw_ = atan2f(2.0f * (q0_ * q3_ + q1_ * q2_),
                       1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_)) * M_RAD_TO_DEG;

    attitude_.roll = roll_;
    attitude_.pitch = pitch_;
    attitude_.yaw = yaw_;

    initialized_ = true;
}
