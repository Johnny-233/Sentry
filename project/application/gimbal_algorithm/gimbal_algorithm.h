/**
 * @file    gimbal_algorithm.h
 * @brief   云台自适应跟随算法
 * @note    结构体字段、宏、函数签名全部照搬旧 C, 一个数字都没改(原头文件没有 include guard, 这里补上)。
 *          单位：欧拉角/累计角 deg; gyro_b 与旧 INS.Gyro 同为 rad/s
 *          (旧 BMI088_GYRO_2000_SEN = 0.001065 rad/s/LSB, 新 ahrs.cpp 也存 rad/s), 故字面映射即行为一致。
 */
#ifndef GIMBAL_ALGORITHM_H
#define GIMBAL_ALGORITHM_H

#include <stdbool.h>
#include "ahrs.h"       /* AHRS::Output */
#include "robot_def.h"  /* Gimbal_Ctrl_Cmd_s */

// 云台算法状态结构体
typedef struct
{
    // 状态变量
    float cmd;                         // 指令
    float last_cmd;                    // 上一次指令
    float cmd_delta;                   // 指令变化量
    float filtered_cmd;                // 滤波后的指令
    float current_angle;               // 当前角度
    float error;                       // 当前与指令差值
    float error_rate;
    float static_error_accumulator;    // 静态误差累积
    float cmd_rate;                    // 指令变化率
    float current_time;

    // 方向检测相关
    bool  direction_changed;           // 方向变化标志
    float last_cmd_delta;              // 上一次指令变化量
    float last_direction_change_time;  // 上次方向变化时间

    // 控制参数
    float filter_factor;               // 当前滤波系数
    float feedforward;                 // 当前前馈值
    float flag;
} GimbalAlgorithm_t;

/* 复位自适应滤波内部状态(从 0 电流态回到控制态时必须调用) */
void GimbalAlgorithmReset(void);

/* 姿态字段映射（PORT_MAPPING §2）：
 *   Pitch → euler[1]     Yaw → euler[2]     YawTotalAngle → yaw_total     Gyro[i] → gyro_b[i] */
float Cal_FollowControl_Set_Yaw(AHRS::Output gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv);
float Cal_FollowControl_Set_Pitch(AHRS::Output gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv);
float Cal_FollowControl_Feedforward(AHRS::Output gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv);

// === Yaw 滤波系数 ===
/* 一阶低通时间常数 τ = 更新周期/(1-filter): RobotTask=200Hz(5ms) 时
 * 0.99 -> τ≈500ms(动目标会明显滞后, 这就是"跟不上"的主因之一), 0.7 -> τ≈17ms, 0.5 -> τ≈10ms */
#define DEFAULT_FILTER_FACTOR 0.6f  // 默认(τ≈12.5ms)
#define STEP_FILTER_FACTOR 0.4f      // 阶跃(τ≈8ms)
#define CORNER_FILTER_FACTOR 0.6f    // 三角形拐点
#define FAST_FILTER_FACTOR 0.6f     // 快速连续变化
#define SLOW_FILTER_FACTOR 0.7f      // 静止/慢速: 0.99 会带来约 0.5s 滞后, 降到 0.7 兼顾噪声与响应

// === Yaw 前馈系数 ===
#define DEFAULT_FEEDFORWARD_FACTOR 2.0f  // 默认
#define STEP_FEEDFORWARD_FACTOR 2.0f     // 阶跃
#define CORNER_FEEDFORWARD_FACTOR 1.5f   // 三角形拐点处，适当减小前馈
#define FAST_FEEDFORWARD_FACTOR 350.0f   // 快速连续变化（车在走，头在追）
#define SLOW_FEEDFORWARD_FACTOR 0.1f     // 小变化但有变化，差多少补多少

// === Pitch 专用参数（范围仅 ±0.82rad ≈ ±47°, 阈值和系数需匹配物理行程）===
// 检测阈值
#define PITCH_STEP_THRESHOLD  0.5f   // 阶跃: >0.5rad(~29°) (yaw:10°)
#define PITCH_FAST_THRESHOLD  0.05f  // 快速: >0.05rad(~3°) (yaw:0.8°)

// 滤波系数 — 比Yaw温和, 但对Pitch范围足够灵敏
#define PITCH_STEP_FILTER_FACTOR   0.65f  // 阶跃: 35%新 (yaw:0.4→60%新)
#define PITCH_CORNER_FILTER_FACTOR 0.75f  // 拐点: 25%新 (yaw:0.6→40%新)
#define PITCH_FAST_FILTER_FACTOR   0.70f  // 跟踪: 30%新 (yaw:0.6→40%新)
#define PITCH_SLOW_FILTER_FACTOR   0.90f  // 静止: 10%新 (yaw:0.99→1%新)

// 前馈系数 — Pitch惯量小,前馈减半
#define PITCH_STEP_FEEDFORWARD_FACTOR   1.0f    // (yaw:2.0)
#define PITCH_CORNER_FEEDFORWARD_FACTOR 1.2f    // (yaw:1.5)
#define PITCH_FAST_FEEDFORWARD_FACTOR   80.0f    // (yaw:350)
#define PITCH_SLOW_FEEDFORWARD_FACTOR   0.05f   // (yaw:0.1)

#endif // GIMBAL_ALGORITHM_H
