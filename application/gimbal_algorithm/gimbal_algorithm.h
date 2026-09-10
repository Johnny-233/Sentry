#include <stdbool.h>
#include "ins_task.h"
#include "robot_def.h"
#include "dji_motor.h"

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

float Cal_FollowControl_Set_Yaw(attitude_t gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv);
float Cal_FollowControl_Set_Pitch(attitude_t gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv);
float Cal_FollowControl_Feedforward(attitude_t gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv);

// === Yaw 滤波系数 ===
#define DEFAULT_FILTER_FACTOR 0.9f  // 默认
#define STEP_FILTER_FACTOR 0.4f      // 阶跃
#define CORNER_FILTER_FACTOR 0.6f    // 三角形拐点
#define FAST_FILTER_FACTOR 0.6f     // 快速连续变化
#define SLOW_FILTER_FACTOR 0.99f      // 静止，强滤波抑制视觉噪声

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
