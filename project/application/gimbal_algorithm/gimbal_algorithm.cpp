/**
 * @file    gimbal_algorithm.cpp
 * @brief   云台自适应跟随算法
 * @note    状态机、判据、滤波/前馈系数宏全部照搬旧 C, 一个数字都没改;
 *          姿态字段映射见 PORT_MAPPING §2。
 *          单位：euler/yaw_total 为 deg; gyro_b 为 rad/s(与旧 INS.Gyro 相同, 故行为一致)。
 */
#include "gimbal_algorithm.h"
#include "bsp_dwt.h"    /* DWT_GetTimeline_ms() */

#include <math.h>       /* fabsf/fmaxf/fminf */

static GimbalAlgorithm_t gimbal_algorithm_yaw;
static GimbalAlgorithm_t gimbal_algorithm_pitch;

/* 复位自适应滤波的内部状态。从 0 电流/停止态回到控制态时必须调用:
 * 否则 filtered_cmd / last_cmd 还是停机前的旧值, 云台会从旧值缓慢爬向新目标
 * (表现为"重新开控制档时云台自己大幅旋转") */
void GimbalAlgorithmReset(void)
{
    gimbal_algorithm_yaw.cmd = 0.0f;
    gimbal_algorithm_yaw.last_cmd = 0.0f;
    gimbal_algorithm_yaw.cmd_delta = 0.0f;
    gimbal_algorithm_yaw.filtered_cmd = 0.0f;
    gimbal_algorithm_yaw.error = 0.0f;
    gimbal_algorithm_yaw.error_rate = 0.0f;
    gimbal_algorithm_yaw.static_error_accumulator = 0.0f;
    gimbal_algorithm_yaw.last_cmd_delta = 0.0f;
    gimbal_algorithm_yaw.direction_changed = false;

    gimbal_algorithm_pitch.cmd = 0.0f;
    gimbal_algorithm_pitch.last_cmd = 0.0f;
    gimbal_algorithm_pitch.cmd_delta = 0.0f;
    gimbal_algorithm_pitch.filtered_cmd = 0.0f;
    gimbal_algorithm_pitch.error = 0.0f;
    gimbal_algorithm_pitch.error_rate = 0.0f;
    gimbal_algorithm_pitch.static_error_accumulator = 0.0f;
    gimbal_algorithm_pitch.last_cmd_delta = 0.0f;
    gimbal_algorithm_pitch.direction_changed = false;
}

/* 方向变化检测(三角波拐点) */
static void DetectDirectionChange(GimbalAlgorithm_t *gimbal)
{
    float current_cmd_delta  =    gimbal->cmd_delta;
    float last_cmd_delta     =    gimbal->last_cmd_delta;
    float current_time       =    gimbal->current_time;

    if (current_cmd_delta * last_cmd_delta < 0 && fabsf(current_cmd_delta) > 0.1f)
    {
        gimbal->direction_changed = true;
        gimbal->last_direction_change_time = current_time;
    }
    else
    {
        // 方向变化后一段时间内仍认为在拐点
        if (current_time - gimbal->last_direction_change_time > 200.0f) {
            gimbal->direction_changed = false;
        }
    }
}

/* 自适应跟随控制核心(通用, 由调用方指定实例和轴)
 * is_pitch=true 用 Pitch 温和参数, false 用 Yaw 快速响应参数 */
static float Cal_FollowControl_Internal(GimbalAlgorithm_t *gimbal, AHRS::Output gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv, float imu_angle, float imu_gyro, float cmd_value, bool is_pitch)
{
    gimbal->current_time = DWT_GetTimeline_ms();
    gimbal->current_angle = imu_angle;
    gimbal->cmd = cmd_value;

    gimbal->cmd_delta = gimbal->cmd - gimbal->last_cmd;

    DetectDirectionChange(gimbal);

    gimbal->error = gimbal->cmd - gimbal->current_angle;
    gimbal->error_rate = 0 - imu_gyro;

    // 自适应滤波参数选择 —— Pitch/Yaw 使用不同的阈值和系数
    gimbal->filter_factor = DEFAULT_FILTER_FACTOR;

    // 每拍(5ms)指令变化量门限: 目标角速度 = 门限/5ms。
    // 原值 10/0.8 意味着目标角速度 <160°/s 时全落进最慢档(τ≈500ms); 现在 4/0.2 对应 <40°/s 用慢档, >40°/s 即进入快档
    float step_threshold   = is_pitch ? PITCH_STEP_THRESHOLD   : 4.0f;
    float fast_threshold   = is_pitch ? PITCH_FAST_THRESHOLD   : 0.2f;
    float step_filter      = is_pitch ? PITCH_STEP_FILTER_FACTOR    : STEP_FILTER_FACTOR;
    float corner_filter    = is_pitch ? PITCH_CORNER_FILTER_FACTOR  : CORNER_FILTER_FACTOR;
    float fast_filter      = is_pitch ? PITCH_FAST_FILTER_FACTOR    : FAST_FILTER_FACTOR;
    float slow_filter      = is_pitch ? PITCH_SLOW_FILTER_FACTOR    : SLOW_FILTER_FACTOR;

    if (fabsf(gimbal->cmd_delta) > step_threshold)
    {
        gimbal->filter_factor = step_filter;
        gimbal->flag = 1;
    }
    else if (gimbal->direction_changed)
    {
        gimbal->filter_factor = corner_filter;
        gimbal->flag = 2;
    }
    else if (fabsf(gimbal->cmd_delta) > fast_threshold)
    {
        gimbal->filter_factor = fast_filter;
        gimbal->flag = 3;
    }
    else
    {
        gimbal->filter_factor = slow_filter;
        gimbal->flag = 0;
    }

    // 静态误差累积（仅在静止或微小变化时）
    if (fabsf(gimbal->cmd_delta) < 0.2f)
    {
        if (fabsf(gimbal->error) > 0.3f && fabsf(gimbal->error_rate) < 0.02f)
        {
            gimbal->static_error_accumulator += gimbal->error * 0.001f;
            gimbal->static_error_accumulator = fmaxf(-0.3f, fminf(gimbal->static_error_accumulator, 0.3f));
        }
    }
    else
    {
        gimbal->static_error_accumulator *= 0.001f;
    }

    gimbal->filtered_cmd = gimbal->filter_factor * gimbal->cmd + (1.0f - gimbal->filter_factor) * gimbal->filtered_cmd;

    gimbal->filtered_cmd += gimbal->static_error_accumulator;

    gimbal->last_cmd = gimbal->cmd;
    gimbal->last_cmd_delta = gimbal->cmd_delta;

    return gimbal->filtered_cmd;
}

/* 云台偏航轴自适应跟随控制 */
float Cal_FollowControl_Set_Yaw(AHRS::Output gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv)
{
    return Cal_FollowControl_Internal(&gimbal_algorithm_yaw, gimbal_IMU_data, gimbal_cmd_recv,
        gimbal_IMU_data.yaw_total, gimbal_IMU_data.gyro_b[2], gimbal_cmd_recv.yaw, false);
}

/* 云台俯仰轴自适应跟随控制(Pitch 专用温和参数) */
float Cal_FollowControl_Set_Pitch(AHRS::Output gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv)
{
    return Cal_FollowControl_Internal(&gimbal_algorithm_pitch, gimbal_IMU_data, gimbal_cmd_recv,
        gimbal_IMU_data.euler[1], gimbal_IMU_data.gyro_b[1], gimbal_cmd_recv.pitch, true);
}

float Cal_FollowControl_Feedforward(AHRS::Output gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv)
{
    GimbalAlgorithm_t *gimbal = &gimbal_algorithm_yaw;

    gimbal->feedforward = DEFAULT_FEEDFORWARD_FACTOR;

    if (fabsf(gimbal->cmd_delta) > 2.0f && fabsf(gimbal->cmd_delta) < 5.0f)
    {
        gimbal->feedforward = gimbal->cmd_delta * FAST_FEEDFORWARD_FACTOR;
    }
    else if (fabsf(gimbal->cmd_delta) >= 15.0f)
    {
        gimbal->feedforward = gimbal->error * STEP_FEEDFORWARD_FACTOR;
    }
    else
    {
        if (fabsf(gimbal->error) > 0.2f)
        {
            gimbal->feedforward = gimbal->error * SLOW_FEEDFORWARD_FACTOR;
        }
    }

    if (gimbal->direction_changed)
    {
        gimbal->feedforward *= CORNER_FEEDFORWARD_FACTOR;
    }

    return gimbal->feedforward;
}
