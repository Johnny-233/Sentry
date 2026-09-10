#include "gimbal_algorithm.h"
#include "ins_task.h"
static GimbalAlgorithm_t gimbal_algorithm_yaw;
static GimbalAlgorithm_t gimbal_algorithm_pitch;

/**
 * @brief 方向变化检测函数
 */
static void DetectDirectionChange(GimbalAlgorithm_t *gimbal)
{
    float current_cmd_delta  =    gimbal->cmd_delta;
    float last_cmd_delta     =    gimbal->last_cmd_delta;
    float current_time       =    gimbal->current_time;

    // 方向变化检测（三角波拐点）
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

/**
 * @brief 自适应跟随控制核心（通用，由调用方指定实例和轴）
 * @param is_pitch true=Pitch轴(温和参数), false=Yaw轴(快速响应)
 */
static float Cal_FollowControl_Internal(GimbalAlgorithm_t *gimbal, attitude_t gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv, float imu_angle, float imu_gyro, float cmd_value, bool is_pitch)
{
    // 获取当前时间和角度
    gimbal->current_time = DWT_GetTimeline_ms();
    gimbal->current_angle = imu_angle;
    gimbal->cmd = cmd_value;

    // 计算指令变化量
    gimbal->cmd_delta = gimbal->cmd - gimbal->last_cmd;

    // 方向变化检测
    DetectDirectionChange(gimbal);

    // 计算误差
    gimbal->error = gimbal->cmd - gimbal->current_angle;
    gimbal->error_rate = 0 - imu_gyro;

    // 自适应滤波参数选择 —— Pitch/Yaw 使用不同的阈值和系数
    gimbal->filter_factor = DEFAULT_FILTER_FACTOR;

    float step_threshold   = is_pitch ? PITCH_STEP_THRESHOLD   : 10.0f;
    float fast_threshold   = is_pitch ? PITCH_FAST_THRESHOLD   : 0.8f;
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

    // 应用滤波
    gimbal->filtered_cmd = gimbal->filter_factor * gimbal->cmd + (1.0f - gimbal->filter_factor) * gimbal->filtered_cmd;

    // 添加静态误差补偿
    gimbal->filtered_cmd += gimbal->static_error_accumulator;

    // 更新历史变量
    gimbal->last_cmd = gimbal->cmd;
    gimbal->last_cmd_delta = gimbal->cmd_delta;

    return gimbal->filtered_cmd;
}

/**
 * @brief 云台偏航轴自适应跟随控制
 */
float Cal_FollowControl_Set_Yaw(attitude_t gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv)
{
    return Cal_FollowControl_Internal(&gimbal_algorithm_yaw, gimbal_IMU_data, gimbal_cmd_recv,
        gimbal_IMU_data.YawTotalAngle, gimbal_IMU_data.Gyro[2], gimbal_cmd_recv.yaw, false);
}

/**
 * @brief 云台俯仰轴自适应跟随控制（使用 Pitch 专用温和参数）
 */
float Cal_FollowControl_Set_Pitch(attitude_t gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv)
{
    return Cal_FollowControl_Internal(&gimbal_algorithm_pitch, gimbal_IMU_data, gimbal_cmd_recv,
        gimbal_IMU_data.Pitch, gimbal_IMU_data.Gyro[1], gimbal_cmd_recv.pitch, true);
}

float Cal_FollowControl_Feedforward(attitude_t gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv)
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
