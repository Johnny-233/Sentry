#include "gimbal_algorithm.h"
#include "ins_task.h"
static GimbalAlgorithm_t gimbal_algorithm;

/**
 * @brief 方向变化检测函数
 * @param current_cmd_delta 当前指令变化量
 * @param current_time 当前时间(ms)
 */
static void DetectDirectionChange(GimbalAlgorithm_t *gimbal)
{
    float current_cmd_delta  =    gimbal->yaw_cmd_delta;
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
 * @brief 云台偏航轴自适应跟随控制
 * @param gimbal_IMU_data 云台IMU数据
 * @param gimbal_cmd_recv 云台控制指令
 */
float Cal_FollowControl_Set(attitude_t gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv) 
{
    // 获取当前时间和角度
    gimbal_algorithm.current_time = DWT_GetTimeline_ms();
    gimbal_algorithm.current_yaw = gimbal_IMU_data.YawTotalAngle;
    gimbal_algorithm.yaw_cmd = gimbal_cmd_recv.yaw;
    
    // 计算指令变化量
    gimbal_algorithm.yaw_cmd_delta = gimbal_algorithm.yaw_cmd - gimbal_algorithm.yaw_last_cmd;
    
    // 方向变化检测
    DetectDirectionChange(&gimbal_algorithm);

    // 计算误差
    gimbal_algorithm. yaw_error = gimbal_algorithm.yaw_cmd  -  gimbal_algorithm.current_yaw ;
    gimbal_algorithm. yaw_error_rate = 0 - gimbal_IMU_data.Gyro[2];  
    
    // 自适应滤波参数选择
    gimbal_algorithm.filter_factor = DEFAULT_FILTER_FACTOR; // 默认滤波系数
    
    if (fabsf(gimbal_algorithm.yaw_cmd_delta) > 10.0f) 
    {
        // 阶跃信号：降低滤波强度，快速响应
        gimbal_algorithm. filter_factor = STEP_FILTER_FACTOR;
        gimbal_algorithm.flag = 1;
    }
    else if (gimbal_algorithm.direction_changed) 
    {
        // 三角波拐点：中等滤波，适当前馈
        gimbal_algorithm.filter_factor = CORNER_FILTER_FACTOR;
        gimbal_algorithm.flag = 2;
    }
    else if (fabsf(gimbal_algorithm.yaw_cmd_delta) > 0.8f)
    {
        // 快速连续变化（三角波线性段）
        gimbal_algorithm.filter_factor = FAST_FILTER_FACTOR;
        gimbal_algorithm.flag = 3;
    }
    else 
    {
        // 微小变化或静止：强滤波
        gimbal_algorithm.filter_factor = SLOW_FILTER_FACTOR;
        gimbal_algorithm.flag = 0;
    }
    
    // 静态误差累积（仅在静止或微小变化时）
    if (fabsf(gimbal_algorithm.yaw_cmd_delta) < 0.2f) 
    {
        if (fabsf(gimbal_algorithm.yaw_error) > 0.3f && fabsf(gimbal_algorithm.yaw_error_rate) < 0.02f)
        {
            gimbal_algorithm.yaw_static_error_accumulator += gimbal_algorithm.yaw_error * 0.001f;
            // 限幅
            gimbal_algorithm.yaw_static_error_accumulator = fmaxf(-0.3f, fminf(gimbal_algorithm.yaw_static_error_accumulator, 0.3f));
        }
    }
    else 
    {
        // 动态时清空静态误差累积
        gimbal_algorithm.yaw_static_error_accumulator *= 0.001f;  // 逐渐衰减
    }
    
    // 应用滤波
    gimbal_algorithm.yaw_filtered_cmd = gimbal_algorithm.filter_factor * gimbal_algorithm.yaw_cmd + (1.0f - gimbal_algorithm.filter_factor) * gimbal_algorithm.yaw_filtered_cmd;
    
    // 添加静态误差补偿
    gimbal_algorithm.yaw_filtered_cmd += gimbal_algorithm.yaw_static_error_accumulator;
    
    // 更新历史变量
    gimbal_algorithm.yaw_last_cmd =  gimbal_algorithm.yaw_cmd;
    gimbal_algorithm.last_cmd_delta =  gimbal_algorithm.yaw_cmd_delta;

    return gimbal_algorithm.yaw_filtered_cmd;
}

float Cal_FollowControl_Feedforward(attitude_t gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv) 
{
    // 计算前馈
    gimbal_algorithm.feedforward = DEFAULT_FEEDFORWARD_FACTOR;
    
    if (fabsf(gimbal_algorithm.yaw_cmd_delta) > 2.0f && fabsf(gimbal_algorithm.yaw_cmd_delta) < 5.0f) 
    {
        // 三角波线性段
        gimbal_algorithm.feedforward = gimbal_algorithm.yaw_cmd_delta * FAST_FEEDFORWARD_FACTOR;
    }
    else if (fabsf(gimbal_algorithm.yaw_cmd_delta) >= 15.0f) 
    {
        // 大幅变化（阶跃）
        gimbal_algorithm.feedforward = gimbal_algorithm.yaw_error * STEP_FEEDFORWARD_FACTOR;
    }
    else
    {
        if (fabsf(gimbal_algorithm.yaw_error) > 0.2f) 
        {
            // 小变化但有误差
            gimbal_algorithm.feedforward = gimbal_algorithm.yaw_error * SLOW_FEEDFORWARD_FACTOR;
        }
        
    }

    // 拐点处适当减小前馈
    if (gimbal_algorithm.direction_changed) 
    {
        gimbal_algorithm.feedforward *= CORNER_FEEDFORWARD_FACTOR;
    }
    
    return gimbal_algorithm.feedforward;
}

