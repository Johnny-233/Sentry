#include "trajectory_planning.h"

//永久三次多项式插值轨迹规划
float FvCubicPlanning(float start_angle ,float delta_angle , float t)
{
    float outcome;

    // 限制时间范围
    if (t > 1.0f) t = 1.0f;
    
    // 三次多项式系数计算
    // p(t) = a0 + a1*t + a2*t^2 + a3*t^3
    float a0 = start_angle;
    float a1 = 0;
    float a2 = 3 * (delta_angle);
    float a3 = -2 * (delta_angle) ;
    
    // 计算当前位置
    outcome = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
    return outcome;
}

//永久线性插值轨迹规划
float FvLinePlanning(float start_angle ,float end_angle , float t)
{
    float outcome;
    outcome = start_angle + (end_angle-start_angle)*t;
    return outcome;
}

/**
 * 三次多项式插值轨迹规划
 * @param start_angle 起始角度
 * @param end_angle 终止角度
 * @param start_velocity 起始速度
 * @param end_velocity 终止速度
 * @param duration 轨迹持续时间
 * @param current_time 当前时间
 * @param finish_flag 完成标志
 * @return 当前角度值
 */
float CubicPlanning(float start_angle, float end_angle, 
                                   int duration, int current_time, uint8_t* finish_flag)
{
    float outcome;
    
    // 归一化时间
    float t = (float)current_time / (float)duration;
    
    // 限制时间范围
    if (t > 1.0f) t = 1.0f;
    
    // 三次多项式系数计算
    // p(t) = a0 + a1*t + a2*t^2 + a3*t^3
    float a0 = start_angle;
    float a1 = 0;
    float a2 = 3 * (end_angle - start_angle);
    float a3 = 2 * (start_angle - end_angle) ;
    
    // 计算当前位置
    outcome = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
    
    // 设置完成标志
    if (current_time >= duration) 
    {
        outcome = end_angle;
        *finish_flag = 1;
    }
    else
    {
        *finish_flag = 0;
    }
    
    return outcome;
}

//线性插值轨迹规划
float LinePlanning(float start_angle ,float end_angle , int time_length ,int time, uint8_t* finish_flag)
{
    float outcome;
    if(time < time_length)
    {
        outcome = start_angle + (end_angle-start_angle)*time/((float)time_length);
        *finish_flag = 0;
    }
    else
    {
        outcome = end_angle;
        *finish_flag = 1;
    }
    return outcome;
}
