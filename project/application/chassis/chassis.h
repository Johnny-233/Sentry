#ifndef CHASSIS_H
#define CHASSIS_H

#include "robot_def.h"

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

/**
 * @brief 底盘回传的反馈数据(原 message_center 订阅, 现为全局实例, 定义在 chassis.cpp)
 */
extern Chassis_Upload_Data_s chassis_feedback_data;

#endif // CHASSIS_H