#ifndef SHOOT_H
#define SHOOT_H

#include "robot_def.h"

/**
 * @brief 发射初始化,会被RobotInit()调用
 * 
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();

/**
 * @brief 发射回传的反馈数据(原 message_center 订阅, 现为全局实例, 定义在 shoot.cpp)
 */
extern Shoot_Upload_Data_s shoot_feedback_data;

#endif // SHOOT_H