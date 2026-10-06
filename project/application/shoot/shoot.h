#ifndef SHOOT_H
#define SHOOT_H

#include "robot_def.h"

/* 发射初始化, 需在开启 RTOS 之前调用(由 Robot_Init 调用) */
void ShootInit();

/* 发射任务, 由 200Hz 的 Robot 核心任务调用 */
void ShootTask();

/* 发射回传的反馈数据(定义在 shoot.cpp) */
extern Shoot_Upload_Data_s shoot_feedback_data;

#endif // SHOOT_H