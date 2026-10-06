#ifndef CHASSIS_H
#define CHASSIS_H

#include "robot_def.h"

/* 底盘应用初始化, 需在开启 RTOS 之前调用(由 Robot_Init 调用) */
void ChassisInit();

/* 底盘应用任务, 由 200Hz 的 Robot 核心任务调用 */
void ChassisTask();

/* 底盘回传的反馈数据(定义在 chassis.cpp) */
extern Chassis_Upload_Data_s chassis_feedback_data;

#endif // CHASSIS_H