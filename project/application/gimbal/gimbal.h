/**
 * @file    gimbal.h
 * @brief   云台应用层
 * @note    返回类型是对方框架的 DJIMotor* / MIMotor*; 云台反馈数据定义在 gimbal.cpp。
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#include "dji_motor.h"
#include "mi_motor.h"
#include "robot_def.h"   /* Gimbal_Upload_Data_s */

/* 初始化云台, 由 Robot_Init 调用 */
void GimbalInit();

/* 云台任务, 由 200Hz 的 Robot 核心任务调用 */
void GimbalTask();

/* 获取 yaw 电机(GM6020)实例指针 */
DJIMotor* GetYawMotor(void);

/* 获取 pitch 电机(MI)实例指针 */
MIMotor* GetPitchMotor(void);

/* 回传给 cmd 的云台状态信息(定义在 gimbal.cpp, 由 cmd 直接读) */
extern Gimbal_Upload_Data_s gimbal_feedback_data;

#endif // GIMBAL_H
