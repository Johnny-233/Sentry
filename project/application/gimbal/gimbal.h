/**
 * @file    gimbal.h
 * @brief   云台应用层（C → C++）
 * @note    由 application/gimbal/gimbal.h 移植：入口函数名不变，返回类型换成对方框架的
 *          DJIMotor* / MIMotor*；新增云台反馈数据的外部声明（规约 §1.5：定义在 gimbal.cpp）。
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#include "dji_motor.h"
#include "mi_motor.h"
#include "robot_def.h"   /* Gimbal_Upload_Data_s */

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
void GimbalInit();

/**
 * @brief 云台任务
 *
 */
void GimbalTask();

/**
 * @brief 获取yaw电机实例指针
 */
DJIMotor* GetYawMotor(void);

/**
 * @brief 获取pitch电机实例指针
 */
MIMotor* GetPitchMotor(void);

/* 回传给cmd的云台状态信息（定义在 gimbal.cpp；原经 message_center 发布，现由 cmd 直接读） */
extern Gimbal_Upload_Data_s gimbal_feedback_data;

#endif // GIMBAL_H
