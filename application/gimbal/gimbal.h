#ifndef GIMBAL_H
#define GIMBAL_H

#include "dji_motor.h"
#include "mi_motor.h"

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
DJIMotorInstance* GetYawMotor(void);

/**
 * @brief 获取pitch电机实例指针
 */
MIMotorInstance* GetPitchMotor(void);

#endif // GIMBAL_H