#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

/**
 * @file    robot_cmd.h
 * @brief   机器人核心控制任务接口
 * @note    chassis/gimbal/shoot 直接读下面的三个 cmd 控制实例与 g_ahrs, 不再走 message_center;
 *          为此需要 include robot_def.h(控制结构体) 与 ahrs.h(AHRS 类定义)。
 */

#include "robot_def.h"
#include "ahrs.h"

/* 机器人核心控制任务初始化, 由 Robot_Init 调用 */
void RobotCMDInit();

/* 机器人核心控制任务, 200Hz 运行(必须高于视觉发送频率) */
void RobotCMDTask();

/* ---------------- cmd 的三个控制实例(定义在 robot_cmd.cpp) ----------------
 * 本文件写, chassis/gimbal/shoot 直接读。 */
extern Chassis_Ctrl_Cmd_s chassis_cmd_recv; // 底盘控制信息(含UI绘制相关)
extern Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;   // 云台控制信息
extern Shoot_Ctrl_Cmd_s shoot_cmd_recv;     // 发射控制信息

/* ---------------- 全局姿态实例(定义在 robot_cmd.cpp) ----------------
 * GimbalInit()/robot.cpp 直接读 output_。 */
extern AHRS g_ahrs;

#endif // !ROBOT_CMD_H
