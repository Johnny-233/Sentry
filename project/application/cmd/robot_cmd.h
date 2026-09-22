#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

/**
 * @file    robot_cmd.h
 * @brief   机器人核心控制任务接口(由 application/cmd/robot_cmd.h + robot_cmd.c 移植)
 *
 * 与原 C 版头文件的差异(都是"底层换成对方框架 + 去掉 message_center"导致的):
 *   1. 新增三个 cmd 控制实例的 extern 声明(定义在 robot_cmd.cpp, 规约 §1.5):
 *      chassis/gimbal/shoot 应用直接读这三个全局实例, 不再走 message_center。
 *   2. 新增全局姿态实例 g_ahrs 的 extern 声明(定义在 robot_cmd.cpp):
 *      替代旧版 cmd 持有的 attitude_t*(INS_Init() 返回值), GimbalInit()/robot.cpp 直接用。
 *   3. 为此需要 include robot_def.h(控制结构体) 与 ahrs.h(AHRS 类定义)。
 */

#include "robot_def.h"
#include "ahrs.h"

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

/* ---------------- cmd 的三个控制实例(定义在 robot_cmd.cpp) ----------------
 * 原来由 message_center 的 Publisher 以 "chassis_cmd"/"gimbal_cmd"/"shoot_cmd" 发布,
 * 现在就是普通全局实例: 本文件写, chassis/gimbal/shoot 直接读。 */
extern Chassis_Ctrl_Cmd_s chassis_cmd_recv; // 底盘控制信息(含UI绘制相关)
extern Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;   // 云台控制信息
extern Shoot_Ctrl_Cmd_s shoot_cmd_recv;     // 发射控制信息

/* ---------------- 全局姿态实例(定义在 robot_cmd.cpp) ----------------
 * 替代旧版 INS_Init()/INS_Task() 的 attitude_t; GimbalInit()/robot.cpp 直接读 output_。 */
extern AHRS g_ahrs;

#endif // !ROBOT_CMD_H
