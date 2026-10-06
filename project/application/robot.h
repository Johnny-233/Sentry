#ifndef ROBOT_H
#define ROBOT_H

/* 保持 extern "C" 入口, 供 Src/freertos.c 调用 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 机器人初始化。必须在 osKernelInitialize() 之后调用(内部要建任务),
 *        目前在 Src/freertos.c 的 MX_FREERTOS_Init() 里调用。
 */
void Robot_Init(void);

/**
 * @brief 机器人核心任务, 放入实时系统以一定频率(200Hz)运行, 内部会调用各个应用的任务
 */
void Robot_Task(void);

#ifdef __cplusplus
}
#endif

#endif
