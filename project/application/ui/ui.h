#ifndef APP_UI_H
#define APP_UI_H

/**
 * @file    ui.h
 * @brief   裁判系统 UI（司机端界面）契约
 * @note    绘制原语用对方框架的 referee 模块(project/modules/referee/referee.h)：
 *            UIDelete/UILineDraw/... → Referee_UIDelete/Referee_UIDraw(...)
 *            UIGraphRefresh          → Referee_UIRefresh(int cnt, ...)
 *            UICharDraw              → Referee_UIChar(...)
 *            UICharRefresh           → Referee_UICharRefresh(uint8_t* data)
 */

#include "robot_def.h"

/* 模式是否切换标志位，0为未切换，1为切换 */
typedef struct
{
    uint32_t chassis_flag : 1;
    uint32_t gimbal_flag : 1;
    uint32_t shoot_flag : 1;
    uint32_t loader_flag : 1;
    uint32_t Power_flag : 1;
    uint32_t aim_flag : 1;
} Referee_Interactive_Flag_t;

/* 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据 */
typedef struct
{
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    // 为UI绘制以及交互数据所用
    chassis_mode_e chassis_mode;              // 底盘模式
    gimbal_mode_e gimbal_mode;                // 云台模式
    shoot_mode_e shoot_mode;                  // 发射模式设置
    loader_mode_e loader_mode;                // 拨盘模式
    AutoAim_mode_e autoaim_mode;
    Chassis_Power_Data_s Chassis_Power_Data;  // 功率控制

    // 上一次的模式，用于flag判断
    chassis_mode_e chassis_last_mode;
    gimbal_mode_e gimbal_last_mode;
    shoot_mode_e shoot_last_mode;
    loader_mode_e loader_last_mode;
    AutoAim_mode_e autoaim_last_mode;

    Chassis_Power_Data_s Chassis_last_Power_Data;
} Referee_Interactive_info_t;

/* UI 绘制所需的状态数据(定义在 ui.cpp) */
extern Referee_Interactive_info_t ui_data;

/* 在 UI 任务启动时调用一次, 画静态图层 */
void MyUIInit();

/* 裁判系统交互任务(UI 刷新), 由 UI 任务循环调用 */
void UITask();

#endif // APP_UI_H
