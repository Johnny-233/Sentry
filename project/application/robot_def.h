#pragma once
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

/* ============================================================================
 * application 层公共类型定义（C → C++ 移植）
 *
 * 与原 C 版 application/robot_def.h 的差异（都是"接口改对方的"导致的）：
 *   1. 姿态来源：原来的 attitude_t（Modules/imu/ins_task.h）→ 对方的 AHRS::Output；
 *      两者都是**角度制(deg)**，YawTotalAngle ↔ output_.yaw_total 语义一致。
 *   2. 敌我颜色：原来的 Enemy_Color_e（Modules/master_machine/master_process.h）
 *      → uint8_t，取值同裁判系统 id.robot_color（0=红 1=蓝）。
 *   3. 不再 include message_center.h / ins_task.h / master_process.h。
 * 其余枚举、结构体、字段与原版**逐字段一致**，控制逻辑不变。
 * ==========================================================================*/

#include "ahrs.h"          /* 对方的姿态模块（原 ins_task） */
#include <stdint.h>

/* 常用数学常量：值与原 Modules/general_def.h **完全一致**，
   这样各处重复定义（宏重定义要求 token 完全相同）也不会冲突 */
#ifndef PI
#define PI 3.1415926535f
#endif
#ifndef PI2
#define PI2 (PI * 2.0f) // 2 pi
#endif
#ifndef RAD_2_DEGREE
#define RAD_2_DEGREE 57.2957795f // 180/pi
#endif
#ifndef DEGREE_2_RAD
#define DEGREE_2_RAD 0.01745329252f // pi/180
#endif
#ifndef RPM_2_ANGLE_PER_SEC
#define RPM_2_ANGLE_PER_SEC 6.0f // ×360°/60sec
#endif
#ifndef RPM_2_RAD_PER_SEC
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec
#endif

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车

#define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 5857  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 1 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
/* pitch 软限位(rad, "上电回零坐标系": 零点 = 上电回零时确定的机械下限)
 * 为什么用这个坐标系: MI 电机上报角度的绝对基准每次上电都不同(实测数值不一致),
 * 所以 gimbal.cpp 里实现了上电回零(homing): 首次进入云台模式时缓慢往下找机械下限并在那里设零。
 * 因此下面的数值是相对"机械下限"的: 实测行程 0.910 rad = 52.1 度, 向上为负方向,
 * 两端各留约 0.05 rad(2.7 度)余量 => 下限侧 -0.02, 上限侧 -0.86。 */
#define PITCH_MAX_ANGLE -0.02         // 机械下限侧(刚离开限位一点, 避免顶死)
#define PITCH_MIN_ANGLE -0.86         // 机械上限侧(行程 0.91 - 余量)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 49.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define OMNI_WHEEL_CHASSIC_RADIUS 230 //全向轮底盘半径
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 60.0f          // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮周长(速度计算用)

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;

typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

typedef enum
{
    AUTO_OFF = 0,
    AUTO_ON,
    FIND_Enermy,
} AutoAim_mode_e;

// 功率限制,从裁判系统获取
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/**
 * @brief 中场巡航控制参数
 */
typedef struct {
    uint8_t flag;               // 巡航启用标志
    float yaw_init;             // 巡航起始角度
    float yaw_total_angle;      // 云台累计转角
    float yaw;                  // 当前目标偏航角
    int direction;              // 扫描方向(1:顺时针 -1:逆时针)
    uint8_t Power_Out;          // 掉线保护标志
} cal_mid_round_patrol_t;

/**
 * @brief 全场巡航控制参数
 */
typedef struct {
    int32_t init_totol_round;   // 初始全场圈数
    int32_t total_round;        // 当前总巡航圈数
    uint8_t flag;               // 巡航状态标志
    float yaw_init;             // 起始基准角度
} cal_round_patrol_t;

/* ----------------用于记录时间或标志位的结构体---------------- */
typedef struct
{
    float t_shoot;
    float t_pitch;
    float t_cmd_error;
    uint8_t vision_flag;  // 视觉系统工作标志
    uint8_t aim_flag;
    uint8_t shoot_flag;
    uint8_t cmd_error_flag;
    uint8_t fire_flag;
    uint8_t reverse_flag;
    uint8_t ACEntryPoint; // 自动控制入口点
} DataLebel_t;

/* ----------------CMD 应用的控制数据（原由 message_center 发布/订阅, 现改为直接访问）---------------- */
// cmd 的底盘控制数据, 由 chassis 直接读
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float vx_dir;       // 前进方向原始速度
    float vy_dir;       // 横移方向原始速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    float chassis_rotate_buff;
    float chassis_speed_buff;
    float power_limit;
} Chassis_Ctrl_Cmd_s;

// cmd 的云台控制数据, 由 gimbal 直接读
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float real_pitch;
    float chassis_rotate_wz;
    AutoAim_mode_e autoaim_mode;
    gimbal_mode_e gimbal_mode;
    float last_deep;               // cached copy of Vision.can_fire (offline detection)
    uint8_t Death_reInit;          // 死亡重初始化标志
} Gimbal_Ctrl_Cmd_s;

// cmd 的发射控制数据, 由 shoot 直接读
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e loader_mode;
    friction_mode_e friction_mode;
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis 的反馈数据（由 cmd 直接读）----------------*/
typedef struct
{
    uint8_t enemy_color;   // 1 for blue, 0 for red（原 Enemy_Color_e）
    uint16_t robot_level;
    uint8_t power_flag;
} Chassis_Upload_Data_s;

typedef struct
{
    AHRS::Output gimbal_imu_data;   // 原 attitude_t
    uint16_t yaw_motor_single_round_angle;
    float offset_diff;   // 云台与底盘偏角差
    float pitch_angle;   // pitch电机角度
    uint8_t cmd_error_flag;
    float init_location;
} Gimbal_Upload_Data_s;

typedef struct
{
    uint8_t cmd_error_flag;
    int16_t loader_speed_aps;
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
