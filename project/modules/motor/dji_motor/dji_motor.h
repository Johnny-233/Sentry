/**
 * @file    dji_motor.h
 * @brief   DJI 电机（C → C++）
 * @note    从 C 版 dji_motor 迁移：struct DJIMotor_Instance → class DJIMotor，
 *          DJIMotor_Register → init、DJIMotor_Set_* → set*，逻辑不变，禁堆。
 *          原 DJIM_GET_* 宏删除，改为直接读公开字段（motor.angle_ 等）。
 */
#pragma once

#include "dji_motor_def.h"
#include "bsp_can.h"
#include "pid.h"
#include "daemon.h"

#define DJIM_MAX_INSTANCE 16   // 最大电机实例数
#define DJIM_MAX_GROUP 6       // 最大发送组数

/* 速度反馈一阶低通系数（0~1，越小滤波越强）。
   ⚠旧固件 Modules/motor/DJImotor/dji_motor.c 的对应系数是 SPEED_SMOOTH_COEF = 0.85
   （speed_aps = 0.15*old + 0.85*new，属于弱滤波）；框架原值 0.15 在同一采样率下相位
   滞后大约 5~6 倍，kp=10 的速度环装上去就会明显抖。为与旧 PID 数字匹配，这里用 0.85。 */
#define DJIM_VELOCITY_LPF_ALPHA 0.85f
/* 电流反馈一阶低通系数，对应旧固件 CURRENT_SMOOTH_COEF = 0.9（电流内环的反馈用它） */
#define DJIM_CURRENT_LPF_ALPHA 0.9f

class DJIMotor {
public:
    /// PID 控制模式
    enum class PidMode : uint8_t {
        Current = 0,    // FOC_CURRENT_LOOP_CONTROL
        Velocity,       // FOC_VELOCITY_LOOP_CONTROL
        Position,       // FOC_POSITION_LOOP_CONTROL
    };

    /// 反馈原始数据
    struct FeedbackRaw {
        uint16_t ecd;   // 电机编码器值
        int16_t rpm;    // 电机转速
        int16_t curr;   // 电机电流
        uint8_t temp;   // 电机温度
    };

    /// 初始化配置
    struct Config {
        CAN_HandleTypeDef* can_handle;  // CAN 句柄
        uint8_t motor_id;               // 电机 ID（1~8）
        DJIMotor_Type motor_type;       // 电机型号
        uint8_t direction;              // 方向：DJIM_DIRECTION_NORMAL / REVERT
        float reduction_ratio;          // 减速比，电机轴 / 输出轴，默认 1.0
        PID::Config pid_angle;          // 位置环 PID 配置
        PID::Config pid_velocity;       // 速度环 PID 配置
        /* 电流内环 PID 配置 + 使能开关（替代旧固件 close_loop_type 里的 CURRENT_LOOP）。
           启用后：速度环输出 = 电流参考，反馈取电调上报电流，电流 PID 输出才发给电调。
           旧底盘/摩擦轮/拨盘都是"速度环→电流环"两级，缺这一级等效增益差 2~3 倍。 */
        PID::Config pid_current;        // 电流内环 PID 配置（安培域，见 pid_port.h）
        uint8_t current_loop_enable;    // 1 = 启用电流内环
        uint8_t pos_freq_div;           // 位置环分频，更新频率=1000Hz/pos_freq_div
        float initial_angle;            // 初始角度 [deg]（正值=电机当前朝向），内部取反存入 offset
    };

    void init(const Config& config);                 // 替代 DJIMotor_Register
    void setEnable(uint8_t motor_enable);            // 替代 DJIMotor_Set_Enable
    void setAngle(float angle);                      // 替代 DJIMotor_Set_Angle（最短路径）
    void setAngleCircular(float angle);              // 替代 DJIMotor_Set_Angle_Circular（多圈）
    void setAngleIncrement(float angle_increment);   // 替代 DJIMotor_Set_Angle_Increment
    void setVelocity(float velocity);                // 替代 DJIMotor_Set_Velocity
    void setCurrent(float current);                  // 替代 DJIMotor_Set_Current
    void setVelocityFF(float velocity);              // 替代 DJIMotor_Set_VelocityFF
    void setCurrentFF(float current);                // 替代 DJIMotor_Set_CurrentFF

    static void timbaseSelect(TIM_HandleTypeDef* htim);  // 替代 DJIMotor_TimbaseSelect（本工程已改为仅记录，不再挂中断）

    /* 电机环更新入口（替代"挂在 1ms 定时器中断上"的老机制）。
       本工程按旧固件 Modules/motor/motor_task.c 的节奏跑 200Hz(5ms)：所有注册实例跑一次
       速度环 PID，再按 CAN 分组各发一帧控制报文。由应用层 MOTOR 任务 5 分频调用。
       注意调用周期必须等于各 pid_velocity 的 period(ms)（本工程填 5）。 */
    static void taskUpdate();

    // —— 跨模块读取的状态（公开直接读）——
    float angle_ = 0.0f;              // 电机角度 [deg]
    float velocity_ = 0.0f;           // 电机速度 [deg/s]（已一阶低通滤波）
    float velocity_raw_ = 0.0f;       // 电机速度原始值 [deg/s]（未滤波）
    float current_ = 0.0f;            // 电机电流 [A]
    float temperature_ = 0.0f;        // 电机温度 [℃]
    float target_angle_ = 0.0f;       // 位置环目标角度 [deg]
    uint8_t motor_enable_ = 0;        // 电机使能，关闭时不更新 PID，直接输出 0
    uint8_t motor_valid_ = 0;         // 电机通信有效，收到首帧反馈后置 1，失联后置 0

private:
    // —— 回调 ——
    static void decodeCallback(void* device);   // 替代 DecodeDJIMotor
    static void loseCallback(void* device);     // 替代 DJIMotor_Lose
    static void timCallback(void* device);      // 替代 DJIMotor_TimHandler

    // —— 内部方法 ——
    static CAN* getGroup(DJIMotor* instance);   // 替代 DJIMotor_Get_Group
    void currentCommand();                      // 替代 DJIM_Current_Command
    void update();                              // 替代 DJIMotor_Update

    // —— 内部机制 ——
    CAN djim_can_;                              // CAN 实例
    FeedbackRaw feedback_raw_ = {};             // 反馈原始数据
    PID pid_angle_;                             // 位置环 PID
    PID pid_velocity_;                          // 速度环 PID
    PID pid_current_;                           // 电流内环 PID（current_loop_enable_ 为 1 时生效）
    uint8_t current_loop_enable_ = 0;           // 电流内环使能
    Daemon daemon_lose_;                        // 失联守护实例

    DJIMotor_Type motor_type_;                  // 电机型号
    uint8_t motor_id_;                          // 电机 ID
    uint8_t direction_;                         // 方向
    float reduction_ratio_ = 1.0f;              // 减速比

    float angle_offset_ = 0.0f;                 // 角度偏移量 [deg]（offset = -initial_angle）
    float target_velocity_ = 0.0f;              // 速度环目标速度 [deg/s]
    float target_current_ = 0.0f;               // 发送给电调的目标电流 [A]
    float velocity_feedforward_ = 0.0f;         // 速度前馈 [deg/s]
    float current_feedforward_ = 0.0f;          // 力矩前馈 [A]

    PidMode pid_mode_ = PidMode::Current;       // PID 控制模式
    uint16_t ecd_last_ = 0;                     // 上次编码器值，用于增量式位置
    uint8_t pos_freq_div_ = 0;                  // 位置环分频
    uint8_t div_cnt_ = 0;                       // 分频计数器
    uint8_t* command_ptr_ = nullptr;            // 指向 CAN 发送缓冲的指针

    // —— 类级注册表（替代文件级 static）——
    static DJIMotor* instances_[DJIM_MAX_INSTANCE];   // 电机实例注册表
    static uint8_t idx_;                              // 已注册电机数
    static CAN tx_group_pool_[DJIM_MAX_GROUP];        // 发送组 CAN 实例池
    static uint8_t group_idx_;                        // 已注册发送组数
    static TIM tim_;                                  // PID 更新定时器
};
