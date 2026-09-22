/**
 * @file    mi_motor.h
 * @brief   小米（MI / CyberGear）微电机（C → C++）
 * @note    由 Modules/motor/MImotor/mi_motor.c 移植，bsp 换成对方的 CAN 类（project/bsp/bsp_can）：
 *            CANInstance → CAN、CANRegister → CAN::init、CANTransmit → CAN::transmit、
 *            CANAddEXFilter → CAN::Config::ext_flag（32 位掩码滤波器 + fifoCallback 的 EXT 分发，
 *            MI 电机的反馈是扩展帧、通信类型 2）。
 *          协议常量、扩展 ID 打包（motor_id:8 | data:16 | mode:5，低 8 位是硬编码的
 *          MI_EXT_ID_MOTOR_FIELD=127）、报文打包、反馈解包、故障状态(motor_state_e)/运行模式
 *          (motor_mode_state_e)/功能码(motor_index_e) 枚举，全部逐行照搬原实现，逻辑/常量/报文格式未改。
 *          发送结构照搬原实现"把要发的帧填进静态缓冲 + 由 MiMotorControl() 周期(100Hz)重发"：
 *            原 mi_sender_assignment[0]（共享发送实例）+ sender_enable_flag
 *              → 每个实例自带一份待发帧：can_.tx_conf_.ExtId + can_.tx_buff_ + tx_pending_；
 *            原文件级 mi_motor_instance[]/idx
 *              → 类静态注册表 mi_motor_instance_[]/idx_（init() 时登记）。
 *          MiMotorControl() 需由 1kHz 的 MOTOR 任务按 cnt%10==0 调用（= 原 100Hz）。
 *          禁堆、禁异常、静态实例。
 */
#pragma once

#include <stdint.h>
#include "bsp_can.h"
#include "pid.h" /* CalMiMotorTorque 的三环级联用对方 PID 类（原 PIDInstance） */

#define MI_MOTOR_CNT 12 // 最多注册的 MI 电机实例数（原 mi_motor_instance[] 的长度）

/* Private defines -----------------------------------------------------------*/
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define IQ_REF_MIN -27.0f
#define IQ_REF_MAX 27.0f
#define SPD_REF_MIN -30.0f
#define SPD_REF_MAX 30.0f
#define LIMIT_TORQUE_MIN 0.0f
#define LIMIT_TORQUE_MAX 12.0f
#define CUR_FILT_GAIN_MIN 0.0f
#define CUR_FILT_GAIN_MAX 1.0f
#define LIMIT_SPD_MIN 0.0f
#define LIMIT_SPD_MAX 30.0f
#define LIMIT_CUR_MIN 0.0f
#define LIMIT_CUR_MAX 27.0f

/* 主机 → 电机报文的扩展 ID 低 8 位（原 EXT_ID_t.motor_id 域）：原实现在所有主机 → 电机报文里
 * 都硬编码写成 127（0x7F），并未使用电机自身 ID。本移植保持该行为，宏名沿用上一版，便于统一修改。 */
#define MI_EXT_ID_MOTOR_FIELD 127

typedef enum
{
    OK                 = 0, //无故障
    BAT_LOW_ERR        = 1, //欠压故障
    OVER_CURRENT_ERR   = 2, //过流
    OVER_TEMP_ERR      = 3, //过温
    MAGNETIC_ERR       = 4, //磁编码故障
    HALL_ERR_ERR       = 5, //HALL编码故障
    NO_CALIBRATION_ERR = 6  //未标定
} motor_state_e; //电机状态（故障信息）

typedef enum
{
    CONTROL_MODE  = 0, //运控模式
    LOCATION_MODE = 1, //位置模式
    SPEED_MODE    = 2, //速度模式
    CURRENT_MODE  = 3  //电流模式
} motor_run_mode_e; //电机运行模式

typedef enum
{
    IQ_REF        = 0X7006, //电流模式Iq指令
    SPD_REF       = 0X700A, //转速模式转速指令
    LIMIT_TORQUE  = 0X700B, //转矩限制
    CUR_KP        = 0X7010, //电流的 Kp
    CUR_KI        = 0X7011, //电流的 Ki
    CUR_FILT_GAIN = 0X7014, //电流滤波系数filt_gain
    LOC_REF       = 0X7016, //位置模式角度指令
    LIMIT_SPD     = 0X7017, //位置模式速度设置
    LIMIT_CUR     = 0X7018  //速度位置模式电流设置
} motor_index_e; //电机功能码

typedef enum
{
    RESET_MODE = 0, //Reset模式[复位]
    CALI_MODE  = 1, //Cali 模式[标定]
    RUN_MODE   = 2  //Motor模式[运行]
} motor_mode_state_e; //电机模式状态

typedef struct
{
    float angle;       //(rad)
    float speed;       //(rad/s)
    float torque;      //(N*m)
    float temperature; //(℃)
} MI_Motor_Measure_s; //小米电机测量值（原 mi_motor.h 原名保留）

/* ---------------- 电机控制器相关类型（原 Modules/motor/motor_def.h 对应部分）----------------
 * CalMiMotorTorque() 用到；只把原 PIDInstance 换成对方框架的 PID 类，字段名/语义不变。
 * 注意：冻结的 MIMotor::Config 里没有这些字段（原 MIMotorInit 从 Motor_Init_Config_s 取），
 *       需要时用 MIMotor::setControllerConfig() 配置。 */
typedef enum
{
    OPEN_LOOP    = 0b0000,
    CURRENT_LOOP = 0b0001,
    SPEED_LOOP   = 0b0010,
    ANGLE_LOOP   = 0b0100,

    // only for checking
    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP   = 0b0110,
    ALL_THREE_LOOP         = 0b0111,
} Closeloop_Type_e; //闭环类型,如果需要多个闭环,则使用或运算

typedef enum
{
    FEEDFORWARD_NONE              = 0b00,
    CURRENT_FEEDFORWARD           = 0b01,
    SPEED_FEEDFORWARD             = 0b10,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e; //前馈类型

typedef enum
{
    MOTOR_FEED = 0,
    OTHER_FEED,
} Feedback_Source_e; //反馈值来源

typedef enum
{
    MOTOR_DIRECTION_NORMAL  = 0,
    MOTOR_DIRECTION_REVERSE = 1 //电机反转
} Motor_Reverse_Flag_e; //电机正反转标志

typedef enum
{
    FEEDBACK_DIRECTION_NORMAL  = 0,
    FEEDBACK_DIRECTION_REVERSE = 1
} Feedback_Reverse_Flag_e; //反馈量正反标志

typedef struct
{
    Closeloop_Type_e outer_loop_type;              // 最外层的闭环,未设置时默认为最高级的闭环
    Closeloop_Type_e close_loop_type;              // 使用几个闭环(串级)
    Motor_Reverse_Flag_e motor_reverse_flag;       // 是否反转
    Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
    Feedback_Source_e angle_feedback_source;       // 角度反馈类型
    Feedback_Source_e speed_feedback_source;       // 速度反馈类型
    Feedfoward_Type_e feedforward_flag;            // 前馈标志
} Motor_Control_Setting_s; //电机控制设置（原 motor_def.h 同名结构体）

typedef struct
{
    float* other_angle_feedback_ptr; // 其他角度反馈来源的反馈数据指针
    float* other_speed_feedback_ptr;
    float* speed_feedforward_ptr;
    float* current_feedforward_ptr;

    PID current_PID; // 原 PIDInstance current_PID（本移植未初始化，与原实现一致）
    PID speed_PID;
    PID angle_PID;

    float pid_ref; // 将会作为每个环的输入和输出顺次通过串级闭环
} Motor_Controller_s; //电机控制器（原 motor_def.h 同名结构体）

typedef struct
{
    float* other_angle_feedback_ptr; // 角度反馈数据指针,注意电机使用total_angle
    float* other_speed_feedback_ptr; // 速度反馈数据指针,单位为angle per sec

    float* speed_feedforward_ptr;   // 速度前馈数据指针
    float* current_feedforward_ptr; // 电流前馈数据指针

    PID::Config current_PID; // 电流环（原 PID_Init_Config_s）
    PID::Config speed_PID;   // 速度环
    PID::Config angle_PID;   // 位置环/角度环
} Motor_Controller_Init_s; //原 Motor_Controller_Init_s，PID 配置改用对方 PID 类

/**********************Functions**************************/

class MIMotor {
public:
    /// 初始化配置（冻结：application/gimbal/gimbal.cpp 在用，签名不可改）
    struct Config {
        CAN_HandleTypeDef* can_handle; // CAN 句柄
        uint8_t motor_id;              // 电机 ID（同时用作接收匹配 ID 与状态标识）
    };

    /* ---- 冻结 API（application/gimbal/gimbal.cpp 调用，签名保持不变）---- */
    void init(const Config& config);                          // 原 MIMotorInit（禁堆：实例由应用层静态持有）
    void setEnable(uint8_t en);                               // 原 MIMotorEnable（通信类型 3）；en=0 → 停止帧
    void stop();                                              // 原 MIMotorInstancestop（通信类型 4）
    void setMechPositionToZero();                             // 原 MIMotorInstanceetMechPositionToZero（通信类型 6）
    void locationControl(float location, float kp, float kd); // 原 MI_motor_LocationControl（通信类型 1，rad）

    /* ---- 原模块公开的其它功能（语义/报文格式/常量完全照搬）---- */
    void control(float torque, float mech_position, float speed, float kp, float kd); // 原 MI_motor_Control（通信类型 1）
    void torqueControl(float torque);                                                 // 原 MI_motor_TorqueControl
    void speedControl(float speed, float kd);                                         // 原 MIMotorInstancepeedControl（原名如此）
    void getID();                                                                     // 原 MI_motor_GetID（通信类型 0，立即发送，超时 5ms）
    void changeID(uint8_t now_id, uint8_t target_id);                                 // 原 MI_motor_ChangeID（通信类型 7，立即发送）
    void readParam(uint16_t index);                                                   // 原 MI_motor_ReadParam（通信类型 17，立即发送）
    void modeSwitch(uint8_t run_mode);                                                // 原 MIMotorModeSwitch（通信类型 18，功能码 0x7005）
    void writeParam(uint16_t index, float param);                                     // 原 MI_motor_WritePram（通信类型 18，立即发送）
    void setPid(float location_kp, float limit_speed, float speed_kp, float speed_ki); // 原 MIMotorSetPid
    void setRef(float location_ref);                                                  // 原 MiMotorSetRef（写功能码 0x7016）

    /// 原 MIMotorInit 里从 Motor_Init_Config_s 取的控制器设置/PID/反馈指针（冻结的 Config 装不下，单独入口）
    void setControllerConfig(const Motor_Control_Setting_s& setting, const Motor_Controller_Init_s& controller_init);

    /* ---- 原结构体里的公开状态 ---- */
    float angle_ = 0.0f;       // 机械角度 [rad]（原 motor->measure.angle）
    float speed_ = 0.0f;       // 转速 [rad/s]（原 motor->measure.speed）
    float torque_ = 0.0f;      // 力矩 [N*m]（原 motor->measure.torque）
    float temperature_ = 0.0f; // 温度 [℃]（原 motor->measure.temperature）
    uint8_t motor_id_ = 0;     // 电机 ID（原 motor->motor_id）
    uint8_t valid_ = 0;        // 收到过反馈帧（原实现无此标志，移植时新增，供上层判通信）

    MI_Motor_Measure_s measure_ = {};                  // 原 motor->measure（与上面 4 个公开字段同源）
    motor_state_e motor_state_ = OK;                   // 原 motor->motor_state
    motor_mode_state_e motor_mode_state_ = RESET_MODE; // 原 motor->motor_mode_state
    Motor_Control_Setting_s motor_settings_ = {};      // 原 motor->motor_settings（正反转,闭环类型等）
    Motor_Controller_s motor_controller_ = {};         // 原 motor->motor_controller

private:
    static void decodeCallback(void* device);                                    // 原 DecodeMiMotor（通信类型 2 反馈解包）
    static uint32_t packExtId(uint8_t mode, uint16_t data, uint8_t motor_field); // 原 EXT_ID_t 位域打包
    void clearTxBuff();                                                          // 原 for(i<8) tx_buff[i]=0
    void stageFrame(uint8_t mode, uint16_t data, uint8_t motor_field = MI_EXT_ID_MOTOR_FIELD); // 原"填进共享发送实例"→本实例待发帧
    void sendFrameNow(uint8_t mode, uint16_t data, uint8_t motor_field, float timeout);        // 一次性帧：立即发送（不进周期重发）

    CAN can_;                // 原 motor_can_instace（收）+ mi_sender_assignment[0]（发）
    uint8_t tx_pending_ = 0; // 原 sender_enable_flag：防止发送空帧

    // —— 类级注册表（替代原文件级 static mi_motor_instance[]/idx）——
    static MIMotor* mi_motor_instance_[MI_MOTOR_CNT]; // 会在 MiMotorControl() 中遍历重发
    static uint8_t idx_;                              // 已注册实例数

    friend void MiMotorControl(void);
    friend float CalMiMotorTorque(void);
};

/**
 * @brief 原 MiMotorControl()：遍历所有已注册实例, 按各实例当前待发帧周期重发（保证 MI 电机不超时掉使能）
 * @note  在 1kHz 的 MOTOR 任务里按 cnt%10==0 调用 = 原来的 100Hz；原实现是
 *        CANTransmit(&mi_sender_assignment[0], 1)（发送缓冲是共享的, 本移植改为每个实例一份）
 */
void MiMotorControl(void);

/**
 * @brief 原 CalMiMotorTorque()：对注册表里第 0 个实例做三环级联计算
 * @retval 最终输出（电机控制量）
 */
float CalMiMotorTorque(void);
