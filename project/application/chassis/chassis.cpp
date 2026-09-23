/**
 * @file    chassis.cpp
 * @brief   底盘应用(由 application/chassis/chassis.c 移植)
 *
 * 移植差异:
 *   1. 旧的 current_PID 电流环在对方框架里由电机模块内部承担
 *   2. message_center 的 SubRegister/PubRegister/SubGetMessage/PubPushMessage 全部删除,
 *      改为直接访问全局实例(规约 §1.5): chassis_cmd_recv 由 robot_cmd.cpp 定义,
 *      chassis_feedback_data 在本文件定义
 *   3. 底层实例类型: DJIMotorInstance* → DJIMotor 静态实例; SuperCapInstance* → SuperCap 静态实例
 *   4. 控制逻辑(状态机 / 麦轮逆解 / 限幅判断)与 chassis.c 逐行一致
 *   5. PID 配置数值照抄旧 C, 输出量纲(A vs 电流计数)/积分时间(ms vs s)/输出限幅的换算
 *      统一交给 application/pid_port.h 的 pidPort()
 */
#include "chassis.h"
#include "robot_def.h"
#include "robot_cmd.h"

#include "dji_motor.h"
#include "super_cap.h"
#include "pid_port.h"
#include "bsp_dwt.h"
#include "arm_math.h"

/* chassis_cmd_recv / gimbal_cmd_recv / shoot_cmd_recv 的 extern 声明在 robot_cmd.h(规约 §1.5) */

/* 旧 general_def.h 里的角度/弧度转换系数, 同值搬过来 */
#ifndef DEGREE_2_RAD
#define DEGREE_2_RAD 0.01745329252f // pi/180
#endif

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)   // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f) // 半轮距
#ifndef PERIMETER_WHEEL
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#endif

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
static float sin_theta, cos_theta; // 底盘速度解算用

static float chassis_rotate_buff;

static SuperCap cap;                                                 // 超级电容
static uint16_t power_data;
static DJIMotor motor_lf, motor_rf, motor_lb, motor_rb;              // left right forward back
/* 用于自旋变速策略的时间变量 */
static float t;

/* 底盘回传的反馈数据(定义在本文件, extern 声明在 chassis.h) */
Chassis_Upload_Data_s chassis_feedback_data;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

/* 旧 C 的速度环配置原样照抄(数值一字不改), 量纲/积分单位/输出限幅的换算见 pid_port.h */
static const PidPort kChassisSpeedPid = {
    .Kp = 10, // 4.5
    .Ki = 0,  // 0
    .Kd = 0,  // 0
    .IntegralLimit = 3000,
    .MaxOut = 12000,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
};

/* 旧 current_PID（底盘）: Kp=0.5, Ki=0, Kd=0, IntegralLimit=3000, MaxOut=15000(ESC 计数)。
   内环在安培域跑，故只有 MaxOut 要换算: 15000*PID_SCALE_M3508 = 18.31A。 */
static const PidPort kChassisCurrentPid = {
    .Kp = 0.5f, // 0.4
    .Ki = 0.0f,
    .Kd = 0.0f,
    .IntegralLimit = 3000,
    .MaxOut = 15000.0f * PID_SCALE_M3508,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
};

void ChassisInit()
{
    // 指定初始化器必须按声明顺序: can_handle, motor_id, motor_type, direction,
    //                               reduction_ratio, pid_angle, pid_velocity, ...
    DJIMotor::Config chassis_motor_config = {
        .can_handle = &hcan1,
        .motor_type = DJIMotor_3508,
        // 旧 C 版 measure.speed_aps = 6.0 * raw_rpm, 是**转子** deg/s 且不除减速比;
        // 对方 velocity_ = DJIM_VELOCITY(rpm) / reduction_ratio_, 故这里填 1.0f 才能与
        // 旧速度参考值(遥控/键盘给出的转子 deg/s)同量纲。底盘无角度环, 填 1.0f 不影响其它逻辑。
        .reduction_ratio = 1.0f,
        // 3508 速度环由 MOTOR 任务 5 分频以 200Hz(5ms) 调用(与旧固件 motor_task.c 一致), 故 period = 5
        .pid_velocity = pidPort(kChassisSpeedPid, PID_SCALE_M3508, 5),
        /* 旧底盘 close_loop_type = SPEED_LOOP | CURRENT_LOOP → 后面还串了一级电流内环。
           内环参考/反馈都在安培域, 故 output_scale = 1.0, period 与速度环相同(5ms)。 */
        .pid_current = pidPort(kChassisCurrentPid, 1.0f, 5),
        .current_loop_enable = 1,
    };

    chassis_motor_config.motor_id = 4;
    chassis_motor_config.direction = DJIM_DIRECTION_REVERT; // 原 MOTOR_DIRECTION_REVERSE
    motor_lf.init(chassis_motor_config);

    chassis_motor_config.motor_id = 3;
    chassis_motor_config.direction = DJIM_DIRECTION_NORMAL; // 原 MOTOR_DIRECTION_NORMAL
    motor_rf.init(chassis_motor_config);

    chassis_motor_config.motor_id = 1;
    chassis_motor_config.direction = DJIM_DIRECTION_REVERT; // 原 MOTOR_DIRECTION_REVERSE
    motor_lb.init(chassis_motor_config);

    chassis_motor_config.motor_id = 2;
    chassis_motor_config.direction = DJIM_DIRECTION_NORMAL; // 原 MOTOR_DIRECTION_NORMAL
    motor_rb.init(chassis_motor_config);

    SuperCap::Config capconfig = {
        .can_handle = &hcan1,
        .rx_id = 0x311,
        .tx_id = 0x310,
    };
    cap.init(capconfig);
}
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

static void ChassisStateSet()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        motor_lf.setEnable(0); // 原 DJIMotorStop
        motor_rf.setEnable(0);
        motor_lb.setEnable(0);
        motor_rb.setEnable(0);
    }
    else
    { // 正常工作
        motor_lf.setEnable(1); // 原 DJIMotorEnable
        motor_rf.setEnable(1);
        motor_lb.setEnable(1);
        motor_rb.setEnable(1);
    }
}

static void SendPowerData()
{
    power_data = chassis_cmd_recv.power_limit;
}

/**
 * @brief 计算每个底盘电机的输出,正运动学解算
 *
 */
static void MecanumCalculate()
{
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    vt_lf = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_lb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx - chassis_vy + chassis_cmd_recv.wz * RB_CENTER;
    vt_rf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * RF_CENTER;
}

static void OmniCalculate()
{
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 全向轮 X 型布局逆运动学: v_i = vx*cos(α_i) + vy*sin(α_i) + R*ω
    // 轮子角度: LF=45°, RF=135°, LB=225°, RB=315°
    // 乘以 √2 使输出幅值与麦轮解算一致
    vt_lf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * OMNI_WHEEL_CHASSIC_RADIUS;
    vt_rf = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * OMNI_WHEEL_CHASSIC_RADIUS;
    vt_lb = -chassis_vx - chassis_vy + chassis_cmd_recv.wz * OMNI_WHEEL_CHASSIC_RADIUS;
    vt_rb = chassis_vx - chassis_vy + chassis_cmd_recv.wz * OMNI_WHEEL_CHASSIC_RADIUS;
}
/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{

    if (cap.cap_msg_.vol < 24 && cap.cap_msg_.vol > 13)
    {
        chassis_feedback_data.power_flag = 1;
    }
    else
    {
        chassis_feedback_data.power_flag = 0;
    }

    // 完成功率限制后进行电机参考输入设定
    motor_lf.setVelocity(vt_lf); // 原 DJIMotorSetRef(速度环, 单位 deg/s)
    motor_rf.setVelocity(vt_rf);
    motor_lb.setVelocity(vt_lb);
    motor_rb.setVelocity(vt_rb);
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    ChassisStateSet();
    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();
    // OmniCalculate();
    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();
    SendPowerData();
    cap.send((uint8_t *)&power_data, sizeof(power_data)); // 原 SuperCapSend(cap, (uint8_t*)&power_data)
}
