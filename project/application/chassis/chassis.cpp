/**
 * @file    chassis.cpp
 * @brief   底盘应用
 *
 * 控制逻辑(状态机 / 限幅)沿旧 C 版; 轮组为 X 型全向轮, 使用标准正/逆运动学。
 * PID 数值照抄旧 C, 量纲换算见 pid_port.h。
 *
 * 说明: 旧固件用麦轮公式在全向轮上也能动, 是因为两者平移项同构、只差 √2 幅值;
 * 这里按全向几何写正逆解, 旋转臂长用 OMNI_WHEEL_CHASSIC_RADIUS。
 */
#include "chassis.h"
#include "robot_def.h"
#include "robot_cmd.h"

#include "dji_motor.h"
#include "super_cap.h"
#include "pid_port.h"
#include "bsp_dwt.h"
#include "arm_math.h"

/* chassis_cmd_recv 等的 extern 声明在 robot_cmd.h */

#ifndef DEGREE_2_RAD
#define DEGREE_2_RAD 0.01745329252f
#endif

#ifndef PERIMETER_WHEEL
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI)
#endif

/* X 型全向: 45° 投影系数 √2/2; 旋转臂长 = 轮心到几何中心距离(mm)→指令域 */
#define OMNI_S           0.70710678118f
#define OMNI_R_CMD       ((OMNI_WHEEL_CHASSIC_RADIUS) * DEGREE_2_RAD)

static float sin_theta, cos_theta;

static SuperCap cap;
static uint16_t power_data;
static DJIMotor motor_lf, motor_rf, motor_lb, motor_rb;

Chassis_Upload_Data_s chassis_feedback_data;

/* 底盘系目标速度(云台系旋过来后) / 四轮目标速度 / 由轮速正解得到的估计速度 */
static float chassis_vx, chassis_vy;
static float vt_lf, vt_rf, vt_lb, vt_rb;
static float est_vx, est_vy, est_wz;

/* 速度环直出电调(电调自带电流环)。去掉软件内环后前向增益≈×3,
   Kp 10→3.5 对齐原等效; MaxOut=15000; 浅 Ki。 */
static const PidPort kChassisSpeedPid = {
    .Kp = 3.5f,
    .Ki = 0.1f,
    .Kd = 0,
    .IntegralLimit = 3000,
    .MaxOut = 15000,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
};

/* 底盘已关电流环; 配置保留。 */
static const PidPort kChassisCurrentPid = {
    .Kp = 0.5f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .IntegralLimit = 3000,
    .MaxOut = 15000.0f * PID_SCALE_M3508,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
};

void ChassisInit()
{
    // 指定初始化器必须按声明顺序(can_handle, motor_id, ..., reduction_ratio, pid_velocity, ...)
    DJIMotor::Config chassis_motor_config = {
        .can_handle = &hcan1,
        .motor_type = DJIMotor_3508,
        // 旧 C 版 measure.speed_aps = 6.0 * raw_rpm, 是**转子** deg/s 且不除减速比;
        // 对方 velocity_ = DJIM_VELOCITY(rpm) / reduction_ratio_, 故这里填 1.0f 才能与
        // 旧速度参考值(遥控/键盘给出的转子 deg/s)同量纲。底盘无角度环, 填 1.0f 不影响其它逻辑。
        .reduction_ratio = 1.0f,
        // 3508 速度环由 MOTOR 任务 5 分频以 200Hz(5ms) 调用(与旧固件 motor_task.c 一致), 故 period = 5
        .pid_velocity = pidPort(kChassisSpeedPid, PID_SCALE_M3508, 5),
        .pid_current = pidPort(kChassisCurrentPid, 1.0f, 5),
        .current_loop_enable = 0,
    };

    chassis_motor_config.motor_id = 4;
    chassis_motor_config.direction = DJIM_DIRECTION_REVERT;
    motor_lf.init(chassis_motor_config);

    chassis_motor_config.motor_id = 3;
    chassis_motor_config.direction = DJIM_DIRECTION_NORMAL;
    motor_rf.init(chassis_motor_config);

    chassis_motor_config.motor_id = 1;
    chassis_motor_config.direction = DJIM_DIRECTION_REVERT;
    motor_lb.init(chassis_motor_config);

    chassis_motor_config.motor_id = 2;
    chassis_motor_config.direction = DJIM_DIRECTION_NORMAL;
    motor_rb.init(chassis_motor_config);

    SuperCap::Config capconfig = {
        .can_handle = &hcan1,
        .rx_id = 0x311,
        .tx_id = 0x310,
    };
    cap.init(capconfig);
}

static void ChassisStateSet()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    {
        motor_lf.setEnable(0);
        motor_rf.setEnable(0);
        motor_lb.setEnable(0);
        motor_rb.setEnable(0);
    }
    else
    {
        motor_lf.setEnable(1);
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
 * X 型全向逆解: 底盘系 (vx, vy, wz) → 四轮线速度指令
 *
 *   v_lf = S*( vx - vy) - wz*R
 *   v_rf = S*( vx + vy) + wz*R
 *   v_lb = S*( vx + vy) - wz*R
 *   v_rb = S*( vx - vy) + wz*R
 *
 * S=√2/2, R=OMNI_WHEEL_CHASSIC_RADIUS(指令域)。左右轮 direction 仍为左反右正。
 */
static void OmniInverse(float vx, float vy, float wz,
                        float *lf, float *rf, float *lb, float *rb)
{
    const float wx = OMNI_S * (vx - vy);
    const float wy = OMNI_S * (vx + vy);
    const float wr = wz * OMNI_R_CMD;
    *lf = wx - wr;
    *rf = wy + wr;
    *lb = wy - wr;
    *rb = wx + wr;
}

/**
 * X 型全向正解: 四轮速度 → 底盘系 (vx, vy, wz)
 * 与 OmniInverse 互逆(最小二乘意义下对 4 轮冗余取均值)。
 */
static void OmniForward(float lf, float rf, float lb, float rb,
                        float *vx, float *vy, float *wz)
{
    /* vx = (lf+rf+lb+rb)/(2√2),  vy = (-lf+rf+lb-rb)/(2√2)
       因 1/(2S)=√2,  1/(2√2)=S/2?  由逆解:
         lf+rb = 2S(vx-vy),  rf+lb = 2S(vx+vy)
         vx = [(lf+rb)+(rf+lb)]/(4S), vy = [-(lf+rb)+(rf+lb)]/(4S)
       4S = 2√2,  1/(4S) = 1/(2√2) = √2/4 = 0.35355339 */
    const float inv_4s = 0.35355339059f;
    *vx = (lf + rf + lb + rb) * inv_4s;
    *vy = (-lf + rf + lb - rb) * inv_4s;
    /* wz: (rb-lf)/(2R) 与 (rf-lb)/(2R) 平均 */
    *wz = ((rb - lf) + (rf - lb)) * (0.25f / OMNI_R_CMD);
}

/* 云台系速度旋到底盘系, 再逆解到四轮 */
static void OmniCalculate()
{
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    OmniInverse(chassis_vx, chassis_vy, chassis_cmd_recv.wz,
                &vt_lf, &vt_rf, &vt_lb, &vt_rb);
}

/* 用电机反馈速度做正解, 得到当前底盘系估计速度(调试用) */
static void OmniEstimateFromMotors()
{
    OmniForward(motor_lf.velocity_, motor_rf.velocity_,
                motor_lb.velocity_, motor_rb.velocity_,
                &est_vx, &est_vy, &est_wz);
}

static void LimitChassisOutput()
{
    if (cap.cap_msg_.vol < 24 && cap.cap_msg_.vol > 13)
        chassis_feedback_data.power_flag = 1;
    else
        chassis_feedback_data.power_flag = 0;

    motor_lf.setVelocity(vt_lf);
    motor_rf.setVelocity(vt_rf);
    motor_lb.setVelocity(vt_lb);
    motor_rb.setVelocity(vt_rb);
}

void ChassisTask()
{
    ChassisStateSet();
    OmniCalculate();
    LimitChassisOutput();
    OmniEstimateFromMotors();
    SendPowerData();
    cap.send((uint8_t *)&power_data, sizeof(power_data));
}
