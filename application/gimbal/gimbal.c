#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "mi_motor.h"
#include "bmi088.h"

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;
static MIMotorInstance *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static uint8_t motor_init=0;

/* ---------------- pitch 坐标系映射 ----------------
 * MI 电机的机械零位每次上电都会被重设(见 mi_motor.c:202 "通信类型6 ... 掉电丢失"),
 * 也就是说"电机角度坐标系"的 0 点是"上电瞬间的姿态"; 而 PITCH_MIN/MAX_ANGLE 是相对重力的
 * 绝对角度(robot_def.h 注释也写明"注意反馈如果是陀螺仪,则填写陀螺仪的角度")。
 * 两者直接混用会导致: 上电后软限位把指令夹到某个极限, 云台先转过去, 之后遥控器再也打不到另一侧
 * (表现为"开机抬高到某个角度, 并把它当成下限位")。
 * 解决: 上电首次使能时记录一次偏置, 之后把 IMU 坐标系的指令映射到电机坐标系。 */
#define PITCH_IMU_TO_MOTOR_SIGN (+1.0f) /* 若上电后 pitch 朝反方向冲/顶死, 改成 -1.0f */
static float pitch_frame_offset_rad = 0.0f; /* 电机角度 = SIGN * IMU_pitch(rad) + offset */

void GimbalInit()
{
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 40, // 30->40 增强跟踪响应
                .Ki = 20,
                .Kd = 6,// 3->6 增加阻尼抑制震荡
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .CoefA = 7,
                .CoefB = 7,
                .MaxOut = 330,
                // 速度前馈(对角度环而言): Output += FF_Gain * d(角度)/dt。
                // 修复 PIDInit 之前这个值一直是 0(前馈静默失效) —— 这正是"动目标跟不上(存在跟随误差)"的主因之一。
                // 原来填写的是 350, 先在 120 起步(修好后 350 可能过冲/振荡), 稳定后可逐步加到 200~350。
                .FF_Gain = 120.0,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 60,  // 150 -> 60, 抑制锁定微振
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
                // 加速度前馈: Output += FF_Gain * d(gyro)/dt。原值 500 修好后会生效,
                // 但陀螺噪声会被放大成电流抖动, 先置 0; 想进一步加快响应再逐步加(50~150)。
                .FF_Gain = 0.0,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .ext_flag = 1,
        },
        .controller_param_init_config={
            .angle_PID={
                .Kp=15,  // 15
                .Kd=1.0,  
                .FF_Gain = 0.0,
            },
            .speed_PID={
                .Kp=1,
                .Ki=0.01,
                .FF_Gain = 0.0,
            },
        },
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = MIMotorInit(&pitch_config);
    MIMotorEnable(pitch_motor);
    MIMotorInstanceetMechPositionToZero(pitch_motor);

    
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

static void GimbalStateSet()
{
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        MIMotorInstancestop(pitch_motor);
        DJIMotorStop(yaw_motor);
        motor_init=0;
        break;
    case GIMBAL_GYRO_MODE:
        DJIMotorEnable(yaw_motor);
        DJIMotorSetRef(yaw_motor,gimbal_cmd_recv.yaw);
        if(motor_init==0)
        {
            MIMotorEnable(pitch_motor);
            // 清除yaw PID积分,防止停机期间积分windup导致使能瞬间过流
            PIDInstance *ap = &yaw_motor->motor_controller.angle_PID;
            PIDInstance *sp = &yaw_motor->motor_controller.speed_PID;
            ap->ITerm = 0; ap->Iout = 0; ap->Last_ITerm = 0;
            sp->ITerm = 0; sp->Iout = 0; sp->Last_ITerm = 0;
            // 以当前角度初始化pid_ref,避免阶跃
            yaw_motor->motor_controller.pid_ref = gimbal_IMU_data->YawTotalAngle;
            gimbal_feedback_data.init_location = gimbal_IMU_data->Pitch;
            // 记录 pitch 坐标系偏置: 使能瞬间电机角度 与 IMU pitch(重力参考) 的差
            pitch_frame_offset_rad = pitch_motor->measure.angle -
                                     PITCH_IMU_TO_MOTOR_SIGN * (gimbal_IMU_data->Pitch * DEGREE_2_RAD);
            motor_init=1;
        }
        // 把 IMU/重力坐标系的 pitch 指令映射到电机坐标系(上电姿态不再影响软限位的含义)
        MI_motor_LocationControl(pitch_motor,
                                 PITCH_IMU_TO_MOTOR_SIGN * gimbal_cmd_recv.pitch + pitch_frame_offset_rad,
                                 pitch_motor->motor_controller.angle_PID.Kp,
                                 pitch_motor->motor_controller.angle_PID.Kd);
        break;
    default:
        break;
    }
}



static void SendGimbalData()
{
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
}

/* 机器人云台控制核心任务 */
void GimbalTask()
{
    // 获取云台控制数据
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    //云台启停
    GimbalStateSet();
    // 设置反馈数据,主要是imu和yaw的ecd
    SendGimbalData();
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}

DJIMotorInstance* GetYawMotor(void)
{
    return yaw_motor;
}

MIMotorInstance* GetPitchMotor(void)
{
    return pitch_motor;
}