#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "mi_motor.h"
#include "bmi088.h"
#include "bsp_log.h"

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;
static MIMotorInstance *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static uint8_t motor_init=0;

/* ---- pitch 上电标定/测量开关 ----
 * 1 = 测试固件: 不设机械零位、不使能也不控制 pitch 电机(保持自由停止),
 *     仅用于通过黑匣子读取 MI 电机上报角度, 验证"机械零位掉电后回到的基准是否固定"。
 * 0 = 正常工作固件。测完必须改回 0！ */
#define PITCH_BRINGUP_TEST 1
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
                // 实测(黑匣子): 目标 17°/s 时稳态误差 3.44°(等效滞后 197ms), 电流仅用 42% -> 等效速度增益偏低
                // 40 -> 80: 等效增益(电流/度误差)翻倍, 预测误差与滞后减半; 若电流接近 70% 或出现振荡就回调
                .Kp = 80, // 30->40->80 增强跟踪响应
                // Ki 是"消除匀速目标跟随误差"的那一项: 20 时每拍只加 20*e*dt, 把跟踪所需偏置补齐约需 1.7~2s
                // (实测现象就是"慢一点才跟上"); 提到 60 后约 0.6s 补足。
                .Ki = 60,
                .Kd = 6,// 3->6 增加阻尼抑制震荡
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                // 给积分足够的偏置权限: 该输出经速度环(×约50)后相当于电流偏置, 250 -> 约 12500 计数
                .IntegralLimit = 250,
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
                // 稳态匀速跟随时需要积分提供偏置电流, 原 3000 偏紧
                .IntegralLimit = 6000,
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
#if PITCH_BRINGUP_TEST
    MIMotorInstancestop(pitch_motor); // 测试: 自由状态(可被重力/手移动), 仅读取上报角度
#else
    MIMotorEnable(pitch_motor);

    /* ---- pitch 机械零位策略 ----
     * MI 电机的机械零位(通信类型6)"掉电丢失"(见 mi_motor.c:202), 所以必须在合适的时候重设。
     * 但"每次 MCU 上电都重设"是错的: 烧录/看门狗/引脚复位只复位 MCU, 电机一直带电、并保持上一次的位置,
     * 此时把当前姿态设成 0, 行程区间就会随复位瞬间的姿态整体漂移 —— 表现就是"抬到某个角度并把那里当下限位,
     * 遥控器再也打不到另一侧"。
     * 正确做法: 只有"整机断电后重新上电"(POR)才重设零位; 其余复位保持电机原有零位(此时它依然有效)。
     * 因此: 换电池后请先把云台摆到参考姿态(一般让它靠重力落到机械下限)再上电; 烧录/复位则完全不影响行程。
     * 注: 本函数在 BSPIWDGLogResetCause() 清复位标志之前执行, 这里读到的是真实的上次复位原因。 */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
    {
        MIMotorInstanceetMechPositionToZero(pitch_motor);
        LOGINFO("[gimbal] POR reset detected: MI pitch mechanical zero re-established");
    }
    else
    {
        LOGINFO("[gimbal] non-POR reset (flash/soft/wdg/pin): keep MI pitch mechanical zero");
    }
#endif

    
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
#if PITCH_BRINGUP_TEST
        MIMotorInstancestop(pitch_motor); // 测试: 保持自由, 绝不用未标定的限位去驱动机构
#else
        MI_motor_LocationControl(pitch_motor,gimbal_cmd_recv.pitch,pitch_motor->motor_controller.angle_PID.Kp,pitch_motor->motor_controller.angle_PID.Kd);
#endif
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
            motor_init=1;
        }
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