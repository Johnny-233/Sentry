// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "controller.h"
#include "mi_motor.h"
#include "gimbal.h"
#include "gimbal_algorithm.h"
#include "referee_UI.h"
#include "referee_task.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include <math.h>

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Minipc_Recv_s *minipc_recv_data; // 视觉接收数据指针,初始化时返回
static Minipc_Send_s minipc_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
static DataLebel_t DataLebel;

static uint8_t gimbal_location_init=0;
static uint8_t power_flag;

static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
static float chassis_rotate_buff;
static float chassis_speed_buff;

static PIDInstance chassis_follow_pid; // 底盘跟随模式PID

static cal_round_patrol_t round_patrol;
static cal_mid_round_patrol_t mid_round_patrol;

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    minipc_recv_data = minipcInit(&huart1); // 视觉通信串口
    referee_data= UITaskInit(&huart6,&ui_data);

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_send.pitch = 0;

    // 底盘跟随模式PID初始化
    PID_Init_Config_s chassis_follow_pid_config = {
        .Kp = 50.0f,
        .Ki = 0.0f,
        .Kd = 3.0f,
        .MaxOut = 4000.0f,
        .DeadBand = 1.0f,
        .Improve = PID_Derivative_On_Measurement,
        .IntegralLimit = 0.0f,
    };
    PIDInit(&chassis_follow_pid, &chassis_follow_pid_config);

}


/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    gimbal_fetch_data.offset_diff = gimbal_fetch_data.yaw_motor_single_round_angle - YAW_ALIGN_ANGLE;
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

static void GimbalPitchLimit()
{
    gimbal_cmd_send.gimbal_mode=GIMBAL_GYRO_MODE;
    // 云台软件限位
    if(gimbal_cmd_send.pitch<PITCH_MIN_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    else if (gimbal_cmd_send.pitch>PITCH_MAX_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    else
    gimbal_cmd_send.pitch=gimbal_cmd_send.pitch;
}

/**
 * @brief 判断视觉有没有发信息
 *
 */
static void VisionJudge()
{
    static float target_lost_time = 0.0f;
    static uint8_t target_timer_running = 0;

    /* 小电脑识别到目标时yaw/pitch会有非零数据 */
    if (minipc_recv_data->Vision.yaw != 0.0f || minipc_recv_data->Vision.pitch != 0.0f)
    {
        DataLebel.vision_flag = 1;
        target_timer_running = 0;
    }
    else if (DataLebel.vision_flag == 1)
    {
        /* yaw和pitch同时为0：可能是无目标，也可能是恰好瞄准在目标中心 */
        if (!target_timer_running)
        {
            target_lost_time = DWT_GetTimeline_s();
            target_timer_running = 1;
        }

        if (DWT_GetTimeline_s() - target_lost_time >= 1.0f)
        {
            /* 连续1秒yaw/pitch都为0，确认丢失目标 */
            DataLebel.vision_flag = 0;
            target_timer_running = 0;
        }
    }

    /* 仅当小电脑瞄准锁定目标时才允许开火 */
    if (minipc_recv_data->Vision.can_fire != 0)
    {
        DataLebel.fire_flag = 1;
    }
    else
    {
        DataLebel.fire_flag = 0;
    }
}

static void BasicSet()
{
    CalcOffsetAngle();
    GimbalPitchLimit();
    VisionJudge();
    //发射基本模式设定
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.shoot_rate=8;
    chassis_cmd_send.power_limit=referee_data->GameRobotState.chassis_power_limit;
}


static void GimbalRC()
{
    gimbal_cmd_send.yaw -= 0.003f * (float)rc_data[TEMP].rc.rocker_right_x;
    gimbal_cmd_send.pitch -= 0.00003f * (float)rc_data[TEMP].rc.rocker_right_y;
    gimbal_cmd_send.real_pitch= ((gimbal_fetch_data.gimbal_imu_data.Pitch)-gimbal_fetch_data.init_location)/57.39;
}

void FoundEnermy()
{
    gimbal_cmd_send.autoaim_mode = AUTO_ON;

    /* yaw: 自适应滤波 + 速率限幅 */
    float yaw_err = minipc_recv_data->Vision.yaw;
    float abs_err = fabsf(yaw_err);

    if (abs_err > 0.3f)
    {
        float target_yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle - yaw_err;
        gimbal_cmd_send.yaw = target_yaw;
        gimbal_cmd_send.yaw = Cal_FollowControl_Set_Yaw(gimbal_fetch_data.gimbal_imu_data, gimbal_cmd_send);
    }
    /* |误差| ≤ 0.3°: 死区锁死 */

    /* pitch: 自适应滤波增量式 — 对视觉误差做滤波,保持在编码器坐标系 */
    {
        static float err_filtered = 0;
        static float err_last = 0;
        float raw_err = minipc_recv_data->Vision.pitch;

        if (fabsf(raw_err) > 0.3f)
        {
            // 根据误差变化量自适应选择滤波强度
            float err_delta = raw_err - err_last;
            float alpha;

            if (fabsf(err_delta) > 0.5f)
                alpha = 0.3f;    // 大跳变: 快速跟上
            else if (fabsf(err_delta) > 0.05f)
                alpha = 0.2f;    // 跟踪中: 中等响应
            else
                alpha = 0.1f;    // 微调: 强滤波抑制噪声

            err_filtered = alpha * raw_err + (1.0f - alpha) * err_filtered;
            err_last = raw_err;

            // 增量式控制, 增益可安全设高
            gimbal_cmd_send.pitch += 0.0005f * err_filtered;
        }
        else
        {
            err_last = 0;
        }
    }
    /* |误差| ≤ 0.5°: 死区锁死 */
}


static void ChassisRotateSet()
{
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_send.chassis_mode)
    {
        //底盘跟随模式,将offset_angle量化到最近的90°作为目标(4个正方向)
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        {
            float raw = chassis_cmd_send.offset_angle;
            float snapped;
            // 就近量化到90°的倍数 —— 4个固定正方向: 0°, ±90°, 180°/-180°
            if (raw >= 0.0f)
                snapped = (float)((int)(raw / 90.0f + 0.5f)) * 90.0f;
            else
                snapped = (float)((int)(raw / 90.0f - 0.5f)) * 90.0f;
            chassis_cmd_send.wz = PIDCalculate(&chassis_follow_pid, raw, snapped);
        }
        break;
        case CHASSIS_ROTATE: // 变速小陀螺
            chassis_cmd_send.wz = 4000 * chassis_cmd_send.chassis_rotate_buff;
        break;
        default:
            chassis_cmd_send.wz = 0.0;
        break;
    }
}
static void ChassisRC()
{
    chassis_cmd_send.vx = -30.0f * (float)rc_data[TEMP].rc.rocker_left_y; // _水平方向
    chassis_cmd_send.vy = 30.0f * (float)rc_data[TEMP].rc.rocker_left_x; // 竖直方向

    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    else if(switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
        chassis_cmd_send.chassis_rotate_buff = 1.0f;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
        chassis_cmd_send.chassis_rotate_buff = -1.0f;
    }
    ChassisRotateSet();
}





static void AutoAimSet()
{
    if(DataLebel.aim_flag==1)
    {
        FoundEnermy();
        if(DataLebel.fire_flag==1)
        {
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        }
    }
}

static void ShootRC()
{
    if(rc_data->rc.dial>200)
    {
        shoot_cmd_send.loader_mode=LOAD_BURSTFIRE;
    }
    else if (rc_data->rc.dial<-200)
    {
        shoot_cmd_send.loader_mode=LOAD_REVERSE;
        DataLebel.reverse_flag=1;
    }
    else
    {
        shoot_cmd_send.loader_mode=LOAD_STOP;
        DataLebel.reverse_flag=0;
    }
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    ChassisRC();
    gimbal_cmd_send.autoaim_mode=AUTO_OFF;
    GimbalRC();
    ShootRC();
}
static void GetGimbalInitImu()
{
    if (mid_round_patrol.flag == 0)
    {
        mid_round_patrol.yaw_init = gimbal_fetch_data.gimbal_imu_data.Yaw;
        mid_round_patrol.yaw = mid_round_patrol.yaw_init;
        mid_round_patrol.flag = 1;
    }
}

static void RoundPatrol()
{
    if (round_patrol.flag == 0)
    {
        round_patrol.init_totol_round = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle / 360.0f;
        round_patrol.flag = 1;
    }
    round_patrol.total_round = (gimbal_fetch_data.gimbal_imu_data.YawTotalAngle / 360.0f) - round_patrol.init_totol_round;
    gimbal_cmd_send.yaw += 0.5f;
}

static void MidRoundPatrol()
{
    if (mid_round_patrol.flag == 0)
    {
        mid_round_patrol.yaw_init = gimbal_fetch_data.gimbal_imu_data.Yaw;
        mid_round_patrol.yaw = mid_round_patrol.yaw_init;
        mid_round_patrol.flag = 1;
    }

    float current_relative_angle = gimbal_fetch_data.gimbal_imu_data.Yaw - mid_round_patrol.yaw_init;
    current_relative_angle += 0.15f * mid_round_patrol.direction;

    if (current_relative_angle > 70.0f)
    {
        current_relative_angle = 70.0f;
        mid_round_patrol.direction = -1;
    }
    else if (current_relative_angle < -70.0f)
    {
        current_relative_angle = -70.0f;
        mid_round_patrol.direction = 1;
    }

    gimbal_cmd_send.yaw = current_relative_angle + round_patrol.total_round * 360.0f;
    mid_round_patrol.yaw_total_angle = current_relative_angle;
}

static void Sentry_ChassisAC()
{
    chassis_cmd_send.vx = -minipc_recv_data->Vision.linear_velocity_x * 4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL * 1000.0f;
    chassis_cmd_send.vy = -minipc_recv_data->Vision.linear_velocity_y * 4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL * 1000.0f;

    if (minipc_recv_data->Vision.gimbal_mode != 0)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        if (chassis_fetch_data.power_flag == 1)
        {
            chassis_cmd_send.chassis_rotate_buff = 2;
        }
        else
        {
            chassis_cmd_send.chassis_rotate_buff = 1.0;
        }
    }
    else
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }
}

static void ShootAC()
{
    if (DataLebel.fire_flag == 1)
    {
        shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
    }
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
        DataLebel.reverse_flag = 0;
    }
}

static void Sentry_GimbalAC()
{
    static MIMotorInstance *PP_Motor;
    PP_Motor = GetPitchMotor();
    static float PIT;
    GetGimbalInitImu();

    /* 小电脑无目标 → 自主巡逻 */
    if (DataLebel.vision_flag == 0)
    {
        DataLebel.t_pitch = (float)DWT_GetTimeline_s();
        // 巡逻pitch摆动铺满整个限位窗口: 中心=(MAX+MIN)/2, 幅值=半程, 由宏推出自动适配
        PIT = sinf(10.0f * DataLebel.t_pitch)
                * (PITCH_MAX_ANGLE - PITCH_MIN_ANGLE) * 0.5f
            + (PITCH_MAX_ANGLE + PITCH_MIN_ANGLE) * 0.5f;

        if (DataLebel.ACEntryPoint)
        {
            // 平滑进入: 从当前实际角度按每周期限步长逼近PIT, 避免停机恢复时硬跳/撞限位
            const float entry_step = 0.02f; // rad/周期(200Hz下约4rad/s), 略大于摆扫速率使其能追上
            float cur = PP_Motor->measure.angle;
            float err = PIT - cur;
            if (fabsf(err) <= entry_step)
            {
                gimbal_cmd_send.pitch = PIT; // 已跟上目标, 退出平滑进入稳态
                DataLebel.ACEntryPoint = 0;
            }
            else
            {
                gimbal_cmd_send.pitch = cur + copysignf(entry_step, err);
            }
        }
        else
        {
            gimbal_cmd_send.pitch = PIT;
        }
        GimbalPitchLimit(); // 兜底限位(在赋值之后)
        RoundPatrol();
        gimbal_cmd_send.autoaim_mode = AUTO_OFF;
    }
    else
    {
        /* 小电脑有目标 → 视觉跟踪 */
        FoundEnermy();
    }
}

static void SentrySet()
{
    Sentry_ChassisAC();
    ChassisRotateSet();
    ShootAC();
    Sentry_GimbalAC();
}

void Deathcheck()
{
    if (referee_data->GameRobotState.current_HP == 0)
    {
        gimbal_cmd_send.Death_reInit = 1;
    }
    else
    {
        gimbal_cmd_send.Death_reInit = 0;
    }
}

static void NoneAutoMouseControl()
{
    
    gimbal_cmd_send.yaw -= (float)rc_data[TEMP].mouse.x / 660 *3 ;
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660/57 ;
    if(rc_data[TEMP].mouse.press_l==1)
    {
        if(DataLebel.reverse_flag==1)
        {
            shoot_cmd_send.loader_mode = LOAD_REVERSE;
        }
        else
        {
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        }
    }
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }            
}
static void MouseControl()
{
    if (rc_data[TEMP].mouse.press_r == 1)
    {
        /* 右键按下 → 小电脑接管云台控制 + can_fire自动开火 */
        gimbal_cmd_send.autoaim_mode = FIND_Enermy;
        FoundEnermy();
        if (minipc_recv_data->Vision.can_fire == 1)
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        else
            shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    else
    {
        /* 右键未按下 → 手动鼠标控制 */
        gimbal_cmd_send.autoaim_mode = AUTO_OFF;
        NoneAutoMouseControl();
    }
}

static void KeyControl()
{
    chassis_cmd_send.vx = (rc_data[TEMP].key[KEY_PRESS].w * 20000 - rc_data[TEMP].key[KEY_PRESS].s * 20000)*chassis_speed_buff;
    chassis_cmd_send.vy = (rc_data[TEMP].key[KEY_PRESS].d * 20000 - rc_data[TEMP].key[KEY_PRESS].a * 20000)*chassis_speed_buff;

    ChassisRotateSet();
    switch (referee_data->GameRobotState.robot_level)
    {
    case 1:
        chassis_rotate_buff = 1;
        chassis_speed_buff  = 1;
        break;
    case 2:         
        chassis_rotate_buff = 1.2;
        chassis_speed_buff  = 1.03;
        break;
    case 3:
        chassis_rotate_buff = 1.3;
        chassis_speed_buff  = 1.05;
        break;
    case 4:
        chassis_rotate_buff = 1.4;
        chassis_speed_buff  = 1.1;
        break;
    case 5:
        chassis_rotate_buff = 1.5;
        chassis_speed_buff  = 1.15;
        break;
    case 6:
        chassis_rotate_buff = 1.6;
        chassis_speed_buff  = 1.2;
        break;
    case 7:
        chassis_rotate_buff = 1.7;
        chassis_speed_buff  = 1.25;
        break;
    case 8:
        chassis_rotate_buff = 1.8;
        chassis_speed_buff  = 1.3;
        break;
    case 9:
        chassis_rotate_buff = 1.9;
        chassis_speed_buff  = 1.35;
        break;
    case 10:
        chassis_rotate_buff = 2;
        chassis_speed_buff  = 1.4;
        break;
    default:
        chassis_rotate_buff = 1;
        chassis_speed_buff  = 1;
        break;
    }

    if(chassis_fetch_data.power_flag==1)
    {
        chassis_cmd_send.chassis_rotate_buff= 2;
    }
    else
    {
        chassis_cmd_send.chassis_rotate_buff= chassis_rotate_buff;
    }






    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) 
    {
    case 0:
        chassis_cmd_send.chassis_mode =CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    default:
        chassis_cmd_send.chassis_mode =CHASSIS_ROTATE;
    }

    if(rc_data[TEMP].key[KEY_PRESS].q)
    {
        DataLebel.reverse_flag=1;
        shoot_cmd_send.loader_mode = LOAD_REVERSE;
    }
    else
    {
        DataLebel.reverse_flag=0;
    }

    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:
        chassis_cmd_send.chassis_speed_buff= 2;
        break;
    default:
        break;
    }
}



/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    MouseControl();
    KeyControl();
}

/**
 * @brief 停止
 */
static void AnythingStop()
{
    gimbal_cmd_send.gimbal_mode=GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.loader_mode = LOAD_STOP;
    gimbal_cmd_send.pitch = 0.0;
    DataLebel.ACEntryPoint = 1;
    //重置与小电脑通信失败的标志位
    DataLebel.cmd_error_flag=0;
}

/**
 * @brief 控制量及模式设置
 *
 */
static void ControlDataDeal()
{
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
    {
        BasicSet();
        RemoteControlSet();
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        BasicSet();
        SentrySet(); // 哨兵自动模式
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right)) 
    {
        AnythingStop();
    }
}

static void EnemyJudge()
{
    if(referee_data->GameRobotState.robot_id>7)
    {
        minipc_send_data.Vision.detect_color = COLOR_RED;
    }
    else
    {
        minipc_send_data.Vision.detect_color = COLOR_BLUE;
    }
}
static void SendToUIData()
{
    ui_data.autoaim_mode=gimbal_cmd_send.autoaim_mode;
    ui_data.chassis_mode=chassis_cmd_send.chassis_mode;
    ui_data.loader_mode=shoot_cmd_send.loader_mode;
    ui_data.shoot_mode=shoot_cmd_send.shoot_mode;
    ui_data.gimbal_mode=gimbal_cmd_send.gimbal_mode;
}


/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    Deathcheck();

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    ControlDataDeal();

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // 推送消息,双板通信,视觉通信等
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    EnemyJudge();
    SendMinipcData(&minipc_send_data);
    SendToUIData();

}
