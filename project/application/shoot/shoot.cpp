/**
 * @file    shoot.cpp
 * @brief   发射应用
 *
 * 注意: 对方 velocity_ = DJIM_VELOCITY(rpm) / reduction_ratio_; 旧 C 版 speed_aps = 6.0*raw_rpm 是
 *       **转子** deg/s 且不除减速比, 故下面 3 个电机都填 reduction_ratio = 1.0f 保持同量纲.
 * PID 数值照抄旧 C, 输出量纲/积分时间/输出限幅的换算统一交给 pid_port.h。
 */
#include "shoot.h"
#include "robot_def.h"
#include "robot_cmd.h"

#include "dji_motor.h"
#include "pid_port.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/* shoot_cmd_recv 的 extern 声明在 robot_cmd.h */

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotor friction_l, friction_r, loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

Shoot_Upload_Data_s shoot_feedback_data;

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

/* 摩擦轮速度环数值照抄旧 C(一字不改), 量纲/积分单位/输出限幅的换算见 pid_port.h */
static const PidPort kFrictionSpeedPid = {
    .Kp = 20,
    .Ki = 1,
    .Kd = 0,
    .IntegralLimit = 10000,
    .MaxOut = 15000,
    .Improve = PID_Integral_Limit,
};

/* 旧 current_PID（摩擦轮/拨盘）: Kp=0.7 Ki=0.1, MaxOut 分别 15000/5000(ESC 计数)。
   内环在安培域跑，故只有 MaxOut 要换算。 */
static const PidPort kFrictionCurrentPid = {
    .Kp = 0.7f,
    .Ki = 0.1f,
    .Kd = 0.0f,
    .IntegralLimit = 10000,
    .MaxOut = 15000.0f * PID_SCALE_M3508,
    .Improve = PID_Integral_Limit,
};

static const PidPort kLoaderSpeedPid = {
    .Kp = 10,
    .Ki = 1,
    .Kd = 0,
    .IntegralLimit = 5000,
    .MaxOut = 5000,
    .Improve = PID_Integral_Limit,
};

static const PidPort kLoaderCurrentPid = {
    .Kp = 0.7f,
    .Ki = 0.1f,
    .Kd = 0.0f,
    .IntegralLimit = 5000,
    .MaxOut = 5000.0f * PID_SCALE_M2006,
    .Improve = PID_Integral_Limit,
};

void ShootInit()
{
    // 左摩擦轮
    DJIMotor::Config friction_config = {
        .can_handle = &hcan2,
        .motor_type = DJIMotor_3508,
        // 同 chassis: 旧 speed_aps 是转子 deg/s 且不除减速比, 故填 1.0f 与旧参考值(35000)同量纲
        .reduction_ratio = 1.0f,
        // 3508 速度环由 MOTOR 任务 5 分频以 200Hz(5ms) 调用, 故 period = 5
        .pid_velocity = pidPort(kFrictionSpeedPid, PID_SCALE_M3508, 5),
        // 旧摩擦轮是 SPEED_LOOP | CURRENT_LOOP 串级, 内环在安培域(output_scale=1)
        .pid_current = pidPort(kFrictionCurrentPid, 1.0f, 5),
        .current_loop_enable = 1,
    };

    friction_config.motor_id = 2;
    friction_config.direction = DJIM_DIRECTION_NORMAL;
    friction_l.init(friction_config);

    friction_config.motor_id = 1; // 右摩擦轮
    friction_config.direction = DJIM_DIRECTION_REVERT;
    friction_r.init(friction_config);

    // 拨盘电机
    DJIMotor::Config loader_config = {
        .can_handle = &hcan2,
        .motor_type = DJIMotor_2006,
        .direction = DJIM_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        // 同摩擦轮: 旧 speed_aps 是转子 deg/s, 填 1.0f 与旧参考值(含 ×REDUCTION_RATIO_LOADER)同量纲
        .reduction_ratio = 1.0f,
        // 2006 速度环由 MOTOR 任务 5 分频以 200Hz(5ms) 调用, 故 period = 5
        .pid_velocity = pidPort(kLoaderSpeedPid, PID_SCALE_M2006, 5),
        // 旧拨盘是 CURRENT_LOOP | SPEED_LOOP 串级, 内环在安培域(output_scale=1)
        .pid_current = pidPort(kLoaderCurrentPid, 1.0f, 5),
        .current_loop_enable = 1,
    };
    // TODO(移植): 原版初始化即处于 SPEED_LOOP 且参考值为 0; 对方框架 init 后 pid_mode_ 为 Current(开环),
    //             若上电瞬间需要"停住拨盘", 需在 Robot 初始化后显式 loader.setVelocity(0), 待确认.
    loader_config.motor_id = 3;
    loader.init(loader_config);
}

static void ShootStateSet()
{
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        friction_l.setEnable(0);
        friction_r.setEnable(0);
        loader.setEnable(0);
    }
    else // 恢复运行
    {
        friction_l.setEnable(1);
        friction_r.setEnable(1);
        loader.setEnable(1);
    }
}

static void ShootRateSet()
{
    switch (shoot_cmd_recv.loader_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        loader.setVelocity(0);
        // 同时设定参考值为0,这样停止的速度最快
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度
        // (reduction_ratio = 1.0f, 参考值与旧版同为转子 deg/s, 故 ×REDUCTION_RATIO_LOADER 保持原样)
        loader.setVelocity(shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER);
        break;
    // 拨盘反转,对速度闭环（待测试）
    case LOAD_REVERSE:
        loader.setVelocity(-1000);
        break;
    default:
        // 未知模式: 原来在这里 while(1) 会把整个 RobotTask 卡死(电机保持最后指令且无看门狗可救),
        // 现在改为"停机 + 报错": 拨盘停止输出, 并回到 LOAD_STOP 状态
        LOG_ERR(LOG_MOD_SHOT, "shoot", "unknown loader_mode [%d], force LOAD_STOP", (int)shoot_cmd_recv.loader_mode);
        loader.setEnable(0);
        shoot_cmd_recv.loader_mode = LOAD_STOP;
        break;
    }
}

static void ShootSpeedSet()
{
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        friction_l.setVelocity(30000); 
        friction_r.setVelocity(30000);
    }
    else // 关闭摩擦轮
    {
        friction_l.setVelocity(0);
        friction_r.setVelocity(0);
    }
}

static void SendShootData()
{
    // 原 loader->measure.speed_aps 与对方 velocity_ 同为转子 deg/s(reduction_ratio = 1.0f);
    // 字段类型是 int16_t, 按原逻辑直接赋值(超出 int16_t 范围时的截断行为与原版一致)
    shoot_feedback_data.loader_speed_aps = (int16_t)loader.velocity_;
}

void ShootTask()
{
    ShootStateSet();
    ShootRateSet();
    ShootSpeedSet();
    SendShootData();
    // 反馈数据,用于卡弹反馈（后续再加个模块离线）
}
