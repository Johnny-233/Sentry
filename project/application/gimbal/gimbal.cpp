/**
 * @file    gimbal.cpp
 * @brief   云台应用层
 * @note    pitch(小米 MI 电机)逻辑逐行照搬旧 C: 上电回零状态机 PitchHomingTask()、软限位、
 *          MI 位置控制, 参数/时序/判据一个数字都没改;
 *          yaw(GM6020)按 PORT_MAPPING.md §6 适配: 对方 DJIMotor 没有外部反馈通路, 原电机
 *          控制器里的角度环+速度环搬到应用层(见下面 yaw 段)。
 *          禁堆、禁异常、静态实例。
 */

#include "gimbal.h"

#include "robot_def.h"
#include "robot_cmd.h"      /* gimbal_cmd_recv（定义在 robot_cmd.cpp） */
#include "ahrs.h"
#include "dji_motor.h"
#include "mi_motor.h"
#include "pid.h"
#include "pid_port.h"       /* 旧 C PID 配置 → 对方 PID 类的量纲/单位换算 */
#include "bsp_log.h"

#include <math.h>

/* 全局姿态实例：定义在 application/cmd/robot_cmd.cpp */
extern AHRS g_ahrs;
/* cmd → gimbal 的控制指令：定义在 application/cmd/robot_cmd.cpp */
extern Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;

/* 云台反馈数据：定义在这里, extern 声明在 gimbal.h */
Gimbal_Upload_Data_s gimbal_feedback_data;

/* 电机静态实例（禁堆）：yaw = GM6020(&hcan2, tx_id=1)；pitch = 小米 MI 电机(&hcan2, 扩展帧) */
static DJIMotor yaw_motor;
static MIMotor  pitch_motor;
static uint8_t motor_init = 0;   // 0 = 需要重新使能/清积分/重设设定值

/* ==================== yaw 结构适配（PORT_MAPPING §6）====================
 * 原 C 版把 IMU 当"外部反馈"塞进电机控制器(角度环 OTHER_FEED + &INS.YawTotalAngle,
 * 速度环 &INS.Gyro[2])；对方 DJIMotor 没有外部反馈通路, 故两个环都在应用层:
 *   角度环: 设定 = gimbal_cmd_recv.yaw(deg), 反馈 = g_ahrs.output_.yaw_total(deg), 输出 = 期望角速度(deg/s)
 *   速度环: 设定 = 角度环输出, 反馈 = g_ahrs.output_.gyro_b[2](rad/s, 与旧 INS.Gyro[2] 同单位 → 增益不用改)
 *   速度环输出 = 电流计数 → yaw_motor.setCurrent(输出 / 25000.0f)(GM6020 电压控制量程 ±25000)
 * 两个环都在 200Hz 的 ROBOT 任务(GimbalTask)里跑 → period = 5ms;
 * 旧 C 的 ki 单位是秒、输出是电流计数, 量纲/单位换算交给 pid_port.h(见其文件头推导)。 */
static PID yaw_angle_pid;
static PID yaw_speed_pid;
static float yaw_ff_last_measure = 0.0f;   // 角度环前馈用：上一次设定值(设定值差分前馈)

/* 数值照抄旧 C(controller_param_init_config), 一字不改 */
static const PidPort kYawAnglePid = {
    .Kp = 95,  // 110 抖动, 回退一档
    .Ki = 60,  // 补足稳态跟随误差: 1.7s → 0.6s
    .Kd = 9,   // 提高阻尼, 抑制高 Kp 抖动
    .IntegralLimit = 250,  // 经速度环(×约50)后≈12500 计数的电流偏置
    .MaxOut = 330,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .DeadBand = 0.1f,
    .FF_Gain = 50.0f,      // 速度前馈(作用在**设定值差分**上)：等效 ~25% 超前(200=满超前)
};
static const PidPort kYawSpeedPid = {
    .Kp = 50,
    .Ki = 60,  // 150 → 60: 抑制锁定微振
    .Kd = 0,
    .IntegralLimit = 6000,  // 匀速跟随需积分提供偏置电流, 3000 偏紧
    .MaxOut = 20000,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .FF_Gain = 0.0f,  // 陀螺噪声会被微分放大, 保持 0(旧 500 从未生效)
};

/* pitch 位置环的 kp/kd：旧版取自 pitch_motor->motor_controller.angle_PID, 值不变(Kp=15, Kd=1.0) */
static const float PITCH_ANGLE_KP = 15.0f;
static const float PITCH_ANGLE_KD = 1.0f;

/* MI 电机 CAN ID：旧 C 版所有 MI 报文的 ExtId.motor_id 域都写死 127（CyberGear 出厂默认 CAN ID，
 * 新模块 mi_motor.h 的 MI_EXT_ID_MOTOR_FIELD 也是 127），而 MIMotor::init() 把 Config::motor_id
 * 同时用作接收匹配 ID。这里取 127，与旧版寻址行为一致。
 * TODO(移植): 实车 MI 电机 CAN ID 若不是 127，同步改这里（或改用模块的 ID 配置接口）。 */
static const uint8_t MI_MOTOR_ID = 127;

/* ---- pitch 上电标定/测量开关 ----
 * 1 = 测试固件: 不设机械零位、不使能也不控制 pitch 电机(保持自由停止),
 *     仅用于通过黑匣子读取 MI 电机上报角度, 验证"机械零位掉电后回到的基准是否固定"。
 * 0 = 正常工作固件。测完必须改回 0！ */
#define PITCH_BRINGUP_TEST 0

void GimbalInit()
{
    /* 姿态实例 g_ahrs 已由 RobotCMDInit() 初始化(预热/校准)并在 robot.cpp 里 start(),
     * 这里只引用全局实例, 不再初始化 IMU。 */

    /* ---- YAW: GM6020, &hcan2, tx_id = 1 ---- */
    DJIMotor::Config yaw_config = {
        .can_handle = &hcan2,
        .motor_id = 1,
        .motor_type = DJIMotor_6020,
        .direction = DJIM_DIRECTION_NORMAL,
        .reduction_ratio = 1.0f,                  // 6020 用 1.0
        /* 电机内部 PID 全部不用（规约 §6 的环搬到应用层）：本工程只对 yaw 电机用
         * setCurrent()（pid_mode_ = Current），DJIMotor::update() 不会走内部两个 PID。
         * 位置环/速度环的 kp/ki/kd 在旧版里也是给"外部反馈"用的，这里留空。 */
        .pid_angle = {},
        .pid_velocity = {},
        .pos_freq_div = 1,                        // 仅内部环使用
        .initial_angle = 0.0f,
    };
    yaw_motor.init(yaw_config);

    /* ---- PITCH: 小米 MI 电机, &hcan2, 扩展帧 ---- */
    MIMotor::Config pitch_config = {
        .can_handle = &hcan2,
        .motor_id = MI_MOTOR_ID,
    };
    pitch_motor.init(pitch_config);

    /* ---- yaw 两个环的 PID（旧 C 数字照抄，量纲换算见 pid_port.h）---- */
    yaw_angle_pid.init(pidPort(kYawAnglePid, PID_SCALE_COUNTS, 5));
    yaw_speed_pid.init(pidPort(kYawSpeedPid, PID_SCALE_COUNTS, 5));

#if PITCH_BRINGUP_TEST
    pitch_motor.stop(); // 测试: 自由状态(可被重力/手移动), 仅读取上报角度
#else
    pitch_motor.setEnable(1);
    /* 这里不调 setMechPositionToZero(): 零位统一由下面的上电回零(homing)在找到机械下限后设定 */
#endif

    /* 原 gimbal_pub/gimbal_sub 的注册已删除（规约 §1.5：消息中心全部去掉，改直接读全局实例） */
}

/* ==================== pitch 上电回零 (homing) ====================
 * 为什么必须做: 实测 MI 电机每次上电后上报角度的绝对基准都会变(数值不一致),
 * 因此"固定绝对限位"不可行(上次的测试固件没使能电机, 所以没暴露这个问题)。
 * 做法: 首次进入云台模式时, 每拍只推进 HOMING_STEP_RAD 往下"找"机械下限(顶到限位时位置误差缓慢累积),
 *       连续超阈值即判定到下限, 在那里发"设置机械零位"(通信类型6), 之后软限位都相对这个零点;
 *       设零后必须静默几拍(只发零位命令, 不再发位置命令), 否则会被下一个位置命令覆盖 ——
 *       MI 电机发送用的是同一个静态缓冲(见 CODE_REVIEW 的 N6)。
 * 安全: 回零用很小的 kp(推力小) + 限时(超时则放弃回零并报错, 不用未标定的限位驱动机构)。
 * ⚠️ 上机首次验证: 观察回零时炮管是否朝"下"移动; 若朝上走, 把 PITCH_HOMING_DIR 改成 -1.0f。 */
#define PITCH_HOMING_ENABLE 1
#define PITCH_HOMING_DIR (+1.0f)   /* +1 = 目标递增方向为"下"(与实测一致: 下垂位读数更大) */
#define HOMING_STEP_RAD 0.001f     /* 每拍推进量 */
#define HOMING_STALL_ERR 0.10f     /* 位置误差超过此值视为顶住 */
#define HOMING_STALL_CNT 40        /* 连续 40 拍(0.2s)确认 */
#define HOMING_MAX_RAD 1.20f       /* 最多推进这么多(略大于全行程 0.91rad) */
#define HOMING_SILENCE_CNT 10      /* 设零后静默拍数, 保证零位命令真的发出去 */
#define HOMING_KP 3.0f             /* 回零期间的位置环 kp(小 => 顶住时推力小) */
#define HOMING_KD 0.5f

static uint8_t pitch_homed = 0;    /* 1 = 本次使能期间已完成回零 */

/* 请求重新回零：MI 电机掉线/重新使能后，setMechPositionToZero() 建立的零位会失效，
   继续在旧坐标系里发指令它就会顶到限位（现象："pitch 控不了"）。 */
static uint8_t homing_reset_req = 0;
static void PitchHomingRequest(void)
{
    homing_reset_req = 1;
}

/* 返回 1 表示已完成回零(调用方可以正常控制 pitch), 0 表示仍在回零/回零失败 */
static uint8_t PitchHomingTask(void)
{
    static uint8_t state = 0; /* 0=开始 1=找下限 2=静默发零位 3=完成 4=失败 */
    static float start_angle, target;
    static uint16_t stall_cnt, total_cnt, silence_cnt, wd_cnt;
    static float max_torque; /* 回零全程出现过的最大|力矩|，用来判断电机到底有没有在出力 */

    if (homing_reset_req) /* 重新进入控制态 / 上次通信中断：丢掉旧零位，重新找下限 */
    {
        homing_reset_req = 0;
        pitch_homed = 0;
        state = 0;
    }

    if (state == 3)
    {
        pitch_homed = 1;
        return 1;
    }
    if (state == 4)
        return 0;

    float angle = pitch_motor.angle_;   // rad

    if (state == 0)
    {
        /* 先把运行模式写回运控模式(0x7005=0)：MI 电机若停在位置/速度/电流模式会完全忽略
           type1 位置指令（现象：有反馈、不出力、手能推）。写完等几十 ms 再使能。 */
        pitch_motor.modeSwitch(0);
        wd_cnt = 0;
        state = 5;
        LOG_INFO(LOG_MOD_GIMB, "homing", "pitch: run_mode -> 0(control), waiting before enable");
    }

    if (state == 5) /* 等运行模式切换生效(~100ms @200Hz)后再使能 */
    {
        if (++wd_cnt < 20)
            return 0;

        pitch_motor.setEnable(1);
        start_angle = angle;
        target = angle;
        stall_cnt = 0;
        total_cnt = 0;
        max_torque = 0.0f;
        state = 1;
        // nano.specs 下无 %f，按对方框架惯例用 mrad 整数打印
        LOG_INFO(LOG_MOD_GIMB, "homing", "pitch homing start at %d mrad", (int)(angle * 1000.0f));
    }

    if (state == 1)
    {
        target += PITCH_HOMING_DIR * HOMING_STEP_RAD;
        total_cnt++;
        pitch_motor.locationControl(target, HOMING_KP, HOMING_KD);   // 原 MI_motor_LocationControl

        { /* 记录最大力矩：电机有没有真的在出力，比"角度有没有变"更能区分
           * "已经顶在机械下限上" 和 "电机根本没使能/故障" 这两种情况 */
            float at = fabsf(pitch_motor.torque_);
            if (at > max_torque)
                max_torque = at;
        }

        if (fabsf(angle - target) > HOMING_STALL_ERR)
            stall_cnt++;
        else
            stall_cnt = 0;

        if (stall_cnt > HOMING_STALL_CNT)
        {
            /* 用"力矩有没有出"而不是"角度有没有变"判断是否真到下限：炮管本来就被重力压在
               下限上时推不动但有力矩=合法；电机没使能/故障则全程力矩≈0。 */
            if (max_torque < 0.05f)
            {
                LOG_ERR(LOG_MOD_GIMB, "homing",
                        "pitch homing FAILED: no torque (max %d mNm), travel %d mrad -> motor not enabled/fault?",
                        (int)(max_torque * 1000.0f), (int)((angle - start_angle) * 1000.0f));
                pitch_motor.stop();
                state = 4;
            }
            else
            {
                pitch_motor.setMechPositionToZero(); /* 在当前位置(机械下限)设零 */
                LOG_INFO(LOG_MOD_GIMB, "homing",
                         "pitch homing: lower limit set (travel %d mrad, max torque %d mNm)",
                         (int)((angle - start_angle) * 1000.0f), (int)(max_torque * 1000.0f));
                silence_cnt = 0;
                state = 2;
            }
        }
        else if (total_cnt > 4000 || fabsf(target - start_angle) > HOMING_MAX_RAD)
        {
            LOG_ERR(LOG_MOD_GIMB, "homing",
                    "pitch homing FAILED (no stall within %d mrad) -> pitch disabled, check PITCH_HOMING_DIR",
                    (int)(HOMING_MAX_RAD * 1000.0f));
            pitch_motor.stop();
            state = 4;
        }
        return 0;
    }

    /* state == 2: 静默几拍, 见文件头 */
    silence_cnt++;
    if (silence_cnt >= HOMING_SILENCE_CNT)
    {
        pitch_motor.setEnable(1);
        gimbal_cmd_recv.pitch = pitch_motor.angle_; /* 以回零后的位置作为指令起点 */
        LOG_INFO(LOG_MOD_GIMB, "homing", "pitch homing done, angle now %d mrad",
                 (int)(pitch_motor.angle_ * 1000.0f));
        state = 3;
        pitch_homed = 1;
        return 1;
    }
    return 0;
}

/* yaw 串级控制: 角度环 → 速度环, 两个环都在应用层 */
static void YawControlUpdate(void)
{
    float yaw_total = g_ahrs.output_.yaw_total;   // 角度反馈 [deg]
    float yaw_gyro  = g_ahrs.output_.gyro_b[2];   // 速度反馈 [rad/s]（与旧 INS.Gyro[2] 同单位）

    /* 角度环：位置式，输出 = 期望角速度。前馈需外部先 setFeedforward(差分), 见 pid_port.h。 */
    /* 前馈源改用**设定值差分**（不是反馈差分）：反馈差分在底盘快转/振动时噪声很大，
       FF_Gain=120 会把它放大成"甩云台"——现象就是云台快转时被拽向底盘、世界角保不住。
       设定值差分只反映摇杆/视觉下达的运动(等效"提前给出设定速率的 ~60%")，
       既保留跟随超前，又完全不理会反馈噪声。 */
    yaw_angle_pid.setFeedforward(gimbal_cmd_recv.yaw - yaw_ff_last_measure);
    yaw_ff_last_measure = gimbal_cmd_recv.yaw;
    yaw_angle_pid.update(yaw_total);

    /* 速度环：设定值 = 角度环输出(期望角速度 deg/s) */
    yaw_speed_pid.setSetpoint(yaw_angle_pid.output_);
    yaw_speed_pid.update(yaw_gyro);

    /* 输出 = 电流计数 → A：GM6020 电压控制模式 25000 计数 = 1.0（旧版 MaxOut=20000 也是这个量纲） */
    yaw_motor.setCurrent(yaw_speed_pid.output_ / 25000.0f);
}

/* 电机掉电/失联恢复 + 裁判断电复活后的重新对齐：
 *  - yaw(GM6020)：软件累加角度基点会错位 → resetAngle() 重新对齐（控制环用 IMU，不受影响）
 *  - pitch(MI)：内部绝对基准重置、机械零位失效 → motor_init=0 并重做回零
 *  - Death_reInit：裁判 HP==0 断电，复活(1→0 沿)时走同一套恢复流程 */
static void GimbalRecoverCheck()
{
    static uint8_t yaw_lost = 0, pitch_lost = 0, death = 0, yaw_seen = 0;

    /* ⚠不能在这里 resetAngle() 清零 yaw 累加角度：上电头几拍 motor_valid_ 还是 0（反馈帧没到），
       会被误判成"掉线→恢复"，一上电就把角度清零 → offset_angle/底盘偏角整体错位，
       现象就是"重新上电后四个就近跟随方向错、摇杆移动方向与实际不符"。
       所以：(a) 只有"曾经有效过"才允许判掉线；(b) 恢复时只重设设定值，不重建角度基准。 */
    if (!yaw_motor.motor_valid_)
    {
        if (yaw_seen)
            yaw_lost = 1;
    }
    else
    {
        yaw_seen = 1;
        if (yaw_lost)
        {
            yaw_lost = 0;
            motor_init = 0; /* 让 motor_init==0 分支用当前 IMU 角度重设设定值, 避免阶跃 */
        }
    }

    if (!pitch_motor.valid_)
        pitch_lost = 1;
    else if (pitch_lost)
    {
        pitch_lost = 0;
        motor_init = 0;
        PitchHomingRequest();
    }

    if (gimbal_cmd_recv.Death_reInit)
        death = 1;
    else if (death)
    {
        death = 0;
        motor_init = 0;
        PitchHomingRequest();
    }
}

static void GimbalStateSet()
{
    GimbalRecoverCheck();

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        /* pitch 不能发 MI type4(reset/stop)：发过之后它就不再响应 type1 指令、只有断电才恢复
           （现象：切 C 档再切回 N 就再也控不了）。改发 kp=kd=0 的零力矩运控帧：不出力、可自由
           推动、位置基准不丢，MiMotorControl() 会 100Hz 持续重发。pitch_homed 保持不变。 */
        pitch_motor.locationControl(pitch_motor.angle_, 0.0f, 0.0f);
        yaw_motor.setEnable(0);
        motor_init = 0;
        /* 0 电流态清零两个 PID 积分（规约 §6）：原逻辑在"重新进入控制态"的 motor_init 分支里清，
         * 这里停机时也清一次，避免停机期间用手转动云台积累积分 windup。 */
        yaw_angle_pid.resetIntegral();
        yaw_speed_pid.resetIntegral();
        break;
    case GIMBAL_GYRO_MODE:
        yaw_motor.setEnable(1);
        yaw_angle_pid.setSetpoint(gimbal_cmd_recv.yaw);
#if PITCH_BRINGUP_TEST
        pitch_motor.stop(); // 测试: 保持自由, 绝不用未标定的限位去驱动机构
#else
#if PITCH_HOMING_ENABLE
        /* 只在零位可能失效时回零：上电后首次进入控制态、或上次回零失败过。
           正常 C→N 零位是保住的（见 ZERO_FORCE 的零力矩释放），不必每次重来。 */
        if (motor_init == 0 && !pitch_homed)
            PitchHomingRequest();
        if (PitchHomingTask()) // 回零完成前不执行正常位置控制(期间只做"缓慢找下限")
#endif
        {
            pitch_motor.locationControl(gimbal_cmd_recv.pitch, PITCH_ANGLE_KP, PITCH_ANGLE_KD);
        }
#endif
        if (motor_init == 0)
        {
            pitch_motor.setEnable(1);
            // 清除yaw PID积分,防止停机期间积分windup导致使能瞬间过流
            yaw_angle_pid.resetIntegral();
            yaw_speed_pid.resetIntegral();
            // 以当前角度初始化设定值,避免阶跃
            yaw_angle_pid.setSetpoint(g_ahrs.output_.yaw_total);
            yaw_ff_last_measure = gimbal_cmd_recv.yaw;  // 前馈基准: 与设定值差分前馈同源
            gimbal_feedback_data.init_location = g_ahrs.output_.euler[1];
            motor_init = 1;
        }
        // yaw 串级控制（角度环 + 速度环，都在应用层）
        YawControlUpdate();
        break;
    default:
        /* GIMBAL_FREE_MODE（以及其它未定义模式）落在这里：与旧 C 版一样"不动作"。
         * TODO(移植): 旧 C 版电机内部环仍在 1kHz 跑，FREE 模式下会继续按上一拍的 pid_ref 出力；
         *   搬到应用层后没有循环在跑，本分支不动作 = 保持上一拍的电流指令（会持续出力）。
         *   本工程 robot_cmd 只下发 GIMBAL_ZERO_FORCE / GIMBAL_GYRO_MODE（见 robot_cmd.c），
         *   FREE 模式未被使用，故按规约 §1.1"保留原逻辑"处理；
         *   若以后要启用 FREE 模式，这里需要显式 yaw_motor.setEnable(0)/setCurrent(0)。 */
        break;
    }
}

static void SendGimbalData()
{
    gimbal_feedback_data.gimbal_imu_data = g_ahrs.output_;
    /* 原 yaw_motor->measure.angle_single_round = 0.043945 * ecd（单圈 0~360 度，cmd 用它算与零位的偏角）。
     * 对方 DJIMotor 不再暴露原始 ecd，这里用累计角度取单圈值，量纲与旧版一致（度，0~360）。
     * TODO(移植): 与 robot_cmd 的 CalcOffsetAngle/YAW_ALIGN_ANGLE(度) 联调确认。 */
    float yaw_single_round = fmodf(yaw_motor.angle_, 360.0f);
    if (yaw_single_round < 0.0f)
        yaw_single_round += 360.0f;
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)yaw_single_round;
    gimbal_feedback_data.pitch_angle = pitch_motor.angle_; /* 原 C 版这句移植时漏了(UI/自瞄恒读 0) */
}

void GimbalTask()
{
    GimbalStateSet();
    SendGimbalData();
}

DJIMotor* GetYawMotor(void)
{
    return &yaw_motor;
}

MIMotor* GetPitchMotor(void)
{
    return &pitch_motor;
}
