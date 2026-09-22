/**
 * @file    robot_cmd.cpp
 * @brief   机器人核心控制任务(RobotCMDInit / RobotCMDTask)
 * @note    由 application/cmd/robot_cmd.c 移植(836 行 C → C++), 控制逻辑/判据/数值逐行保留。
 *
 * 与旧 C 版的机制差异(都是"底层换对方框架"导致的, 逐条列出便于对照):
 *   1. message_center 全部删除: 原 PubRegister/SubRegister/SubGetMessage/PubPushMessage 去掉,
 *      cmd 直接读写全局实例 chassis_cmd_recv/gimbal_cmd_recv/shoot_cmd_recv(定义在本文件);
 *      底盘/云台/发射的反馈直接读 chassis/gimbal/shoot .cpp 里的全局实例(规约 §1.5)。
 *   2. 底层接口:
 *      RemoteControlInit(&huart3)   → Remote_Init(&huart3)  + remote.h 的 REMOTE_RC_* 宏;
 *      minipcInit(&huart1)          → Minipc_Init(&huart1);
 *      UITaskInit(&huart6,&ui_data) → Referee_Init(&huart6) + application/ui/ui.h 的 ui_data。
 *   3. 姿态: INS_Init()/INS_Task()/attitude_t → 全局 AHRS g_ahrs(本文件定义, robot_cmd.h 里 extern);
 *      字段映射(规约 §2): YawTotalAngle→output_.yaw_total, Pitch→euler[1], Yaw→euler[2],
 *                        Gyro[i]→gyro_b[i], Accel[i]→accel_b[i]。
 *   4. 视觉链路用回本仓原本的 seasky 协议(project/modules/master_machine):
 *      minipcInit(&huart1) → Minipc_Init(&huart1, &g_ahrs); 接收帧类型是 Minipc_Recv_s
 *      (字段在 .Vision 下); 本任务结尾照原样 EnemyJudge() + SendMinipcData(...),
 *      另外 INS 任务每 1ms 还会 SendMinipcData(NULL)。
 *      **发送数据只有一份**(模块持有的那份, 用 Minipc_GetSendData() 取): 原实现里
 *      robot_cmd 自带一份、模块内部另有一份, 1kHz 那条路径发出去的那份 detect_color
 *      从来没被填过(一直是 0=红)。现在两条路径共用同一份, 每帧都带正确颜色。
 *   5. 蜂鸣器/IWDG 本次不做; 原 robot_cmd.c 里也没有这两者的调用(见 PORT_MAPPING §5)。
 */
// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "ui.h"
#include "pid_port.h"
// module
#include "remote.h"
#include "master_process.h"
#include "ahrs.h"
#include "dji_motor.h"
#include "mi_motor.h"
#include "gimbal.h"
#include "gimbal_algorithm.h"
#include "chassis.h"
#include "shoot.h"
#include "referee.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "main.h"
#include "usart.h"

#include <math.h>

/* 各应用的反馈全局实例(定义在各自 .cpp, 规约 §1.5, 上面 chassis.h/shoot.h/gimbal.h 里已 extern):
   chassis_feedback_data / gimbal_feedback_data / shoot_feedback_data */

/* 私有宏,自动将编码器转换成角度值
   (ECD_ANGLE_COEF_DJI 原在 Modules/motor/DJImotor/dji_motor.h, 对方框架没带这个宏, 值不变) */
#ifndef ECD_ANGLE_COEF_DJI
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制
#endif
#ifndef DEGREE_2_RAD
#define DEGREE_2_RAD 0.01745329252f // pi/180 (原 general_def.h)
#endif

#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* ---------------- 遥控器开关/按键的机型适配 ----------------
 * 下面这一层的意义：把"机型差异"全吃掉，让后面的控制判据（ChassisRC / ControlDataDeal）
 * 两个机型共用同一份代码、同一套数值。
 *
 * 旧 remote_control.c(DBUS) 从帧里解出两个"开关"：
 *   switch_right = byte5[4:5]，switch_left = byte5[6:7]
 * 旧 robot_cmd.c 用 switch_is_down/mid/up 判据：== 2 / == 3 / == 1。
 *
 * 【本车现在用的新遥控 VT13/VT03（图传链路，带键鼠）】按你给的重新映射：
 *   右三档开关 mode_sw：C(0) = 零电流停机   N(1) = 手动(底盘跟随)   S(2) = 哨兵自动模式
 *   左/右 FN 直接选小陀螺方向（新遥控没有左三档开关）：
 *       都不按 = 底盘跟随；左 FN = 小陀螺一个方向；右 FN = 另一个方向
 *   —— 正好套进原来的三个判据：DOWN=跟随 / MID=小陀螺正 / UP=小陀螺反。
 *   ⚠️ 若实测发现左右 FN 的小陀螺转向反了，只需把 `RC_SWITCH_LEFT()` 里的 1u / 2u 对调。
 *
 * 【DT7/HT10A（对方的默认机型，旧遥控）】真三档开关，UP=1/MID=3/DOWN=2，与旧 RC_SW_* 同值。
 */
#if defined(REMOTE_DEVICE_VT13)
/* 右三档开关：C=停机 / N=手动 / S=哨兵自动 */
#define RC_SWITCH_RIGHT() ((uint8_t)REMOTE_RC_SWITCH()) // mode_sw: 0=C 1=N 2=S
#define RC_SW_VALUE_DOWN 0u                             // C → AnythingStop(零电流)
#define RC_SW_VALUE_MID 1u                              // N → 手动(底盘跟随 / 小陀螺由 FN 选)
#define RC_SW_VALUE_UP 2u                               // S → 哨兵自动模式
/* 左"开关"：用左右 FN 模拟原来的三档(都不按=跟随)。
   注意这里必须和右边共用同一组 VALUE 常量，所以：
     返回 0(=DOWN) → ChassisRC() 判为"跟随"   ← 两个 FN 都不按
     返回 1(=MID)  → 小陀螺 chassis_rotate_buff = +1
     返回 2(=UP)   → 小陀螺 chassis_rotate_buff = -1
   ⚠️ 若实测左右 FN 的转向反了，把下面这行的 1u / 2u 对调即可（不要动 VALUE_*，
      那会连带改掉右边三档开关 N/S 的含义）。 */
#define RC_SWITCH_LEFT() ((uint8_t)(REMOTE_RC_FN_LEFT() ? 1u : (REMOTE_RC_FN_RIGHT() ? 2u : 0u)))
#else
#define RC_SWITCH_LEFT() ((uint8_t)REMOTE_RC_SW_LEFT())
#define RC_SWITCH_RIGHT() ((uint8_t)REMOTE_RC_SW_RIGHT())
#define RC_SW_VALUE_UP ((uint8_t)REMOTE_RC_SW_UP)
#define RC_SW_VALUE_MID ((uint8_t)REMOTE_RC_SW_MID)
#define RC_SW_VALUE_DOWN ((uint8_t)REMOTE_RC_SW_DOWN)
#endif

/* ============ cmd 的三个控制实例(定义在此, extern 声明见 robot_cmd.h) ============
 * 原类型名: 旧版叫 chassis_cmd_send/gimbal_cmd_send/shoot_cmd_send 并通过 message_center 发布;
 * 现在就是 chassis/gimbal/shoot 直接读的全局实例。 */
Chassis_Ctrl_Cmd_s chassis_cmd_recv;
Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;
Shoot_Ctrl_Cmd_s shoot_cmd_recv;

/* ============ 全局姿态实例(定义在此, extern 声明见 robot_cmd.h) ============
 * 替代旧版 cmd 持有的 attitude_t*(INS_Init() 返回值), GimbalInit()/robot.cpp 直接读。 */
AHRS g_ahrs;

/* cmd应用包含的模块实例指针和交互信息存储 */
static Minipc_Recv_s *minipc_recv_data; // 视觉接收数据指针,初始化时返回(seasky 协议)
/* 视觉发送数据不再由 cmd 自己持有一份: 统一用模块那份(Minipc_GetSendData()),
   这样 1kHz(INS 任务)与 200Hz(本任务)发出去的是同一份内容, 见文件头差异 4。 */
static referee_info_t *referee_data;        // 用于获取裁判系统的数据

static Robot_Status_e robot_state; // 机器人整体工作状态
static DataLebel_t DataLebel;

static uint8_t gimbal_location_init = 0;
static uint8_t power_flag;

static float chassis_rotate_buff;
static float chassis_speed_buff;
#if defined(REMOTE_DEVICE_VT13)
/* 手动档里 1 = 摇杆(遥控器)接管, 0 = 键鼠接管; 默认摇杆。判据见 ControlDataDeal()。 */
static uint8_t cmd_mode_is_remote = 1;
#endif

static PID chassis_follow_pid; // 底盘跟随模式PID

static cal_round_patrol_t round_patrol;
static cal_mid_round_patrol_t mid_round_patrol;

/* 旧 chassis_follow_pid 的配置, 一字不改地搬过来(规约 §1.5/§4: PID 参数不许改)。
   量纲/时间单位差异(旧 PID 用秒, 对方 PID 用 ms)由 pid_port.h 的 pidPort() 统一换算:
   本 PID 的输出是底盘旋转角速度指令(wz, 被 chassis.cpp 乘半径当轮速用), 不是电流计数, 故取
   PID_SCALE_COUNTS; 调用周期 = ROBOT 任务 200Hz = 5ms。 */
static const PidPort kChassisFollowPid = {
    .Kp = 50.0f,
    .Ki = 0.0f,
    .Kd = 3.0f,
    .IntegralLimit = 0.0f,
    .MaxOut = 4000.0f,
    .Improve = PID_Derivative_On_Measurement,
    .DeadBand = 1.0f,
    .FF_Gain = 0.0f,
};

void RobotCMDInit()
{
    /* 遥控器: 原 RemoteControlInit(&huart3) → 对方的 Remote_Init(&huart3)。
       返回的机型帧指针不需要保存 —— 取数统一用 remote.h 的 REMOTE_RC_* 宏(内部就是 remote_data),
       在线判断用 Remote_Online()。 */
    Remote_Init(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    minipc_recv_data = Minipc_Init(&huart1, &g_ahrs); // 视觉通信串口(seasky 协议, 姿态取自 g_ahrs)
    referee_data = Referee_Init(&huart6);    // 裁判系统(原 UITaskInit(&huart6,&ui_data), UI 状态结构体已挪到 application/ui/)

    /* 姿态解算: 原 INS_Init()(cmd 拥有 attitude_t 实例)。
       参数取原工程 IMU 配置(规约 §2): 量程 G6/Dps2000, 加速度低通 0.0085s;
       其余(安装角/EKF 噪声/预热超时)用对方默认值 —— 不写即 0, AHRS::init 内部按 <=0 取默认。 */
    AHRS::Config ahrs_config = {
        .accel_range = BMI088::AccRange::G6,
        .gyro_range = BMI088::GyroRange::Dps2000,
        .board_yaw = 0.0f,
        .board_pitch = 0.0f,
        .board_roll = 0.0f,
        .accel_lpf_coef = 0.0085f,
    };

    /* 注意(移植关键点):
     *   a) 对方 AHRS::init() 内部会 preheat(): 用 HAL_Delay 等 IMU 加热到 40°C。HAL tick 来自
     *      TIM14 中断, **关中断时 HAL_Delay 会死等 → 整车卡死在启动阶段**。
     *      因此 robot.cpp 的 Robot_Init() 已经去掉了原 C 版的 __disable_irq()/__enable_irq() 包裹
     *      (原 C 版能这么写, 是因为它的 INS_Init 是在 INS 任务里跑的, 预热也在任务里)。
     *      下面这段 save/restore PRIMASK 只是**防御性**的: 万一以后又把本函数挪回关中断区间,
     *      它会临时开中断避免死等; 正常情况下 primask_save 非 0 的"关中断"路径不会走到。
     *   b) 对方 AHRS::init() 内部已经调用 start()(建解算任务), 所以这里**不能再调一次**
     *      g_ahrs.start(), 否则会多出一个解算任务, 两个任务同时写 output_。 */
    const uint32_t primask_save = __get_PRIMASK();
    __enable_irq();
    g_ahrs.init(ahrs_config); // 内部: BMI088 初始化 + 预热 + 陀螺校准 + start()
    if (primask_save != 0u)
        __disable_irq();

    /* 原 message_center 的 6 个 Pub/Sub 注册全部删除:
       gimbal_cmd_pub/gimbal_feed_sub/shoot_cmd_pub/shoot_feed_sub/chassis_cmd_pub/chassis_feed_sub。
       控制指令改为直接读写上面三个全局实例, 反馈读各应用 .cpp 里的全局实例。 */
    gimbal_cmd_recv.pitch = 0;

    // 底盘跟随模式PID初始化
    chassis_follow_pid.init(pidPort(kChassisFollowPid, PID_SCALE_COUNTS, 5));
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    gimbal_feedback_data.offset_diff = gimbal_feedback_data.yaw_motor_single_round_angle - YAW_ALIGN_ANGLE;
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_feedback_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_recv.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_recv.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_recv.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_recv.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_recv.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_recv.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

static void GimbalPitchLimit()
{
    gimbal_cmd_recv.gimbal_mode = GIMBAL_GYRO_MODE;
    // 云台软件限位
    if (gimbal_cmd_recv.pitch < PITCH_MIN_ANGLE)
        gimbal_cmd_recv.pitch = PITCH_MIN_ANGLE;
    else if (gimbal_cmd_recv.pitch > PITCH_MAX_ANGLE)
        gimbal_cmd_recv.pitch = PITCH_MAX_ANGLE;
    else
        gimbal_cmd_recv.pitch = gimbal_cmd_recv.pitch;
}

/**
 * @brief 判断视觉有没有发信息
 *
 */
static void VisionJudge()
{
    static float target_lost_time = 0.0f;
    static uint8_t target_timer_running = 0;

    /* 小电脑识别到目标时 yaw/pitch 会有非零数据(seasky 收帧 Minipc_Recv_s.Vision) */
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

    /* 首次进入时用"当前电机角度"作为 pitch 指令初值:
     * pitch 指令与电机上报角度同坐标系(固定绝对限位), 从当前值起步可保证上电/模式切换不跳变,
     * 也避免"初值 0 落在限位区间外被夹到某一端"导致云台突然转动。 */
    static uint8_t pitch_cmd_inited = 0;
    if (!pitch_cmd_inited)
    {
        MIMotor *pm = GetPitchMotor(); // 原 MIMotorInstance*
        if (pm != NULL && pm->angle_ != 0.0f)
        {
            gimbal_cmd_recv.pitch = pm->angle_;
            pitch_cmd_inited = 1;
        }
    }

    GimbalPitchLimit();
    VisionJudge();
    //发射基本模式设定
    shoot_cmd_recv.shoot_mode = SHOOT_ON;
    shoot_cmd_recv.friction_mode = FRICTION_ON;
    shoot_cmd_recv.shoot_rate = 8;
    chassis_cmd_recv.power_limit = referee_data->robot_status.chassis_power_limit; // 原 GameRobotState.chassis_power_limit
}

static void GimbalRC()
{
    gimbal_cmd_recv.yaw -= 0.003f * (float)REMOTE_RC_RH();
    gimbal_cmd_recv.pitch -= 0.00003f * (float)REMOTE_RC_RV();
    gimbal_cmd_recv.real_pitch = ((gimbal_feedback_data.gimbal_imu_data.euler[1]) - gimbal_feedback_data.init_location) / 57.39;
}

void FoundEnermy()
{
    gimbal_cmd_recv.autoaim_mode = AUTO_ON;

    /* yaw: 自适应滤波 + 速率限幅 */
    float yaw_err = minipc_recv_data->Vision.yaw;
    float abs_err = fabsf(yaw_err);

    if (abs_err > 0.3f)
    {
        float target_yaw = gimbal_feedback_data.gimbal_imu_data.yaw_total - yaw_err;
        gimbal_cmd_recv.yaw = target_yaw;
        gimbal_cmd_recv.yaw = Cal_FollowControl_Set_Yaw(gimbal_feedback_data.gimbal_imu_data, gimbal_cmd_recv);
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

            // 误差估计的滤波强度(τ = 5ms/alpha)。
            // 注意: 0.0012 增益 + 0.7/0.5/0.4 的组合会在实车上引起 pitch 震荡(相位裕度不足),
            // 已回退到原值。要提高跟踪速度必须同时加大阻尼(MI 电机 kd), 见 CODE_REVIEW.md 调参记录。
            if (fabsf(err_delta) > 0.5f)
                alpha = 0.3f;    // 大跳变: 快速跟上
            else if (fabsf(err_delta) > 0.05f)
                alpha = 0.2f;    // 跟踪中: 中等响应
            else
                alpha = 0.1f;    // 微调: 强滤波抑制噪声

            err_filtered = alpha * raw_err + (1.0f - alpha) * err_filtered;
            err_last = raw_err;

            // 增量式控制, 增益可安全设高
            gimbal_cmd_recv.pitch += 0.0005f * err_filtered;
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
    switch (chassis_cmd_recv.chassis_mode)
    {
        //底盘跟随模式,将offset_angle量化到最近的90°作为目标(4个正方向)
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        {
            float raw = chassis_cmd_recv.offset_angle;
            float snapped;
            // 就近量化到90°的倍数 —— 4个固定正方向: 0°, ±90°, 180°/-180°
            if (raw >= 0.0f)
                snapped = (float)((int)(raw / 90.0f + 0.5f)) * 90.0f;
            else
                snapped = (float)((int)(raw / 90.0f - 0.5f)) * 90.0f;
            /* 原: chassis_cmd_send.wz = PIDCalculate(&chassis_follow_pid, raw, snapped);
               (measure=raw, ref=snapped); 对方 PID 需要显式 setSetpoint + update */
            chassis_follow_pid.setSetpoint(snapped);
            chassis_follow_pid.update(raw);
            chassis_cmd_recv.wz = chassis_follow_pid.output_;
        }
        break;
        case CHASSIS_ROTATE: // 变速小陀螺
            chassis_cmd_recv.wz = 4000 * chassis_cmd_recv.chassis_rotate_buff;
        break;
        default:
            chassis_cmd_recv.wz = 0.0;
        break;
    }
}

static void ChassisRC()
{
    chassis_cmd_recv.vx = -30.0f * (float)REMOTE_RC_LV(); // _水平方向
    chassis_cmd_recv.vy = 30.0f * (float)REMOTE_RC_LH();  // 竖直方向

    if (RC_SWITCH_LEFT() == RC_SW_VALUE_DOWN)
    {
        chassis_cmd_recv.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    else if (RC_SWITCH_LEFT() == RC_SW_VALUE_MID)
    {
        chassis_cmd_recv.chassis_mode = CHASSIS_ROTATE;
        chassis_cmd_recv.chassis_rotate_buff = 1.0f;
    }
    else if (RC_SWITCH_LEFT() == RC_SW_VALUE_UP)
    {
        chassis_cmd_recv.chassis_mode = CHASSIS_ROTATE;
        chassis_cmd_recv.chassis_rotate_buff = -1.0f;
    }
    ChassisRotateSet();
}

static void AutoAimSet()
{
    if (DataLebel.aim_flag == 1)
    {
        FoundEnermy();
        if (DataLebel.fire_flag == 1)
        {
            shoot_cmd_recv.loader_mode = LOAD_BURSTFIRE;
        }
    }
}

static void ShootRC()
{
    if (REMOTE_RC_WHEEL() > 200)
    {
        shoot_cmd_recv.loader_mode = LOAD_BURSTFIRE;
    }
    else if (REMOTE_RC_WHEEL() < -200)
    {
        shoot_cmd_recv.loader_mode = LOAD_REVERSE;
        DataLebel.reverse_flag = 1;
    }
    else
    {
        shoot_cmd_recv.loader_mode = LOAD_STOP;
        DataLebel.reverse_flag = 0;
    }
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    ChassisRC();
    gimbal_cmd_recv.autoaim_mode = AUTO_OFF;
    GimbalRC();
    ShootRC();
}

static void GetGimbalInitImu()
{
    if (mid_round_patrol.flag == 0)
    {
        mid_round_patrol.yaw_init = gimbal_feedback_data.gimbal_imu_data.euler[2]; // 原 .Yaw
        mid_round_patrol.yaw = mid_round_patrol.yaw_init;
        mid_round_patrol.flag = 1;
    }
}

static void RoundPatrol()
{
    if (round_patrol.flag == 0)
    {
        round_patrol.init_totol_round = gimbal_feedback_data.gimbal_imu_data.yaw_total / 360.0f; // 原 .YawTotalAngle
        round_patrol.flag = 1;
    }
    round_patrol.total_round = (gimbal_feedback_data.gimbal_imu_data.yaw_total / 360.0f) - round_patrol.init_totol_round;
    gimbal_cmd_recv.yaw += 0.5f;
}

static void MidRoundPatrol()
{
    if (mid_round_patrol.flag == 0)
    {
        mid_round_patrol.yaw_init = gimbal_feedback_data.gimbal_imu_data.euler[2]; // 原 .Yaw
        mid_round_patrol.yaw = mid_round_patrol.yaw_init;
        mid_round_patrol.flag = 1;
    }

    float current_relative_angle = gimbal_feedback_data.gimbal_imu_data.euler[2] - mid_round_patrol.yaw_init;
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

    gimbal_cmd_recv.yaw = current_relative_angle + round_patrol.total_round * 360.0f;
    mid_round_patrol.yaw_total_angle = current_relative_angle;
}

static void Sentry_ChassisAC()
{
    /* 视觉链路已换回本仓原本的 seasky 协议(Minipc_Recv_s), linear_velocity_x/y 与 gimbal_mode 都在,
       这里按原 robot_cmd.c 的公式与数值使用。 */
    const float vision_linear_velocity_x = minipc_recv_data->Vision.linear_velocity_x;
    const float vision_linear_velocity_y = minipc_recv_data->Vision.linear_velocity_y;
    const int32_t vision_gimbal_mode = minipc_recv_data->Vision.gimbal_mode;

    chassis_cmd_recv.vx = -vision_linear_velocity_x * 4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL * 1000.0f;
    chassis_cmd_recv.vy = -vision_linear_velocity_y * 4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL * 1000.0f;

    if (vision_gimbal_mode != 0)
    {
        chassis_cmd_recv.chassis_mode = CHASSIS_ROTATE;
        if (chassis_feedback_data.power_flag == 1)
        {
            chassis_cmd_recv.chassis_rotate_buff = 2;
        }
        else
        {
            chassis_cmd_recv.chassis_rotate_buff = 1.0;
        }
    }
    else
    {
        chassis_cmd_recv.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }
}

static void ShootAC()
{
    if (DataLebel.fire_flag == 1)
    {
        shoot_cmd_recv.loader_mode = LOAD_BURSTFIRE;
    }
    else
    {
        shoot_cmd_recv.loader_mode = LOAD_STOP;
        DataLebel.reverse_flag = 0;
    }
}

static void Sentry_GimbalAC()
{
    static MIMotor *PP_Motor; // 原 MIMotorInstance*
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
            /* 平滑进入：沿**指令自身**每周期限步长推进到 PIT（0.02rad/周期，200Hz 下约 4rad/s），
             * 推进到 PIT 后退出、转入正常巡逻。既避免停机恢复时的硬跳/撞限位，又不会把自己卡死。
             *
             * ⚠不能写成 `cmd = 实测角度 ± step`（原地版本就是这么写的）：那样位置误差被永远压在
             * 一拍步长(0.02rad)以内，kp*0.02≈0.3N·m 抬不动炮管 —— 从 C 档(零电流)切到 S 档时，
             * 炮管正被重力压在机械下限上，命令每拍都从"实测+0.02"重算，于是电机一直顶着却动不了、
             * ACEntryPoint 永远退不出、S 档 pitch 永远不上下摆动。现场读数正是这个状态：
             * pitch_cmd=-0.020(被限位夹住)、pitch_act=0.0010(实际压在下限)、torque=-0.560N·m(在出力)。 */
            float err_cmd = PIT - gimbal_cmd_recv.pitch;
            const float entry_step = 0.02f; // rad/周期
            if (fabsf(err_cmd) <= entry_step)
            {
                gimbal_cmd_recv.pitch = PIT; // 已跟上目标, 退出平滑进入稳态
                DataLebel.ACEntryPoint = 0;
            }
            else
            {
                gimbal_cmd_recv.pitch += copysignf(entry_step, err_cmd);
            }
        }
        else
        {
            gimbal_cmd_recv.pitch = PIT;
        }
        GimbalPitchLimit(); // 兜底限位(在赋值之后)
        RoundPatrol();
        gimbal_cmd_recv.autoaim_mode = AUTO_OFF;
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
    if (referee_data->robot_status.current_HP == 0) // 原 GameRobotState.current_HP
    {
        gimbal_cmd_recv.Death_reInit = 1;
    }
    else
    {
        gimbal_cmd_recv.Death_reInit = 0;
    }
}

#if defined(REMOTE_DEVICE_VT13)
/* ---------------- 键鼠控制(仅 VT13 接收机有键鼠通道) ----------------
 * 说明: 原 robot_cmd.c 里 MouseKeySet() **没有任何调用点**(只有定义), 所以下面这几个函数
 *       在当前整车逻辑里是备用的; DT7/HT10A 机型(对方 remote_config.h 默认)没有键鼠数据
 *       (dt7_rc_t 只有摇杆/开关/拨轮), 故按机型编译掉。本车旧接收机是带键鼠的 VT13,
 *       若要恢复键鼠路径, 把 remote_config.h 切到 REMOTE_DEVICE_VT13 即可。
 * TODO(移植): 若要在 DT7 上保留键鼠控制, 需要给 remote 模块补键鼠解析。 */
static void NoneAutoMouseControl()
{
    gimbal_cmd_recv.yaw -= (float)remote_data->mouse_x / 660 * 3;
    gimbal_cmd_recv.pitch += (float)remote_data->mouse_y / 660 / 57;
    if (REMOTE_MOUSE_LEFT_PRESSED())
    {
        if (DataLebel.reverse_flag == 1)
        {
            shoot_cmd_recv.loader_mode = LOAD_REVERSE;
        }
        else
        {
            shoot_cmd_recv.loader_mode = LOAD_BURSTFIRE;
        }
    }
    else
    {
        shoot_cmd_recv.loader_mode = LOAD_STOP;
    }
}

static void MouseControl()
{
    if (REMOTE_MOUSE_RIGHT_PRESSED())
    {
        /* 右键按下 → 小电脑接管云台控制 + can_fire自动开火 */
        gimbal_cmd_recv.autoaim_mode = FIND_Enermy;
        FoundEnermy();
        if (minipc_recv_data->Vision.can_fire == 1)
            shoot_cmd_recv.loader_mode = LOAD_BURSTFIRE;
        else
            shoot_cmd_recv.loader_mode = LOAD_STOP;
    }
    else
    {
        /* 右键未按下 → 手动鼠标控制 */
        gimbal_cmd_recv.autoaim_mode = AUTO_OFF;
        NoneAutoMouseControl();
    }
}

static void KeyControl()
{
    const float key_w = REMOTE_KEY_PRESSED(REMOTE_KEY_W) ? 1.0f : 0.0f;
    const float key_s = REMOTE_KEY_PRESSED(REMOTE_KEY_S) ? 1.0f : 0.0f;
    const float key_a = REMOTE_KEY_PRESSED(REMOTE_KEY_A) ? 1.0f : 0.0f;
    const float key_d = REMOTE_KEY_PRESSED(REMOTE_KEY_D) ? 1.0f : 0.0f;

    chassis_cmd_recv.vx = (key_w * 20000.0f - key_s * 20000.0f) * chassis_speed_buff;
    chassis_cmd_recv.vy = (key_d * 20000.0f - key_a * 20000.0f) * chassis_speed_buff;

    ChassisRotateSet();
    switch (referee_data->robot_status.robot_level) // 原 GameRobotState.robot_level
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

    if (chassis_feedback_data.power_flag == 1)
    {
        chassis_cmd_recv.chassis_rotate_buff = 2;
    }
    else
    {
        chassis_cmd_recv.chassis_rotate_buff = chassis_rotate_buff;
    }

    /* 原: switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) —— 按一次切换一次底盘模式。
       对方 remote 模块只给按键位图(没有 key_count 计数), 这里本地做同样的上升沿计数。 */
    static uint8_t key_r_count = 0;
    static uint8_t key_r_last = 0;
    const uint8_t key_r_now = REMOTE_KEY_PRESSED(REMOTE_KEY_R) ? 1u : 0u;
    if (key_r_now && !key_r_last)
        key_r_count++;
    key_r_last = key_r_now;

    switch (key_r_count % 2)
    {
    case 0:
        chassis_cmd_recv.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    default:
        chassis_cmd_recv.chassis_mode = CHASSIS_ROTATE;
    }

    if (REMOTE_KEY_PRESSED(REMOTE_KEY_Q))
    {
        DataLebel.reverse_flag = 1;
        shoot_cmd_recv.loader_mode = LOAD_REVERSE;
    }
    else
    {
        DataLebel.reverse_flag = 0;
    }

    // 待添加 按shift允许超功率 消耗缓冲能量
    if (REMOTE_KEY_PRESSED(REMOTE_KEY_SHIFT))
    {
        chassis_cmd_recv.chassis_speed_buff = 2;
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
#endif // REMOTE_DEVICE_VT13

/**
 * @brief 停止
 */
static void AnythingStop()
{
    gimbal_cmd_recv.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_recv.shoot_mode = SHOOT_OFF;
    shoot_cmd_recv.friction_mode = FRICTION_OFF;
    shoot_cmd_recv.loader_mode = LOAD_STOP;
    // 停止时把 pitch 指令设为当前电机角度(绝对坐标系), 而不是 0:
    // 0 落在限位区间外, 重新进入云台模式时会被夹到某一端造成跳变。
    {
        MIMotor *pm = GetPitchMotor();
        if (pm != NULL)
            gimbal_cmd_recv.pitch = pm->angle_; // 原 pm->measure.angle (rad)
    }
    DataLebel.ACEntryPoint = 1;
    //重置与小电脑通信失败的标志位
    DataLebel.cmd_error_flag = 0;
}

/**
 * @brief 控制量及模式设置
 *
 */
static void ControlDataDeal()
{
    if (RC_SWITCH_RIGHT() == RC_SW_VALUE_MID)
    {
        BasicSet();

#if defined(REMOTE_DEVICE_VT13)
        /* ---- 手动档里"遥控器(摇杆)"与"键鼠"二选一(粘滞切换) ----
         * 原 robot_cmd.c 的 MouseKeySet() 根本没有调用点(键鼠那套是死代码), 两套输入也没有互斥;
         * 现在直接把键鼠接上, 就必须解决冲突: KeyControl() 每拍都会写 chassis_cmd_recv.vx/vy 和
         * chassis_mode(R 键切换), 如果和 RemoteControlSet() 同拍都跑, 摇杆与左/右 FN 小陀螺会被覆盖。
         * 判据用对方 cmd.cpp 的 mode_is_remote 同一套: 动鼠标/按鼠标键/按键盘 → 键鼠接管;
         * 动摇杆 → 切回遥控器。默认(上电不动)是遥控器。 */
        if (remote_data->mouse_x != 0 || remote_data->mouse_y != 0 || remote_data->key != 0 ||
            REMOTE_MOUSE_LEFT_PRESSED() || REMOTE_MOUSE_RIGHT_PRESSED() || REMOTE_MOUSE_MIDDLE_PRESSED())
        {
            cmd_mode_is_remote = 0;
        }
        else if (REMOTE_RC_LH() != 0 || REMOTE_RC_RH() != 0 || REMOTE_RC_LV() != 0 || REMOTE_RC_RV() != 0)
        {
            cmd_mode_is_remote = 1;
        }

        if (cmd_mode_is_remote)
            RemoteControlSet();   // 摇杆: 左摇杆底盘 / 右摇杆云台 / 拨轮发射 / FN 选小陀螺方向
        else
            MouseKeySet();        // 键鼠: 鼠标云台+左右键开火/自瞄 / WASD 底盘 / R 切模式 / Q 反转 / SHIFT 提速
#else
        RemoteControlSet();
#endif
    }
    else if (RC_SWITCH_RIGHT() == RC_SW_VALUE_UP)
    {
        BasicSet();
        SentrySet(); // 哨兵自动模式
    }
    else if (RC_SWITCH_RIGHT() == RC_SW_VALUE_DOWN)
    {
        AnythingStop();
    }
    else
    {
        /* 落到这里 = 开关值不在 UP/MID/DOWN 三个已知档位里。
           旧 DBUS 接收机的三档就是 1/2/3, 三个分支必中其一, 所以这本来是死分支;
           VT13 的 `mode_sw` 是 C/N/S = 0/1/2, 正常也不会落到这里(C/N/S 都已映射),
           留着兜底成停机: 否则三个分支都不进, 指令会保持上一拍的值(车会按旧指令继续跑)。 */
        AnythingStop();
    }
    /* TODO(移植): 原版没有整车级遥控掉线保护(遥控失联由 gimbal/chassis 内部的 cmd_error_flag
       与超时各自处理), 判据保持原样未加保护; 若要在 cmd 层加, 可用 remote.h 的 Remote_Online()
       门控本函数。 */
}

/* 从"0 电流/停止"回到云台控制档时, 把绝对角度指令重置为当前实际姿态。
 * 不这样做的话: gimbal_cmd_recv.yaw/pitch 是跨模式保留的绝对指令, 若在 0 电流模式下用手转动了云台,
 * 重新开控制档时云台会回头去追那个过期指令(表现为"yaw 自己大幅旋转"), 而不是就近跟目标。
 * 同时清掉自适应滤波的历史状态, 避免从旧值缓慢爬向新目标。 */
static void GimbalCmdReinitOnModeEntry(void)
{
    static gimbal_mode_e last_mode = GIMBAL_ZERO_FORCE;

    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE && last_mode != GIMBAL_GYRO_MODE)
    {
        gimbal_cmd_recv.yaw = gimbal_feedback_data.gimbal_imu_data.yaw_total; // 以当前 IMU 偏航角为起点(原 .YawTotalAngle)
        MIMotor *pm = GetPitchMotor();
        if (pm != NULL)
            gimbal_cmd_recv.pitch = pm->angle_; // pitch 以当前电机角度为起点(原 pm->measure.angle)
        GimbalAlgorithmReset();
        DataLebel.ACEntryPoint = 1; // 自动模式用平滑进入
        LOG_INFO(LOG_MOD_SM, "cmd", "[cmd] gimbal re-enabled: yaw cmd reset to %.1f, pitch cmd=%.3f\r\n",
                 (double)gimbal_cmd_recv.yaw, (double)gimbal_cmd_recv.pitch);
    }
    last_mode = gimbal_cmd_recv.gimbal_mode;
}

/* 敌我颜色: 把"敌方"颜色告诉视觉(自己是蓝方 id>7 → 敌方是红; 自己是红方 → 敌方是蓝)。
   原 robot_cmd.c 用 referee_data->GameRobotState.robot_id, 对方裁判模块里对应 id.robot_id。
   写的缓冲就是模块发出去的那一份(1kHz/200Hz 共用)。 */
static void EnemyJudge()
{
    if (referee_data->id.robot_id > 7)
    {
        Minipc_GetSendData()->Vision.detect_color = COLOR_RED;   // 自己是蓝方, 敌方红
    }
    else
    {
        Minipc_GetSendData()->Vision.detect_color = COLOR_BLUE;  // 自己是红方, 敌方蓝
    }
}

/* 把三个 cmd 实例里的模式同步给裁判 UI 状态(ui_data 定义在 application/ui/ui.cpp,
   原来是把指针交给 UITaskInit(); 现在 UI 结构与绘制都归 application/ui/) */
static void SendToUIData()
{
    ui_data.autoaim_mode = gimbal_cmd_recv.autoaim_mode;
    ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;
    ui_data.loader_mode = shoot_cmd_recv.loader_mode;
    ui_data.shoot_mode = shoot_cmd_recv.shoot_mode;
    ui_data.gimbal_mode = gimbal_cmd_recv.gimbal_mode;
}

/* 把裁判系统/底盘的信息填进视觉发送帧。**原 seasky 实现里这些字段没有任何赋值点**
   （一直是 0），这里按字段含义接上，映射如下：
     self_sentry_hp   ← game_robot_HP.ally_7_robot_HP   (7 号 = 哨兵)
     self_hero_hp     ← game_robot_HP.ally_1_robot_HP   (1 号 = 英雄)
     self_infantry_hp ← game_robot_HP.ally_3_robot_HP   (3 号步兵; 新版协议只发 1~4 + 7)
     remain_time      ← game_state.stage_remain_time
     remain_bullet    ← projectile_allowance.projectile_allowance_17mm (哨兵用 17mm)
     match_progress   ← game_state.game_progress (4bit 位域)
     bullet_speed     ← shoot_data.initial_speed
     vx/vy            ← 见下面"用源代码的逻辑"的说明 */
static void VisionSetMatchData()
{
    Minipc_Send_s *tx = Minipc_GetSendData();

    tx->Vision.self_sentry_hp = referee_data->game_robot_HP.ally_7_robot_HP;
    tx->Vision.self_hero_hp = referee_data->game_robot_HP.ally_1_robot_HP;
    tx->Vision.self_infantry_hp = referee_data->game_robot_HP.ally_3_robot_HP;
    tx->Vision.remain_time = referee_data->game_state.stage_remain_time;
    tx->Vision.remain_bullet = referee_data->projectile_allowance.projectile_allowance_17mm;
    tx->Vision.match_progress = (uint8_t)referee_data->game_state.game_progress;
    tx->Vision.bullet_speed = referee_data->shoot_data.initial_speed;

    /* vx/vy: 用**源代码里那条换算**的逆运算。
       源代码(Sentry_ChassisAC)把视觉下发的速度换进底盘指令域:
           chassis_cmd_recv.vx = -linear_velocity_x * 4 * REDUCTION_RATIO_WHEEL * 360 / PERIMETER_WHEEL * 1000
       这里取它的逆, 于是发回给视觉的 vx/vy 与它下发的 linear_velocity 是**同一个量纲/同一路坐标系**
       (云台系, 即 chassis_cmd_recv.vx/vy 所在的域):
           Vision.vx = -chassis_cmd_recv.vx / (4 * REDUCTION_RATIO_WHEEL * 360 / PERIMETER_WHEEL * 1000)
       注意两点(要改就是这一处):
         1) 用的是**指令速度** chassis_cmd_recv.vx/vy, 不是轮子实测速度;
         2) 视觉自己开车时(Sentry_ChassisAC)这两个值就是它刚下发的指令, 会原样回给它们;
            手动模式下则是摇杆/键鼠算出来的指令速度。 */
    const float kVisionVelScale = 4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL * 1000.0f;
    tx->Vision.vx = -chassis_cmd_recv.vx / kVisionVelScale;
    tx->Vision.vy = -chassis_cmd_recv.vy / kVisionVelScale;

    /* occupation (占领状态): 裁判协议里没有"占领"字段(robot_pos 只有 x/y/angle, event_data 是位域),
       没有可靠来源 → 保持 0。 */
}

/* ==================== 云台跟踪调试采样(黑匣子) ====================
 * 目的: 定位"动目标跟不上"到底是电流饱和、结构性问题还是增益不足。
 * 只读数据, 不改任何控制量; 采满后自动冻结(RAM 里留 128 拍触发前 + 128 拍触发后), 断电/复位即清空。
 * 用 OpenOCD 读 g_vt_head / g_vt_frozen / g_vt_buf 即可取出分析。
 */
#define VT_LEN 256
typedef struct
{
    uint32_t frame_cnt; // 视觉帧计数(与上一拍不同 => 本拍收到了新帧)
    float cmd_yaw;      // 视觉目标角(度)
    float yaw_imu;      // IMU 总偏航角(度)
    float gyro_z;       // 陀螺 z 轴
    float vis_err;      // 视觉偏航误差(度)
    float ang_out;      // yaw 角度环输出(= 速度环参考)
    int16_t cur_out;    // yaw 速度环输出(= 电流指令, 饱和就看它)
    uint16_t can_fire;
    /* --- pitch 标定用: 测出 电机坐标系 <-> IMU/重力坐标系 的方向与偏置 --- */
    float pitch_cmd;    // gimbal_cmd_recv.pitch (rad, 现有代码按电机坐标系使用)
    float pitch_motor;  // MI 电机角度 measure.angle (rad)
    float pitch_imu;    // IMU pitch (rad, 重力参考)
    float yaw_motor_speed; // yaw 电机转速 (deg/s, 云台相对底盘的角速度)
    float yaw_motor_angle; // yaw 电机角度 (deg, 云台相对底盘)
    float pad_;            // 对齐
} vt_sample_t;

static vt_sample_t vt_buf[VT_LEN];
volatile uint32_t g_vt_head = 0;   // 已写入样本数(环形)
volatile uint8_t g_vt_frozen = 0;     // 1 = 已冻结, 可以 dump
volatile uint8_t g_vt_freeze_req = 0; // 调试用: 由 OpenOCD 写 1 即冻结, 保留最近 256 拍
static uint32_t vt_post_cnt = 0;

static void VisionTraceSample(void)
{
    if (g_vt_frozen)
        return;
    if (g_vt_freeze_req) // 手动冻结(OpenOCD 写 g_vt_freeze_req=1)
    {
        g_vt_frozen = 1;
        return;
    }

    MIMotor *pm = GetPitchMotor(); // 原 MIMotorInstance*
    DJIMotor *ym = GetYawMotor();  // 原 DJIMotorInstance*
    vt_sample_t *s = &vt_buf[g_vt_head % VT_LEN];

    /* 视觉帧计数: 用本仓 seasky 模块的 g_vision_frame_cnt(每次成功解帧 +1), 与原实现一致 */
    s->frame_cnt = g_vision_frame_cnt;
    s->cmd_yaw = gimbal_cmd_recv.yaw;
    s->yaw_imu = gimbal_feedback_data.gimbal_imu_data.yaw_total;    // 原 .YawTotalAngle
    s->gyro_z = gimbal_feedback_data.gimbal_imu_data.gyro_b[2];     // 原 .Gyro[2]
    s->vis_err = minipc_recv_data->Vision.yaw;
    /* TODO(移植): yaw 的角度/速度环按规约 §6 已移到应用层(gimbal.cpp), 对方 DJIMotor 的 PID 又是
       私有成员, 这里取不到 angle_PID/speed_PID 的 Output; 需要时由 gimbal.cpp 暴露只读接口。 */
    s->ang_out = 0.0f;
    s->cur_out = 0;
    s->can_fire = (uint16_t)minipc_recv_data->Vision.can_fire;
    s->pitch_cmd = gimbal_cmd_recv.pitch;
    s->pitch_motor = (pm != NULL) ? pm->angle_ : 0.0f;
    s->pitch_imu = gimbal_feedback_data.gimbal_imu_data.euler[1] * DEGREE_2_RAD; // 原 .Pitch
    /* TODO(移植): 对方 DJIMotor 只有 angle_(deg)/velocity_(deg/s), 没有 total_angle;
       yaw_motor_angle 用 angle_ 顶替(单圈/累计语义不同, 分析时注意)。 */
    s->yaw_motor_speed = ym->velocity_; // 原 ym->measure.speed_aps
    s->yaw_motor_angle = ym->angle_;
    g_vt_head++;

    if (vt_post_cnt == 0 && fabsf(s->vis_err) > 5.0f && g_vt_head > 64)
        vt_post_cnt = 128; // 触发: 误差 >5° 说明正在追动目标, 再录 128 拍后冻结
    else if (vt_post_cnt > 0 && --vt_post_cnt == 0)
        g_vt_frozen = 1;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    /* 原 SubGetMessage(chassis_feed_sub/shoot_feed_sub/gimbal_feed_sub, ...) 三行删除:
       chassis/gimbal/shoot 的反馈现在是各应用 .cpp 里定义的全局实例
       (chassis_feedback_data/gimbal_feedback_data/shoot_feedback_data), cmd 直接读。 */

    Deathcheck();

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    ControlDataDeal();
    GimbalCmdReinitOnModeEntry();

    /* 原 PubPushMessage(chassis_cmd_pub/shoot_cmd_pub/gimbal_cmd_pub, ...) 三行删除:
       上面写的 chassis_cmd_recv/gimbal_cmd_recv/shoot_cmd_recv 就是底盘/云台/发射直接读的实例。 */

    EnemyJudge();
    VisionSetMatchData();
    SendMinipcData(NULL);                    // 视觉链路: 本仓 seasky 协议, 发模块那份(与 1kHz 路径同一份)

    SendToUIData();
    VisionTraceSample();
}
