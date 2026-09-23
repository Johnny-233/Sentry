/**
 * @file    robot.cpp
 * @brief   整车入口与任务划分
 * @note    由 application/robot.c + application/robot_task.h 移植（C → C++）。
 *
 * 与原 C 版的差异（都是"底层换成对方框架"导致的，逐条列在这里方便对照）：
 *   1. BSPInit() 里的 DWT_Init(168)/BSPLogInit() → 对方的 DWT_Init()/BSP_LogInit()；
 *      BuzzerInit() 对方框架没有 → 不做（TODO）。
 *   2. 多了 DJIMotor::timbaseSelect(&htim5)：对方的电机模块把速度/位置环跑在 htim5 中断里，
 *      必须在所有电机 init 之后调用一次（对应原来 motor_task.c 的 1kHz 控制循环）。
 *   3. 原 MOTOR 任务里的 MotorControlTask() 已由上面那条接管，本任务只保留心跳与超时打印。
 *   4. 原 INS 任务里的 INS_Init()/INS_Task() → g_ahrs.init()（在 RobotCMDInit 里）+ g_ahrs.start()
 *      （对方 AHRS 自带解算任务）；本任务只保留 1kHz 发视觉数据与心跳。
 *      视觉链路用回本仓原本的 seasky 协议（`project/modules/master_machine/`），
 *      发送就是原 `SendMinipcData(NULL)`，与 `robot_task.h` 的 INS 任务完全一致。
 *   5. MOTOR 任务里补回了原 `motor_task.c` 的 MI 电机周期重发：`if(cnt%10==0) MiMotorControl();`。
 *   5. 原 DaemonTask() 是全局 daemon 链表的轮询；对方框架每个模块自带 Daemon 实例（挂在 TIM 上），
 *      所以本任务不再轮询，只做"控制任务卡死检测"打印。
 *   6. IWDG 暂不做（对方框架没有 IWDG 模块）：原来卡死会硬件复位，现在只打印。
 */
#include "robot.h"
#include "robot_def.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#include "ui.h"

#include "ahrs.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_tim.h"
#include "dji_motor.h"
#include "mi_motor.h"
#include "remote.h"
#include "master_process.h"
#include "referee.h"

#include "cmsis_os.h"
#include "tim.h"

/* ---- 看门狗心跳(原 robot_task.h 的静态变量, 见上方差异 6) ---- */
static volatile uint8_t iwdg_alive_ins = 0;
static volatile uint8_t iwdg_alive_motor = 0;
static volatile uint8_t iwdg_alive_robot = 0;

/* ---- 任务句柄 ---- */
static osThreadId_t insTaskHandle;
static osThreadId_t robotTaskHandle;
static osThreadId_t motorTaskHandle;
static osThreadId_t daemonTaskHandle;
static osThreadId_t uiTaskHandle;

/* ============================================================================
 *  INS 任务: 1kHz
 *  原版: INS_Init(); 循环 { INS_Task(); SendMinipcData(NULL); osDelay(1); }
 *  现版: 姿态解算在 g_ahrs 自己的任务里, 本任务只负责每 1ms 把视觉数据发出去
 * ==========================================================================*/
static void StartINSTASK(void *argument)
{
    (void)argument;
    float ins_start, ins_dt;
    LOG_INFO(LOG_MOD_INS, "ins", "[freeRTOS] INS Task Start\r\n");
    for (;;)
    {
        iwdg_alive_ins = 1; // 心跳: 供 daemon 任务判定本任务是否卡死
        ins_start = DWT_GetTimeline_ms();

        /* 视觉数据: 与原 robot_task.h 的 INS 任务一致, 每 1ms 发一帧。
           发送数据只有一份(模块持有的那份), 与 200Hz 的 RobotCMDTask 共用 —— 所以
           detect_color/姿态等每帧都是最新的, 不会出现"1kHz 那帧颜色一直是 0"的老问题。 */
        SendMinipcData(NULL);

        ins_dt = DWT_GetTimeline_ms() - ins_start;
        if (ins_dt > 1)
            LOG_ERR(LOG_MOD_INS, "ins", "[freeRTOS] INS Task is being DELAY! dt = [%d]\r\n", (int)ins_dt);
        osDelay(1);
    }
}

/* ============================================================================
 *  MOTOR 任务: 1kHz
 *  电机闭环已由 DJIMotor 的 htim5 中断接管, 这里只保留心跳与超时打印
 * ==========================================================================*/
/* 调试用：MOTOR 任务实际周期(DWT 周期数，168MHz 下 1ms=168000)，确认 1kHz 没被别的任务饿死 */
volatile uint32_t g_motor_task_dcyc = 0;
volatile uint32_t g_motor_task_maxdcyc = 0;

static void StartMOTORTASK(void *argument)
{
    (void)argument;
    float motor_dt, motor_start;
    LOG_INFO(LOG_MOD_MOTOR, "motor", "[freeRTOS] MOTOR Task Start\r\n");
    for (;;)
    {
        iwdg_alive_motor = 1; // 心跳
        motor_start = DWT_GetTimeline_ms();

        {
            static uint32_t last_task_cyc = 0;
            uint32_t now_task_cyc = DWT->CYCCNT;
            if (last_task_cyc != 0)
            {
                g_motor_task_dcyc = now_task_cyc - last_task_cyc;
                if (g_motor_task_dcyc > g_motor_task_maxdcyc)
                    g_motor_task_maxdcyc = g_motor_task_dcyc;
            }
            last_task_cyc = now_task_cyc;
        }

        /* 1ms 时基：本板 TIM5 是 PWM 配置、NVIC 没开 TIM5_IRQn，框架原来那条硬件中断是断的，
           所以用 1kHz 的 MOTOR 任务合成这个 tick，喂各 Daemon/Serial 的超时（周期都配 1ms）。 */
        USER_TIM_PeriodElapsedCallback(&htim5);

        /* 电机环：旧固件是 `if(cnt%5==0) DJIMotorControl();` 即 200Hz；框架原本挂在 1ms 中断上，
           采样率×5 会让噪声整段进 PID（电流抖/顶 MaxOut）。为保留旧 PID 数字，这里按 5ms 调，
           chassis.cpp / shoot.cpp 的 pid_velocity.period 也填 5ms。 */
        static uint8_t motor_cnt = 0;
        if (++motor_cnt >= 5)
        {
            motor_cnt = 0;
            DJIMotor::taskUpdate();
        }

        /* MI 电机重发：旧固件 `if(cnt%10==0) MiMotorControl();` = 100Hz；收不到帧它会自己掉使能 */
        static uint8_t mi_cnt = 0;
        if (++mi_cnt >= 10)
        {
            mi_cnt = 0;
            MiMotorControl();
        }

        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOG_ERR(LOG_MOD_MOTOR, "motor", "[freeRTOS] MOTOR Task is being DELAY! dt = [%d]\r\n", (int)motor_dt);
        osDelay(1);
    }
}

/* ============================================================================
 *  DAEMON 任务: 100Hz
 * ==========================================================================*/
static void StartDAEMONTASK(void *argument)
{
    (void)argument;
    float daemon_dt, daemon_start;
    LOG_INFO(LOG_MOD_SYS, "daemon", "[freeRTOS] Daemon Task Start\r\n");
    for (;;)
    {
        daemon_start = DWT_GetTimeline_ms();
        /* TODO(移植): 原版在这里跑全局 DaemonTask() 与 BuzzerTask(); 对方框架的离线检测
           在各模块自己的 Daemon 实例里(挂 TIM), 蜂鸣器没有对应模块。 */

#if defined(SENTRY_APP_LAYER) || 1
        /* ---- 链路自愈(每 500ms 一次) ----
         * HAL 的 UART DMA 接收有概率死锁(DMA 不再搬数据, SR 里 RXNE 挂着, HAL 仍报 BUSY):
         * 一旦发生, 遥控/视觉的帧就冻结在最后一帧, 车不再响应。对方框架的 Daemon 只在
         * "掉线那一刻"回调一次, 救不回来(而且它自己也可能卡在 HAL_BUSY), 所以这里周期性
         * 重开一次串口接收 —— 与旧 C 固件在每个离线回调里调 USARTServiceInit 的兜底同一目的。 */
        {
            static uint8_t recover_cnt = 0;
            if (++recover_cnt >= 50)
            {
                recover_cnt = 0;
                if (!Remote_Online())
                    Remote_Recover();
                if (!Minipc_Online())
                    Minipc_Recover();
            }
        }
#endif
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 10)
            LOG_ERR(LOG_MOD_SYS, "daemon", "[freeRTOS] Daemon Task is being DELAY! dt = [%d]\r\n", (int)daemon_dt);

        /* 控制任务卡死检测(原来喂 IWDG, 见文件头差异 6) */
        if (iwdg_alive_ins && iwdg_alive_motor && iwdg_alive_robot)
        {
            /* 三个控制任务都推进过 */
        }
        else
        {
            static uint8_t stall_log_cnt = 0; // 限频打印, 避免刷屏把 RTT 堵死
            if ((stall_log_cnt++ % 20u) == 0u)
                LOG_ERR(LOG_MOD_SYS, "daemon",
                        "[stall] task stall! ins=%d motor=%d robot=%d\r\n",
                        (int)iwdg_alive_ins, (int)iwdg_alive_motor, (int)iwdg_alive_robot);
        }
        iwdg_alive_ins = 0;
        iwdg_alive_motor = 0;
        iwdg_alive_robot = 0;

        osDelay(10);
    }
}

/* ============================================================================
 *  ROBOT 核心任务: 200Hz(5ms)
 * ==========================================================================*/
static void StartROBOTTASK(void *argument)
{
    (void)argument;
    float robot_dt, robot_start;
    LOG_INFO(LOG_MOD_SYS, "robot", "[freeRTOS] ROBOT core Task Start\r\n");
    for (;;)
    {
        iwdg_alive_robot = 1; // 心跳
        robot_start = DWT_GetTimeline_ms();
        Robot_Task();
        robot_dt = DWT_GetTimeline_ms() - robot_start;
        if (robot_dt > 5)
            LOG_ERR(LOG_MOD_SYS, "robot", "[freeRTOS] ROBOT core Task is being DELAY! dt = [%d]\r\n", (int)robot_dt);
        osDelay(5);
    }
}

/* ============================================================================
 *  UI 任务
 * ==========================================================================*/
static void StartUITASK(void *argument)
{
    (void)argument;
    LOG_INFO(LOG_MOD_SYS, "ui", "[freeRTOS] UI Task Start\r\n");
    MyUIInit();
    LOG_INFO(LOG_MOD_SYS, "ui", "[freeRTOS] UI Init Done, communication with ref has established\r\n");
    for (;;)
    {
        /* 每给裁判系统发送一包数据会挂起一次,详见 UITask 的 refereeSend() */
        UITask();
        osDelay(1); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
    }
}

/* ============================================================================
 *  构造 InitStructure 并创建任务(原 OSTaskInit)
 *  注意: osThreadAttr_t.stack_size 单位是**字节**, 原 osThreadDef 的第 5 个参数是**字**,
 *        所以这里都乘 4。
 * ==========================================================================*/
static void OSTaskInit()
{
    /* INS 任务只做: 置心跳 + 每 1ms 发一帧视觉数据(SendMinipcData 用的是静态缓冲)。
       姿态解算在 AHRS 自己的任务里, 所以这里不需要大栈。
       原版 1024 字(4KB)是照抄旧代码; 但对方的任务栈是从 20KB 的 FreeRTOS 堆里切的,
       我们这几个 4KB 栈已把堆用到只剩 368B, 这里降到 1KB 留出余量
       (实测够用; 若以后往 INS 任务里加逻辑, 记得同步调大)。 */
    const osThreadAttr_t ins_attr = {
        .name = "instask", .stack_size = 512 * 4, .priority = (osPriority_t)osPriorityAboveNormal,
    };
    const osThreadAttr_t motor_attr = {
        .name = "motortask", .stack_size = 1024 * 4, .priority = (osPriority_t)osPriorityNormal,
    };
    const osThreadAttr_t daemon_attr = {
        .name = "daemontask", .stack_size = 256 * 4, .priority = (osPriority_t)osPriorityNormal,
    };
    const osThreadAttr_t robot_attr = {
        .name = "robottask", .stack_size = 1024 * 4, .priority = (osPriority_t)osPriorityNormal,
    };
    const osThreadAttr_t ui_attr = {
        .name = "uitask", .stack_size = 512 * 4, .priority = (osPriority_t)osPriorityNormal,
    };

    insTaskHandle = osThreadNew(StartINSTASK, nullptr, &ins_attr);
    motorTaskHandle = osThreadNew(StartMOTORTASK, nullptr, &motor_attr);
    daemonTaskHandle = osThreadNew(StartDAEMONTASK, nullptr, &daemon_attr);
    robotTaskHandle = osThreadNew(StartROBOTTASK, nullptr, &robot_attr);
    uiTaskHandle = osThreadNew(StartUITASK, nullptr, &ui_attr);
}

/* ============================================================================
 *  对外入口
 * ==========================================================================*/
void Robot_Init(void)
{
    /* 原 C 版这里用 __disable_irq() / __enable_irq() 把初始化整个包起来，原因是
     * "初始化过程中不要被中断打断"。**本移植版必须去掉它**，因为对方框架的
     * `AHRS::init()` 内部会 `preheat()`：它用 `HAL_GetTick()` + `HAL_Delay()` 等 IMU 升温，
     * 而 HAL 时基是 TIM14 中断（见 Src/stm32f4xx_hal_timebase_tim.c）——
     * 关着中断调它就是死循环（tick 永远不涨，预热永不超时），开机直接卡死。
     * 原 C 版能在关中断的情况下初始化，是因为它的 `INS_Init()` 是在 INS 任务里调的
     * （见 application/robot_task.h 的 StartINSTASK），预热也在任务里跑；移植后 IMU 初始化
     * 提前到了这里（对方框架的 `AHRS::init()` 自带预热+校准+启动）。
     * 对方的 Robot_Init 也是全程开中断的。 */
    __enable_irq();

    DWT_Init();
    BSP_LogInit();
    /* TODO(移植): 原 BSPInit() 还会 BuzzerInit(), 对方框架没有蜂鸣器模块 */

    RobotCMDInit();
    GimbalInit();
    ShootInit();
    ChassisInit();

    /* 对方的电机模块把闭环跑在 htim5 中断里, 必须在所有电机注册之后调一次 */
    DJIMotor::timbaseSelect(&htim5);

    OSTaskInit();
}

void Robot_Task(void)
{
    RobotCMDTask();
    GimbalTask();
    ShootTask();
    ChassisTask();
}
