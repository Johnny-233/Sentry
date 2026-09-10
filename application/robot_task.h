/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "master_process.h"
#include "daemon.h"
#include "buzzer.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_iwdg.h"

/* ---- 看门狗心跳 ----
 * 每个持续运行的任务在自己的循环里置位; daemon 任务(100Hz)确认三个控制任务都推进过才喂狗,
 * 任一控制任务卡死 -> 1s 内不再喂狗 -> IWDG 硬件复位。UI 任务不参与(它卡住不应导致整车复位)。
 */
static volatile uint8_t iwdg_alive_ins = 0;
static volatile uint8_t iwdg_alive_motor = 0;
static volatile uint8_t iwdg_alive_robot = 0;

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
    // 后续修改为读取传感器数据准备好的中断处理,

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 1024);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    uiTaskHandle = osThreadCreate(osThread(uitask), NULL);
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init(); // 确保BMI088被正确初始化.
    LOGINFO("[freeRTOS] INS Task Start");
    for (;;)
    {
        // 1kHz
        iwdg_alive_ins = 1; // 心跳: 供 daemon 任务判定本任务是否卡死
        ins_start = DWT_GetTimeline_ms();
        INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;
        if (ins_dt > 1)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]", &ins_dt);
        SendMinipcData(NULL); // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {
        iwdg_alive_motor = 1; // 心跳
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;)
    {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        BuzzerTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 10)
            LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);

        // 看门狗: 三个控制任务在上一个 10ms 周期内都有推进才喂狗。
        // 任一任务卡死 -> 停止喂狗 -> 约 1s 后 IWDG 复位(reload 见 bsp_iwdg.h)。
        if (iwdg_alive_ins && iwdg_alive_motor && iwdg_alive_robot)
        {
            BSPIWDGFeed();
        }
        else
        {
            static uint8_t stall_log_cnt = 0; // 限频打印, 避免刷屏把 RTT 堵死
            if ((stall_log_cnt++ % 20u) == 0u)
                LOGERROR("[iwdg] task stall! ins=%d motor=%d robot=%d -> NOT feeding (reset in ~1s)",
                         (int)iwdg_alive_ins, (int)iwdg_alive_motor, (int)iwdg_alive_robot);
        }
        iwdg_alive_ins = 0;
        iwdg_alive_motor = 0;
        iwdg_alive_robot = 0;

        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    static float robot_dt;
    static float robot_start;
    LOGINFO("[freeRTOS] ROBOT core Task Start");
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        iwdg_alive_robot = 1; // 心跳
        robot_start = DWT_GetTimeline_ms();
        RobotTask();
        robot_dt = DWT_GetTimeline_ms() - robot_start;
        if (robot_dt > 5)
            LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%f]", &robot_dt);
        osDelay(5);
    }
}

__attribute__((noreturn)) void StartUITASK(void const *argument)
{
    LOGINFO("[freeRTOS] UI Task Start");
    MyUIInit();
    LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
    for (;;)
    {
        // 每给裁判系统发送一包数据会挂起一次,详见UITask函数的refereeSend()
        UITask();
        osDelay(1); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
    }
}
