/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 应用层入口: application/ 的整车控制逻辑(C → C++ 移植版, 见 application/PORT_MAPPING.md)
   用 extern "C" 提供 Robot_Init() / Robot_Task()。 */
#define SENTRY_APP_LAYER 1

#ifdef SENTRY_APP_LAYER
#include "robot.h"
#endif
#include "usb_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  /* CMSIS-RTOS2 的 stack_size 单位是字节: 256*4 = 1KB。
     Cmd_Task 里有浮点运算和多层调用, 比 CubeMX 默认的 128*4 留一倍余量 */
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
/* configSUPPORT_STATIC_ALLOCATION=1 时内核用静态内存建 Idle 任务, 需要这里给内存。
   (CMSIS-RTOS2 的 cmsis_os2.c 里另有 __WEAK 版本, 这里保留 CubeMX 生成的强定义) */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* C++ 应用层入口。必须在 osKernelInitialize() 之后调用, 因为里面要用
     CMSIS-RTOS2 的 osThreadNew 建任务 (见 project/modules/* 的 init 约定)。 */
#ifdef SENTRY_APP_LAYER
  Robot_Init();
#endif
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* 这里**不再**调用 Robot_Task()。
   * 阶段 3 时应用层还没有自己的任务划分, 所以把它挂在 defaultTask 上按 1kHz 跑;
   * 阶段 4 的 application/robot.cpp 已经建了自己的 5 个任务(INS/MOTOR/ROBOT/DAEMON/UI),
   * 其中 ROBOT 任务(200Hz)负责 Robot_Task()。若这里再调一遍:
   *   1) 整套应用逻辑会被跑两次(cmd/chassis/gimbal/shoot 的全局指令被两个任务同时写, 有竞争);
   *   2) defaultTask 只有 1KB 栈(见上面的 defaultTask_attributes), 而 Robot_Task() 里是完整的
   *      控制逻辑(浮点解算 + 日志格式化), **必然栈溢出**, 会踩坏紧邻的 FreeRTOS 堆/TCB
   *      —— 实测现象就是跑一秒左右在调度器里 HardFault(pxCurrentTCB 被踩成 NULL /
   *      xTaskIncrementTick 里精确总线错误)。
   * 所以 defaultTask 现在只负责 USB 初始化和空转。 */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 任务栈溢出钩子(configCHECK_FOR_STACK_OVERFLOW = 2 时由内核调用)。
 * 弱定义在 cmsis_os2.c 里是 configASSERT(0) 死等, 这里换成: 记下任务名 → 停机。
 * 调试时用 gdb/Ozone 读 g_stack_overflow_task 就知道是哪个任务栈给小了。
 * (正常运行时不应触发; 触发说明某个任务的 stack_size 需要调大。) */
#if (configCHECK_FOR_STACK_OVERFLOW > 0)
volatile char g_stack_overflow_task[configMAX_TASK_NAME_LEN] = {0};
volatile uint32_t g_stack_overflow_cnt = 0;

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
  (void)xTask;
  for (uint32_t i = 0; i < configMAX_TASK_NAME_LEN; i++)
    g_stack_overflow_task[i] = (char)pcTaskName[i];
  g_stack_overflow_cnt++;
  taskDISABLE_INTERRUPTS();
  for (;;) { }
}
#endif

/* USER CODE END Application */
