#ifndef BSP_IWDG_H
#define BSP_IWDG_H

#include <stdint.h>

/* 独立看门狗(IWDG)封装
 *
 * 背景: 本工程此前完全没有启用看门狗(Inc/stm32f4xx_hal_conf.h 里 HAL_IWDG_MODULE_ENABLED 是注释掉的,
 *       仓库里也没有 stm32f4xx_hal_iwdg.c), 因此程序一旦跑飞/任务卡死, MCU 不会复位,
 *       电机会保持最后一条 CAN 指令继续输出。IWDG 由 LSI 独立时钟驱动, 一旦使能软件无法关闭,
 *       是这个场景下唯一的硬件兜底。
 *
 * 本封装直接用寄存器操作, 不引入 ST 的 HAL IWDG 驱动源文件。
 * 超时 ≈ (reload + 1) * 64 / f_LSI; f_LSI 典型 32kHz, 有容差, 因此 reload=BSP_IWDG_RELOAD_1S 时
 * 典型约 1.0s(最坏情况约 0.68s ~ 1.9s)。
 */
#define BSP_IWDG_RELOAD_1S 500u

/* 启动看门狗(使能后无法关闭)。reload 见上面的超时公式。 */
void BSPIWDGInit(uint16_t reload);

/* 喂狗(重装载计数)。喂狗节奏由上层决定, 见 application/robot_task.h 的心跳判定。 */
void BSPIWDGFeed(void);

/* 打印并清除"上次复位原因"标志(放在日志系统初始化之后调用) */
void BSPIWDGLogResetCause(void);

#endif /* BSP_IWDG_H */
