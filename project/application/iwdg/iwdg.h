/**
 * @file    iwdg.h
 * @brief   独立看门狗(IWDG): 纯安全网, 不参与控制。三个控制任务心跳都到齐才喂。
 * @note    本仓没有 HAL 的 IWDG 驱动(Drivers 下无 stm32f4xx_hal_iwdg.c,
 *          Inc/stm32f4xx_hal_conf.h 里 HAL_IWDG_MODULE_ENABLED 是注释状态),
 *          所以这里按 HAL 的时序手写寄存器, 不动 Drivers / 生成文件。
 */
#ifndef IWDG_H
#define IWDG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 三个控制任务的心跳(定义在 robot.cpp 里): 各自任务循环内置 1, DAEMON 喂狗后清零 */
extern volatile uint8_t iwdg_alive_ins;
extern volatile uint8_t iwdg_alive_motor;
extern volatile uint8_t iwdg_alive_robot;

/**
 * @brief 启动 IWDG 并喂第一次; LSI≈32kHz / 64 分频 / 重载 200 ⇒ 标称超时 ≈400ms
 * @note  IWDG 一旦启动无法关闭, 必须在所有长耗时初始化(IMU 预热)之后才调用
 */
void IWDG_Init(void);

/**
 * @brief 喂狗(KR=0xAAAA)
 */
void IWDG_Feed(void);

#ifdef __cplusplus
}
#endif

#endif /* IWDG_H */
