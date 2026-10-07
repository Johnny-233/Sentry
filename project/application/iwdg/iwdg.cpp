/**
 * @file    iwdg.cpp
 * @brief   手写 IWDG 寄存器时序(等价 HAL_IWDG_Init), 见 iwdg.h
 * @note    只访问 IWDG(0x40003000) 与 DBGMCU(0xE0042000) 两个外设。
 */
#include "iwdg.h"

#include "stm32f4xx_hal.h"

/* IWDG 键值(RM0090 21.4.1) */
#define IWDG_KEY_START 0x0000CCCCu        /* 启动, LSI 自动打开 */
#define IWDG_KEY_WRITE_ENABLE 0x00005555u /* 允许写 PR/RLR */
#define IWDG_KEY_RELOAD 0x0000AAAAu       /* 喂狗: 重装计数器 */

/* 参数: LSI≈32kHz, 64 分频, 重载 200 ⇒ 标称超时 = 200 * 64 / 32000 = 400ms */
#define IWDG_PRESCALER_DIV64 4u
#define IWDG_RELOAD_400MS 200u

/* PR/RLR 影子寄存器同步等待上界(LSI 异常时不至于死等) */
#define IWDG_SYNC_TIMEOUT 0x000FFFFFu

/* 等 PVU/RVU 清零: 之后写 PR/RLR 才会生效 */
static void IWDG_WaitSync(void)
{
    for (uint32_t t = 0; t < IWDG_SYNC_TIMEOUT; ++t)
    {
        if ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) == 0u)
            return;
    }
}

void IWDG_Init(void)
{
    /* 第一步冻结: 调试器 halt 内核时看门狗暂停, 否则断点/单步期间会被咬复位 */
    __HAL_DBGMCU_FREEZE_IWDG();

    IWDG->KR = IWDG_KEY_START;        /* 0xCCCC */
    IWDG->KR = IWDG_KEY_WRITE_ENABLE; /* 0x5555 */

    IWDG_WaitSync(); /* 等 PVU/RVU 清零才能写 PR/RLR */
    IWDG->PR = IWDG_PRESCALER_DIV64;
    IWDG->RLR = IWDG_RELOAD_400MS;
    IWDG_WaitSync(); /* 等新分频/重载值生效 */

    IWDG->KR = IWDG_KEY_RELOAD; /* 0xAAAA: 启动后立刻喂一次 */
}

void IWDG_Feed(void)
{
    IWDG->KR = IWDG_KEY_RELOAD;
}
