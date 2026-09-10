#include "bsp_iwdg.h"
#include "main.h"
#include "bsp_log.h"

/* IWDG 键值(见参考手册 IWDG_KR) */
#define IWDG_KEY_RELOAD 0x0000AAAAu
#define IWDG_KEY_ENABLE 0x0000CCCCu
#define IWDG_KEY_WRITE_ACCESS_ENABLE 0x00005555u
/* 预分频 = 64 (PR = 0b100) */
#define IWDG_PRESCALER_DIV64 0x04u

void BSPIWDGInit(uint16_t reload)
{
    /* 1. 打开 LSI(看门狗时钟源)并等待稳定; 带计数上限, 避免在这里死等 */
    RCC->CSR |= RCC_CSR_LSION;
    for (uint32_t i = 0; i < 100000u; ++i)
    {
        if (RCC->CSR & RCC_CSR_LSIRDY)
            break;
    }
    if ((RCC->CSR & RCC_CSR_LSIRDY) == 0u)
    {
        LOGERROR("[iwdg] LSI not ready, watchdog NOT started");
        return;
    }

    /* 2. 调试器把内核停住(断点/单步)时冻结看门狗计数, 否则调试中会被反复复位
     * (STM32F4 的 DBGMCU 没有独立时钟门控, 直接写寄存器即可; __HAL_RCC_DBGMCU_CLK_ENABLE 只在 F7/H7 存在) */
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    /* 3. 使能并配置(使能后软件无法关闭) */
    IWDG->KR = IWDG_KEY_ENABLE;
    IWDG->KR = IWDG_KEY_WRITE_ACCESS_ENABLE;
    IWDG->PR = IWDG_PRESCALER_DIV64;
    IWDG->RLR = reload;
    while (IWDG->SR != 0u) /* 等待 PVU/RVU 更新完成 */
        ;
    IWDG->KR = IWDG_KEY_RELOAD;

    LOGINFO("[iwdg] watchdog started, reload=%u (typical ~1.0s)", (unsigned)reload);
}

void BSPIWDGFeed(void)
{
    IWDG->KR = IWDG_KEY_RELOAD;
}

void BSPIWDGLogResetCause(void)
{
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
        LOGWARNING("[iwdg] last reset was caused by IWDG (watchdog timeout or task stall)");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
        LOGWARNING("[iwdg] last reset was caused by WWDG");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
        LOGWARNING("[iwdg] last reset was caused by software reset");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
        LOGINFO("[iwdg] last reset was power-on reset");
    __HAL_RCC_CLEAR_RESET_FLAGS();
}
