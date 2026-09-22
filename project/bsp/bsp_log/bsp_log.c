#include "bsp_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

volatile uint8_t  g_log_level   = BSP_LOG_DEFAULT_LEVEL;
volatile uint16_t g_log_modules = BSP_LOG_DEFAULT_MODULES;
volatile uint32_t g_faults      = 0;

static char log_buf[BSP_LOG_BUF_SIZE];

static const char* level_str(LogLevel_e level)
{
    switch (level) {
        case LOG_LVL_DBG:  return "DBG ";
        case LOG_LVL_INFO: return "INFO";
        case LOG_LVL_ERR:  return "ERR ";
        default:           return "??? ";
    }
}

__attribute__((weak))
const char* bsp_log_mod_name(LogModule_e mod)
{
    switch (mod) {
#define X(bit, name, label) case LOG_MOD_##name: return label;
        LOG_MODULE_LIST
#undef X
        default: return "??? ";
    }
}

const char* fault_name(uint32_t bit)
{
    switch (bit) {
#define X(bit, name, desc) case FAULT_##name: return desc;
        FAULT_LIST
#undef X
        default: return "unknown";
    }
}

void Fault_Set(uint32_t bit)
{
    uint32_t old = g_faults;
    g_faults |= bit;
    if (!(old & bit)) {
        _log_write(LOG_LVL_ERR, LOG_MOD_SYS, "fault",
                   "%s SET   | faults=0x%04lX",
                   fault_name(bit), g_faults);
    }
}

void Fault_Clr(uint32_t bit)
{
    uint32_t old = g_faults;
    g_faults &= ~bit;
    if (old & bit) {
        _log_write(LOG_LVL_ERR, LOG_MOD_SYS, "fault",
                   "%s CLEAR | faults=0x%04lX",
                   fault_name(bit), g_faults);
    }
}

void _log_write(LogLevel_e level, LogModule_e mod, const char *who,
                const char *fmt, ...)
{
    if (level == LOG_LVL_DBG || level == LOG_LVL_INFO) {
        if (level < g_log_level) return;
        if (!(mod & g_log_modules)) return;
    }

    uint32_t tick = bsp_log_get_tick_ms();
    int head_len = snprintf(log_buf, BSP_LOG_BUF_SIZE,
                            "[%7lu.%03lu][%-4s][%-4s][%-5s] ",
                            (unsigned long)(tick / 1000),
                            (unsigned long)(tick % 1000),
                            level_str(level),
                            bsp_log_mod_name(mod),
                            who);

    if (head_len < 0 || head_len >= BSP_LOG_BUF_SIZE) {
        head_len = 0;
    }

    va_list args;
    va_start(args, fmt);
    int body_len = vsnprintf(log_buf + head_len,
                             BSP_LOG_BUF_SIZE - head_len - 2,
                             fmt, args);
    va_end(args);

    if (body_len < 0) {
        body_len = 0;
    }

    int total = head_len + body_len;
    if (total < BSP_LOG_BUF_SIZE - 2) {
        log_buf[total++] = '\r';
        log_buf[total++] = '\n';
        log_buf[total]   = '\0';
    } else {
        log_buf[BSP_LOG_BUF_SIZE - 3] = '\r';
        log_buf[BSP_LOG_BUF_SIZE - 2] = '\n';
        log_buf[BSP_LOG_BUF_SIZE - 1] = '\0';
    }

    BSP_LOG_LOCK();
    BSP_LOG_OUTPUT(log_buf);
    BSP_LOG_UNLOCK();
}

void BSP_LogInit(void)
{
    SEGGER_RTT_Init();

    g_log_level   = BSP_LOG_DEFAULT_LEVEL;
    g_log_modules = BSP_LOG_DEFAULT_MODULES;
    g_faults      = 0;

    LOG_INFO(LOG_MOD_SYS, "boot",
             "========================================");
    LOG_INFO(LOG_MOD_SYS, "boot",
             "BSP_LOG init | platform=%s", BSP_LOG_PLATFORM);
    LOG_INFO(LOG_MOD_SYS, "boot",
             "level=%d modules=0x%04X buf=%d",
             g_log_level, g_log_modules, BSP_LOG_BUF_SIZE);
    LOG_INFO(LOG_MOD_SYS, "boot",
             "========================================");
}
