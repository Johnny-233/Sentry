#ifndef BSP_LOG_H
#define BSP_LOG_H

#include "bsp_log_port.h"
#include "bsp_log_config.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LOG_LVL_DBG  = 0,
    LOG_LVL_INFO = 1,
    LOG_LVL_ERR  = 2,
} LogLevel_e;

#define X(bit, name, label) LOG_MOD_##name = (1u << bit),
typedef enum {
    LOG_MODULE_LIST
} LogModule_e;
#undef X

#define X(bit, name, desc) FAULT_##name = (1u << bit),
typedef enum {
    FAULT_LIST
    FAULT_COUNT
} FaultBit_e;
#undef X

extern volatile uint8_t  g_log_level;
extern volatile uint16_t g_log_modules;
extern volatile uint32_t g_faults;

void BSP_LogInit(void);
void Fault_Set(uint32_t bit);
void Fault_Clr(uint32_t bit);
const char* fault_name(uint32_t bit);
const char* bsp_log_mod_name(LogModule_e mod);

void _log_write(LogLevel_e level, LogModule_e mod, const char *who,
                const char *fmt, ...);

#define LOG_DBG(mod, who, fmt, ...) \
    _log_write(LOG_LVL_DBG, mod, who, fmt, ##__VA_ARGS__)

#define LOG_INFO(mod, who, fmt, ...) \
    _log_write(LOG_LVL_INFO, mod, who, fmt, ##__VA_ARGS__)

#define LOG_ERR(mod, who, fmt, ...) \
    _log_write(LOG_LVL_ERR, mod, who, fmt, ##__VA_ARGS__)

#define LOG_ASSERT(cond, mod, who, fmt, ...)                     \
    do {                                                         \
        if (!(cond)) {                                           \
            _log_write(LOG_LVL_ERR, mod, who,                    \
                "ASSERT %s FAILED | " fmt,                       \
                #cond, ##__VA_ARGS__);                           \
        }                                                        \
    } while (0)

#define LOG_ASSERT_RET(cond, mod, who, fmt, ...)                 \
    do {                                                         \
        if (!(cond)) {                                           \
            _log_write(LOG_LVL_ERR, mod, who,                    \
                "ASSERT %s FAILED | " fmt,                       \
                #cond, ##__VA_ARGS__);                           \
            return;                                              \
        }                                                        \
    } while (0)

#define LOG_ASSERT_RETVAL(cond, val, mod, who, fmt, ...)         \
    do {                                                         \
        if (!(cond)) {                                           \
            _log_write(LOG_LVL_ERR, mod, who,                    \
                "ASSERT %s FAILED | " fmt,                       \
                #cond, ##__VA_ARGS__);                           \
            return (val);                                        \
        }                                                        \
    } while (0)

#define FMT_FLOAT(val, decimals)                                     \
    (int)(val),                                                      \
    (int)(((val) - (int)(val)) * _fmt_float_pow10(decimals))

static inline int _fmt_float_pow10(int n)
{
    static const int tab[] = {1, 10, 100, 1000, 10000, 100000};
    if (n < 0) n = 0;
    if (n > 5) n = 5;
    return tab[n];
}

#define FMT_FLOAT_ABS(val, decimals)                                 \
    (int)((val) < 0 ? -(int)(val) : (int)(val)),                     \
    (int)(_fmt_float_frac_abs(val, decimals))

static inline int _fmt_float_frac_abs(float val, int decimals)
{
    float frac = val - (int)val;
    if (frac < 0) frac = -frac;
    return (int)(frac * _fmt_float_pow10(decimals));
}

#ifdef __cplusplus
}
#endif

#endif
