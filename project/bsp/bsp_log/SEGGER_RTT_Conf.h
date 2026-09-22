#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

/* CMake 只编 .c/.cpp，没有 SEGGER_RTT_ASM 汇编；走 C 实现即可 */
#ifndef RTT_USE_ASM
#define RTT_USE_ASM 0
#endif

#ifdef __IAR_SYSTEMS_ICC__
  #include <intrinsics.h>

#endif

#ifndef   SEGGER_RTT_MAX_NUM_UP_BUFFERS
  #define SEGGER_RTT_MAX_NUM_UP_BUFFERS             (3)
#endif

#ifndef   SEGGER_RTT_MAX_NUM_DOWN_BUFFERS
  #define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS           (3)
#endif

#ifndef   BUFFER_SIZE_UP
  /* 512：这根 DAPLink 大块 DAP_TRANSFER_BLOCK 会崩，4096 易在 pyocd rtt 读缓冲时掉线 */
  #define BUFFER_SIZE_UP                            (512)
#endif

#ifndef   BUFFER_SIZE_DOWN
  #define BUFFER_SIZE_DOWN                          (16)
#endif

#ifndef   SEGGER_RTT_PRINTF_BUFFER_SIZE
  #define SEGGER_RTT_PRINTF_BUFFER_SIZE             (256u)
#endif

#ifndef   SEGGER_RTT_MODE_DEFAULT
  #define SEGGER_RTT_MODE_DEFAULT                   SEGGER_RTT_MODE_NO_BLOCK_SKIP
#endif

#ifndef   SEGGER_RTT_MEMCPY_USE_BYTELOOP
  #define SEGGER_RTT_MEMCPY_USE_BYTELOOP              0
#endif

#ifndef   SEGGER_RTT_MAX_INTERRUPT_PRIORITY
  #define SEGGER_RTT_MAX_INTERRUPT_PRIORITY         (0x20)
#endif

#if ((defined(__SES_ARM) || defined(__SES_RISCV) || defined(__CROSSWORKS_ARM) || defined(__GNUC__) || defined(__clang__)) && !defined (__CC_ARM) && !defined(WIN32))
  #if (defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_8M_BASE__))
    #define SEGGER_RTT_LOCK()   {                                                                   \
                                    unsigned int _SEGGER_RTT__LockState;                                         \
                                  __asm volatile ("mrs   %0, primask  \n\t"                         \
                                                  "movs  r1, #1       \n\t"                         \
                                                  "msr   primask, r1  \n\t"                         \
                                                  : "=r" (_SEGGER_RTT__LockState)                                \
                                                  :                                                 \
                                                  : "r1", "cc"                                      \
                                                  );

    #define SEGGER_RTT_UNLOCK()   __asm volatile ("msr   primask, %0  \n\t"                         \
                                                  :                                                 \
                                                  : "r" (_SEGGER_RTT__LockState)                                 \
                                                  :                                                 \
                                                  );                                                \
                                }
  #elif (defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_8M_MAIN__))
    #ifndef   SEGGER_RTT_MAX_INTERRUPT_PRIORITY
      #define SEGGER_RTT_MAX_INTERRUPT_PRIORITY   (0x20)
    #endif
    #define SEGGER_RTT_LOCK()   {                                                                   \
                                    unsigned int _SEGGER_RTT__LockState;                                         \
                                  __asm volatile ("mrs   %0, basepri  \n\t"                         \
                                                  "mov   r1, %1       \n\t"                         \
                                                  "msr   basepri, r1  \n\t"                         \
                                                  : "=r" (_SEGGER_RTT__LockState)                                \
                                                  : "i"(SEGGER_RTT_MAX_INTERRUPT_PRIORITY)          \
                                                  : "r1", "cc"                                      \
                                                  );

    #define SEGGER_RTT_UNLOCK()   __asm volatile ("msr   basepri, %0  \n\t"                         \
                                                  :                                                 \
                                                  : "r" (_SEGGER_RTT__LockState)                                 \
                                                  :                                                 \
                                                  );                                                \
                                }
  #else
    #define SEGGER_RTT_LOCK()
    #define SEGGER_RTT_UNLOCK()
  #endif
#endif

#ifdef __CC_ARM
  #if (defined __TARGET_ARCH_6S_M)
    #define SEGGER_RTT_LOCK()   {                                                                   \
                                  unsigned int _SEGGER_RTT__LockState;                                           \
                                  register unsigned char _SEGGER_RTT__PRIMASK __asm( "primask");                 \
                                  _SEGGER_RTT__LockState = _SEGGER_RTT__PRIMASK;                                              \
                                  _SEGGER_RTT__PRIMASK = 1u;                                                     \
                                  __schedule_barrier();

    #define SEGGER_RTT_UNLOCK()   _SEGGER_RTT__PRIMASK = _SEGGER_RTT__LockState;                                              \
                                  __schedule_barrier();                                             \
                                }
  #elif (defined(__TARGET_ARCH_7_M) || defined(__TARGET_ARCH_7E_M))
    #ifndef   SEGGER_RTT_MAX_INTERRUPT_PRIORITY
      #define SEGGER_RTT_MAX_INTERRUPT_PRIORITY   (0x20)
    #endif
    #define SEGGER_RTT_LOCK()   {                                                                   \
                                  unsigned int _SEGGER_RTT__LockState;                                           \
                                  register unsigned char BASEPRI __asm( "basepri");                 \
                                  _SEGGER_RTT__LockState = BASEPRI;                                              \
                                  BASEPRI = SEGGER_RTT_MAX_INTERRUPT_PRIORITY;                      \
                                  __schedule_barrier();

    #define SEGGER_RTT_UNLOCK()   BASEPRI = _SEGGER_RTT__LockState;                                              \
                                  __schedule_barrier();                                             \
                                }
  #endif
#endif

#ifdef __ICCARM__
  #if (defined (__ARM6M__)          && (__CORE__ == __ARM6M__))             ||                      \
      (defined (__ARM8M_BASELINE__) && (__CORE__ == __ARM8M_BASELINE__))
    #define SEGGER_RTT_LOCK()   {                                                                   \
                                  unsigned int _SEGGER_RTT__LockState;                                           \
                                  _SEGGER_RTT__LockState = __get_PRIMASK();                                      \
                                  __set_PRIMASK(1);

    #define SEGGER_RTT_UNLOCK()   __set_PRIMASK(_SEGGER_RTT__LockState);                                         \
                                }
  #elif (defined (__ARM7EM__)         && (__CORE__ == __ARM7EM__))          ||                      \
        (defined (__ARM7M__)          && (__CORE__ == __ARM7M__))           ||                      \
        (defined (__ARM8M_MAINLINE__) && (__CORE__ == __ARM8M_MAINLINE__))
    #ifndef   SEGGER_RTT_MAX_INTERRUPT_PRIORITY
      #define SEGGER_RTT_MAX_INTERRUPT_PRIORITY   (0x20)
    #endif
    #define SEGGER_RTT_LOCK()   {                                                                   \
                                  unsigned int _SEGGER_RTT__LockState;                                           \
                                  _SEGGER_RTT__LockState = __get_BASEPRI();                                      \
                                  __set_BASEPRI(SEGGER_RTT_MAX_INTERRUPT_PRIORITY);

    #define SEGGER_RTT_UNLOCK()   __set_BASEPRI(_SEGGER_RTT__LockState);                                         \
                                }
  #endif
#endif

#ifndef   SEGGER_RTT_LOCK
  #define SEGGER_RTT_LOCK()
#endif

#ifndef   SEGGER_RTT_UNLOCK
  #define SEGGER_RTT_UNLOCK()
#endif


#endif
