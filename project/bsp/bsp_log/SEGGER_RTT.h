#ifndef SEGGER_RTT_H
#define SEGGER_RTT_H

#include "SEGGER_RTT_Conf.h"

#ifndef RTT_USE_ASM
  #if (defined __CROSSWORKS_ARM)
    #define _CC_HAS_RTT_ASM_SUPPORT 1
    #if (defined __ARM_ARCH_7M__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 1
    #elif (defined __ARM_ARCH_7EM__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 1
      #define _CORE_NEEDS_DMB           1
      #define RTT__DMB() __asm volatile ("dmb\n" : : :);
    #elif (defined __ARM_ARCH_8M_BASE__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 0
      #define _CORE_NEEDS_DMB           1
      #define RTT__DMB() __asm volatile ("dmb\n" : : :);
    #elif (defined __ARM_ARCH_8M_MAIN__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 1
      #define _CORE_NEEDS_DMB           1
      #define RTT__DMB() __asm volatile ("dmb\n" : : :);
    #else
      #define _CORE_HAS_RTT_ASM_SUPPORT 0
    #endif
  #elif (defined __ARMCC_VERSION)
    #if (__ARMCC_VERSION >= 6000000)
      #define _CC_HAS_RTT_ASM_SUPPORT 1
    #else
      #define _CC_HAS_RTT_ASM_SUPPORT 0
    #endif
    #if (defined __ARM_ARCH_6M__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 0
    #elif (defined __ARM_ARCH_7M__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 1
    #elif (defined __ARM_ARCH_7EM__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 1
      #define _CORE_NEEDS_DMB           1
      #define RTT__DMB() __asm volatile ("dmb\n" : : :);
    #elif (defined __ARM_ARCH_8M_BASE__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 0
      #define _CORE_NEEDS_DMB           1
      #define RTT__DMB() __asm volatile ("dmb\n" : : :);
    #elif (defined __ARM_ARCH_8M_MAIN__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 1
      #define _CORE_NEEDS_DMB           1
      #define RTT__DMB() __asm volatile ("dmb\n" : : :);
    #elif ((defined __ARM_ARCH_7A__) || (defined __ARM_ARCH_7R__))
      #define _CORE_NEEDS_DMB           1
      #define RTT__DMB() __asm volatile ("dmb\n" : : :);
    #else
      #define _CORE_HAS_RTT_ASM_SUPPORT 0
    #endif
  #elif ((defined __GNUC__) || (defined __clang__))
    #define _CC_HAS_RTT_ASM_SUPPORT 1
    #if (defined __ARM_ARCH_7M__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 1
    #elif (defined __ARM_ARCH_7EM__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 1
      #define _CORE_NEEDS_DMB           1
      #define RTT__DMB() __asm volatile ("dmb\n" : : :);
    #elif (defined __ARM_ARCH_8M_BASE__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 0
      #define _CORE_NEEDS_DMB           1
      #define RTT__DMB() __asm volatile ("dmb\n" : : :);
    #elif (defined __ARM_ARCH_8M_MAIN__)
      #define _CORE_HAS_RTT_ASM_SUPPORT 1
      #define _CORE_NEEDS_DMB           1
      #define RTT__DMB() __asm volatile ("dmb\n" : : :);
    #elif ((defined __ARM_ARCH_7A__) || (defined __ARM_ARCH_7R__))
      #define _CORE_NEEDS_DMB           1
      #define RTT__DMB() __asm volatile ("dmb\n" : : :);
    #else
      #define _CORE_HAS_RTT_ASM_SUPPORT 0
    #endif
  #elif ((defined __IASMARM__) || (defined __ICCARM__))
    #define _CC_HAS_RTT_ASM_SUPPORT 1
    #if (__VER__ < 6300000)
      #define VOLATILE
    #else
      #define VOLATILE volatile
    #endif
    #if (defined __ARM7M__)
      #if (__CORE__ == __ARM7M__)
        #define _CORE_HAS_RTT_ASM_SUPPORT 1
      #endif
    #endif
    #if (defined __ARM7EM__)
      #if (__CORE__ == __ARM7EM__)
        #define _CORE_HAS_RTT_ASM_SUPPORT 1
        #define _CORE_NEEDS_DMB 1
        #define RTT__DMB() asm VOLATILE ("DMB");
      #endif
    #endif
    #if (defined __ARM8M_BASELINE__)
      #if (__CORE__ == __ARM8M_BASELINE__)
        #define _CORE_HAS_RTT_ASM_SUPPORT 0
        #define _CORE_NEEDS_DMB 1
        #define RTT__DMB() asm VOLATILE ("DMB");
      #endif
    #endif
    #if (defined __ARM8M_MAINLINE__)
      #if (__CORE__ == __ARM8M_MAINLINE__)
        #define _CORE_HAS_RTT_ASM_SUPPORT 1
        #define _CORE_NEEDS_DMB 1
        #define RTT__DMB() asm VOLATILE ("DMB");
      #endif
    #endif
    #if (defined __ARM8EM_MAINLINE__)
      #if (__CORE__ == __ARM8EM_MAINLINE__)
        #define _CORE_HAS_RTT_ASM_SUPPORT 1
        #define _CORE_NEEDS_DMB 1
        #define RTT__DMB() asm VOLATILE ("DMB");
      #endif
    #endif
    #if (defined __ARM7A__)
      #if (__CORE__ == __ARM7A__)
        #define _CORE_NEEDS_DMB 1
        #define RTT__DMB() asm VOLATILE ("DMB");
      #endif
    #endif
    #if (defined __ARM7R__)
      #if (__CORE__ == __ARM7R__)
        #define _CORE_NEEDS_DMB 1
        #define RTT__DMB() asm VOLATILE ("DMB");
      #endif
    #endif
  #else
    #define _CC_HAS_RTT_ASM_SUPPORT   0
    #define _CORE_HAS_RTT_ASM_SUPPORT 0
  #endif
  #ifndef _CORE_HAS_RTT_ASM_SUPPORT
    #define _CORE_HAS_RTT_ASM_SUPPORT 0
  #endif
  #if (_CC_HAS_RTT_ASM_SUPPORT && _CORE_HAS_RTT_ASM_SUPPORT)
    #define RTT_USE_ASM                           (1)
  #else
    #define RTT_USE_ASM                           (0)
  #endif
#endif

#ifndef _CORE_NEEDS_DMB
  #define _CORE_NEEDS_DMB 0
#endif

#ifndef RTT__DMB
  #if _CORE_NEEDS_DMB
    #error "Don't know how to place inline assembly for DMB"
  #else
    #define RTT__DMB()
  #endif
#endif

#ifndef SEGGER_RTT_CPU_CACHE_LINE_SIZE
  #define SEGGER_RTT_CPU_CACHE_LINE_SIZE (0)
#endif

#ifndef SEGGER_RTT_UNCACHED_OFF
  #if SEGGER_RTT_CPU_CACHE_LINE_SIZE
    #error "SEGGER_RTT_UNCACHED_OFF must be defined when setting SEGGER_RTT_CPU_CACHE_LINE_SIZE != 0"
  #else
    #define SEGGER_RTT_UNCACHED_OFF (0)
  #endif
#endif
#if RTT_USE_ASM
  #if SEGGER_RTT_CPU_CACHE_LINE_SIZE
    #error "RTT_USE_ASM is not available if SEGGER_RTT_CPU_CACHE_LINE_SIZE != 0"
  #endif
#endif

#ifndef SEGGER_RTT_ASM
#include <stdlib.h>
#include <stdarg.h>

#if SEGGER_RTT_CPU_CACHE_LINE_SIZE
  #define SEGGER_RTT__ROUND_UP_2_CACHE_LINE_SIZE(NumBytes) (((NumBytes + SEGGER_RTT_CPU_CACHE_LINE_SIZE - 1) / SEGGER_RTT_CPU_CACHE_LINE_SIZE) * SEGGER_RTT_CPU_CACHE_LINE_SIZE)
#else
  #define SEGGER_RTT__ROUND_UP_2_CACHE_LINE_SIZE(NumBytes) (NumBytes)
#endif
#define SEGGER_RTT__CB_SIZE                              (16 + 4 + 4 + (SEGGER_RTT_MAX_NUM_UP_BUFFERS * 24) + (SEGGER_RTT_MAX_NUM_DOWN_BUFFERS * 24))
#define SEGGER_RTT__CB_PADDING                           (SEGGER_RTT__ROUND_UP_2_CACHE_LINE_SIZE(SEGGER_RTT__CB_SIZE) - SEGGER_RTT__CB_SIZE)

typedef struct {
  const     char*    sName;
            char*    pBuffer;
            unsigned SizeOfBuffer;
            unsigned WrOff;
  volatile  unsigned RdOff;
            unsigned Flags;
} SEGGER_RTT_BUFFER_UP;

typedef struct {
  const     char*    sName;
            char*    pBuffer;
            unsigned SizeOfBuffer;
  volatile  unsigned WrOff;
            unsigned RdOff;
            unsigned Flags;
} SEGGER_RTT_BUFFER_DOWN;

typedef struct {
  char                    acID[16];
  int                     MaxNumUpBuffers;
  int                     MaxNumDownBuffers;
  SEGGER_RTT_BUFFER_UP    aUp[SEGGER_RTT_MAX_NUM_UP_BUFFERS];
  SEGGER_RTT_BUFFER_DOWN  aDown[SEGGER_RTT_MAX_NUM_DOWN_BUFFERS];
#if SEGGER_RTT__CB_PADDING
  unsigned char           aDummy[SEGGER_RTT__CB_PADDING];
#endif
} SEGGER_RTT_CB;

extern SEGGER_RTT_CB _SEGGER_RTT;

#ifdef __cplusplus
  extern "C" {
#endif
int          SEGGER_RTT_AllocDownBuffer         (const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int          SEGGER_RTT_AllocUpBuffer           (const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int          SEGGER_RTT_ConfigUpBuffer          (unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int          SEGGER_RTT_ConfigDownBuffer        (unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int          SEGGER_RTT_GetKey                  (void);
unsigned     SEGGER_RTT_HasData                 (unsigned BufferIndex);
int          SEGGER_RTT_HasKey                  (void);
unsigned     SEGGER_RTT_HasDataUp               (unsigned BufferIndex);
void         SEGGER_RTT_Init                    (void);
unsigned     SEGGER_RTT_Read                    (unsigned BufferIndex,       void* pBuffer, unsigned BufferSize);
unsigned     SEGGER_RTT_ReadNoLock              (unsigned BufferIndex,       void* pData,   unsigned BufferSize);
int          SEGGER_RTT_SetNameDownBuffer       (unsigned BufferIndex, const char* sName);
int          SEGGER_RTT_SetNameUpBuffer         (unsigned BufferIndex, const char* sName);
int          SEGGER_RTT_SetFlagsDownBuffer      (unsigned BufferIndex, unsigned Flags);
int          SEGGER_RTT_SetFlagsUpBuffer        (unsigned BufferIndex, unsigned Flags);
int          SEGGER_RTT_WaitKey                 (void);
unsigned     SEGGER_RTT_Write                   (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_WriteNoLock             (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_WriteSkipNoLock         (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_ASM_WriteSkipNoLock     (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_WriteString             (unsigned BufferIndex, const char* s);
void         SEGGER_RTT_WriteWithOverwriteNoLock(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_PutChar                 (unsigned BufferIndex, char c);
unsigned     SEGGER_RTT_PutCharSkip             (unsigned BufferIndex, char c);
unsigned     SEGGER_RTT_PutCharSkipNoLock       (unsigned BufferIndex, char c);
unsigned     SEGGER_RTT_GetAvailWriteSpace      (unsigned BufferIndex);
unsigned     SEGGER_RTT_GetBytesInBuffer        (unsigned BufferIndex);

#define      SEGGER_RTT_HASDATA(n)       (((SEGGER_RTT_BUFFER_DOWN*)((char*)&_SEGGER_RTT.aDown[n] + SEGGER_RTT_UNCACHED_OFF))->WrOff - ((SEGGER_RTT_BUFFER_DOWN*)((char*)&_SEGGER_RTT.aDown[n] + SEGGER_RTT_UNCACHED_OFF))->RdOff)

#if RTT_USE_ASM
  #define SEGGER_RTT_WriteSkipNoLock  SEGGER_RTT_ASM_WriteSkipNoLock
#endif

unsigned     SEGGER_RTT_ReadUpBuffer            (unsigned BufferIndex, void* pBuffer, unsigned BufferSize);
unsigned     SEGGER_RTT_ReadUpBufferNoLock      (unsigned BufferIndex, void* pData, unsigned BufferSize);
unsigned     SEGGER_RTT_WriteDownBuffer         (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned     SEGGER_RTT_WriteDownBufferNoLock   (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);

#define      SEGGER_RTT_HASDATA_UP(n)    (((SEGGER_RTT_BUFFER_UP*)((char*)&_SEGGER_RTT.aUp[n] + SEGGER_RTT_UNCACHED_OFF))->WrOff - ((SEGGER_RTT_BUFFER_UP*)((char*)&_SEGGER_RTT.aUp[n] + SEGGER_RTT_UNCACHED_OFF))->RdOff)

int     SEGGER_RTT_SetTerminal        (unsigned char TerminalId);
int     SEGGER_RTT_TerminalOut        (unsigned char TerminalId, const char* s);

int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);
int SEGGER_RTT_vprintf(unsigned BufferIndex, const char * sFormat, va_list * pParamList);

#ifdef __cplusplus
  }
#endif

#endif

#define SEGGER_RTT_MODE_NO_BLOCK_SKIP         (0)
#define SEGGER_RTT_MODE_NO_BLOCK_TRIM         (1)
#define SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL    (2)
#define SEGGER_RTT_MODE_MASK                  (3)

#define RTT_CTRL_RESET                "\x1B[0m"
#define RTT_CTRL_CLEAR                "\x1B[2J"

#define RTT_CTRL_TEXT_BLACK           "\x1B[2;30m"
#define RTT_CTRL_TEXT_RED             "\x1B[2;31m"
#define RTT_CTRL_TEXT_GREEN           "\x1B[2;32m"
#define RTT_CTRL_TEXT_YELLOW          "\x1B[2;33m"
#define RTT_CTRL_TEXT_BLUE            "\x1B[2;34m"
#define RTT_CTRL_TEXT_MAGENTA         "\x1B[2;35m"
#define RTT_CTRL_TEXT_CYAN            "\x1B[2;36m"
#define RTT_CTRL_TEXT_WHITE           "\x1B[2;37m"

#define RTT_CTRL_TEXT_BRIGHT_BLACK    "\x1B[1;30m"
#define RTT_CTRL_TEXT_BRIGHT_RED      "\x1B[1;31m"
#define RTT_CTRL_TEXT_BRIGHT_GREEN    "\x1B[1;32m"
#define RTT_CTRL_TEXT_BRIGHT_YELLOW   "\x1B[1;33m"
#define RTT_CTRL_TEXT_BRIGHT_BLUE     "\x1B[1;34m"
#define RTT_CTRL_TEXT_BRIGHT_MAGENTA  "\x1B[1;35m"
#define RTT_CTRL_TEXT_BRIGHT_CYAN     "\x1B[1;36m"
#define RTT_CTRL_TEXT_BRIGHT_WHITE    "\x1B[1;37m"

#define RTT_CTRL_BG_BLACK             "\x1B[24;40m"
#define RTT_CTRL_BG_RED               "\x1B[24;41m"
#define RTT_CTRL_BG_GREEN             "\x1B[24;42m"
#define RTT_CTRL_BG_YELLOW            "\x1B[24;43m"
#define RTT_CTRL_BG_BLUE              "\x1B[24;44m"
#define RTT_CTRL_BG_MAGENTA           "\x1B[24;45m"
#define RTT_CTRL_BG_CYAN              "\x1B[24;46m"
#define RTT_CTRL_BG_WHITE             "\x1B[24;47m"

#define RTT_CTRL_BG_BRIGHT_BLACK      "\x1B[4;40m"
#define RTT_CTRL_BG_BRIGHT_RED        "\x1B[4;41m"
#define RTT_CTRL_BG_BRIGHT_GREEN      "\x1B[4;42m"
#define RTT_CTRL_BG_BRIGHT_YELLOW     "\x1B[4;43m"
#define RTT_CTRL_BG_BRIGHT_BLUE       "\x1B[4;44m"
#define RTT_CTRL_BG_BRIGHT_MAGENTA    "\x1B[4;45m"
#define RTT_CTRL_BG_BRIGHT_CYAN       "\x1B[4;46m"
#define RTT_CTRL_BG_BRIGHT_WHITE      "\x1B[4;47m"

#endif
