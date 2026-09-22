# DWT 时钟模块

## 概述

DWT（Data Watchpoint and Trace）是 ARM Cortex-M 内核的硬件外设，提供高精度的 cycle counter。本模块封装了 DWT 的常用功能，提供微秒级延时和时间测量。

## 为什么用 DWT 而不是 HAL_Delay

| 特性 | HAL_Delay | DWT_Delay |
|------|-----------|-----------|
| 精度 | 1ms（基于 SysTick） | ~6ns（基于 CPU 时钟） |
| 阻塞方式 | 中断驱动，可被抢占 | 忙等待，不可抢占 |
| 适用场景 | 粗粒度延时 | 高精度延时、时间测量 |
| 对中断的影响 | 不影响 | 延时期间中断被延迟 |

**结论**：需要精确计时或高精度延时时用 DWT，普通延时用 HAL_Delay。

## 工作机制

### 硬件原理

```
CPU 时钟 (168MHz)
    ↓
DWT->CYCCNT 寄存器（32位，每周期+1）
    ↓
溢出处理（32位回绕，软件扩展到64位）
    ↓
换算系数：1秒 = 168,000,000 cycles
         1毫秒 = 168,000 cycles
         1微秒 = 168 cycles
```

### 溢出处理

CYCCNT 是 32 位寄存器，最大计数约 25.5 秒（@168MHz）。本模块通过软件扩展到 64 位：

1. 每次读取时检查是否发生回绕（当前值 < 上次值）
2. 回绕时溢出计数器 +1
3. 64 位时间 = (溢出次数 << 32) | CYCCNT

## 初始化

```c
#include "bsp_dwt.h"

void Robot_Init(void)
{
    DWT_Init();  // 必须最先调用！
    // ... 其他初始化 ...
}
```

**必须在所有使用 DWT 的模块之前调用**，包括：
- AHRS（使用 DWT_GetDeltaT 和 DWT_Delay_ms）
- 任何需要高精度计时的模块

## API 使用

### 延时函数

```c
DWT_Delay_ms(1.0f);   // 延时 1 毫秒
DWT_Delay_us(100.0f);  // 延时 100 微秒
```

### 时间戳获取

```c
// 获取当前时间（不同精度）
float time_s = DWT_GetTimeline_s();    // 秒
float time_ms = DWT_GetTimeline_ms();  // 毫秒
uint64_t time_us = DWT_GetTimeline_us();  // 微秒
```

### 时间差测量

```c
uint32_t last_cnt = 0;

// 循环中
float dt = DWT_GetDeltaT(&last_cnt);  // 距离上次调用的时间差（秒）
```

### 代码执行时间测量

```c
float elapsed;
TIME_ELAPSE(elapsed, {
    // 要测量的代码
    for (int i = 0; i < 1000; i++) {
        // ...
    }
});
printf("执行时间: %.6f 秒\n", elapsed);
```

## 依赖关系

```
DWT (bsp_dwt)
  ↑
  ├── AHRS (姿态解算)
  ├── 任何需要高精度计时的模块
  └── ...
```

**关键规则**：DWT 是基础模块，必须在所有依赖它的模块之前初始化。

## 注意事项

1. **初始化顺序**：`DWT_Init()` 必须在 `Robot_Init()` 中最先调用
2. **中断影响**：`DWT_Delay` 是忙等待，延时期间中断被延迟
3. **线程安全**：`DWT_CNT_Update()` 使用位锁保护，可在中断中调用
4. **精度限制**：受限于 CPU 时钟频率，最高精度约 6ns（@168MHz）
