# daemon 模块

## 概述

看门狗/守护进程模块，用于检测设备是否在线。当设备在指定周期内没有"喂狗"，则触发离线回调。

## 依赖

```
daemon
  ↑
bsp_tim (BSP 层，定时器)
```

## 工作模式

通过 `TIM_DAEMON_SUPPORT` 宏选择模式：

| 模式 | 说明 | 推荐 |
|------|------|------|
| TIM 模式 | 使用 BSP_TIM 定时器，纯 ISR 计数 | ✓ 推荐 |
| RTOS 模式 | 使用 CMSIS 软件定时器 | 复杂，不推荐 |

### TIM 模式（推荐）

使用 1ms 定时器中断，每个 daemon 实例维护一个计数器。

```
定时器中断 (1ms)
    ↓
Timer_Callback()
    ↓
遍历所有 daemon 实例
    ↓
count > 0 → count--
count == 0 → 触发 daemon_callback，标记 offline
```

### RTOS 模式局限

使用 CMSIS 软件定时器。局限：
- 必须绕过 CMSIS 调用 `xTimerReset`
- ISR 中必须调用 ISR 版本的 `xTimerResetFromISR`
- 配置复杂，可移植性差

## API

```c
Daemon_Instance *Daemon_Register(Daemon_Init_Config_s *config);
void Daemon_Reset(Daemon_Instance *daemon);
// 检查在线状态：daemon->online == 1 在线，0 离线
```

## 初始化配置

```c
Daemon_Init_Config_s config = {
    .tim_config = { .htim = &htim5 },  // TIM 模式需要指定定时器
    .cycle = 100,                       // 超时周期（ms）
    .daemon_callback = my_offline_handler,
    .device = my_instance,
};
Daemon_Instance *daemon = Daemon_Register(&config);
```

## 工作流程

```
注册 daemon
    ↓
Daemon_Reset() → online=1, count=cycle
    ↓
定时器每 1ms 中断
    ↓
count-- → count==0 时触发回调
    ↓
daemon_callback(device) → 设备离线处理
    ↓
daemon->online = 0
```

## 定时器配置建议

- **周期**：1ms（所有 daemon 共享同一个定时器）
- **定时器**：TIM5（已配置）
- **精度**：1ms 级别

## 注意事项

1. 所有 daemon 实例共享同一个定时器中断
2. 定时器周期决定 daemon 的最小检测精度
3. `cycle` 参数是超时倍数，实际超时 = cycle × 定时器周期
4. TIM 模式下，ISR 中喂狗只需调用 `Daemon_Reset()`
