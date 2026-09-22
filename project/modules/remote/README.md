# remote 模块

## 概述

遥控接收门面：编译期路由到 **VT13**（图传）、**DT7**（独立接收器）或 **HT-10A**（SBUS）。应用层通过 `Remote` / `Remote_Init` 与 `REMOTE_*` 宏读数。

## 机型切换

编辑 [`remote_config.h`](remote_config.h)，**只保留一个**机型宏：

```c
// #define REMOTE_DEVICE_VT13
#define REMOTE_DEVICE_DT7
// #define REMOTE_DEVICE_HT10A
```

换机型后必须同步改 CubeMX 里该 UART 的波特率 / 字长 / 校验。`Remote::init` 会核对 HAL 句柄参数，不匹配则打日志且不启动接收。

| 机型 | Baud | WordLength | Parity | 典型接线 |
|------|------|------------|--------|----------|
| VT13 | 921600 | 8B | None | 图传串口（本工程常用 USART3） |
| DT7  | 100000 | 9B | Even | DT7 接收器 DBUS |
| HT-10A | 100000 | 9B | Even | SBUS 接收端（本工程常用 USART3） |

当前默认 **DT7**。切到 HT-10A 后 `cmd` 还没有第三套控制路径，会编不过，需另改应用层。

## 目录

```
remote/
  remote_config.h     机型宏 + 期望串口参数
  remote.h / .cpp     门面 class Remote + 访问宏
  vt13/               VT13 协议
  dt7/                DT7 协议（无键鼠）
  ht10a/              HT-10A SBUS 协议
```

## 依赖

```
remote (facade)
  ├─ vt13 / dt7 / ht10a（编译期三选一）
  │    ├─ serial
  │    ├─ daemon
  │    └─ crc（仅 VT13）
  └─ bsp_usart / bsp_tim
```

## API

```cpp
Remote::Config cfg = {
    .device = Remote::Device::Vt13,  // 须与 remote_config.h 一致
    .usart_handle = &huart3,
};
Remote::instance().init(cfg);

// 兼容旧调用（内部填 Config）
Remote_Init(&huart3);
uint8_t on = Remote_Online();
```

## VT13 数据访问

- 帧长 21，帧头 `0xA9 0x53`，CRC16
- `REMOTE_RC_RH/RV/LV/LH/WHEEL`、`REMOTE_RC_SWITCH`、FN/Trigger、键鼠宏（与迁移前一致）

## DT7 数据访问

- 帧长 18，无 CRC；仅摇杆 ×2、左右开关、拨轮
- `REMOTE_RC_RH/RV/LV/LH/WHEEL`
- `REMOTE_RC_SW_LEFT()` / `REMOTE_RC_SW_RIGHT()`（值：UP=1, DOWN=2, MID=3）
- **无键鼠宏**

## HT-10A 数据访问

- 帧长 25，帧头 `0x0F`、帧尾 `0x00`
- `REMOTE_RC_CH(n)`：16 路 SBUS 11bit 原值（`n` 为 0..15）
- `REMOTE_RC_RH/RV/LV/LH()`：CH1–CH4 减 992
- `REMOTE_RC_SW_RIGHT()` / `REMOTE_RC_SW_LEFT()`：CH5 / CH8 减 192（实测 UP=0, MID=32, DOWN=64）
- `REMOTE_RC_SW_CENTER_LEFT()` / `REMOTE_RC_SW_CENTER_RIGHT()`：CH6 / CH7 原值
- 正中间旋钮这轮不起名，从 `REMOTE_RC_CH(8)` / `REMOTE_RC_CH(9)` 看
- **无键鼠宏**；切机型后需另改 `cmd`

## 相关

- C 板 DBUS 线序（接收器）：DBUS(PC8) 5V(PB8) GND(PA8)
