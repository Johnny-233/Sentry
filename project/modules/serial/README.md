# serial 模块

## 概述

串口通信管理模块，作为 bsp_usart 的上层封装，提供不定长数据接收和缓冲区管理。

## 依赖

- 请将串口配置为 DMA + Circular Buffer 模式，本模块依赖空闲中断和 DMA Circular Buffer

```
serial
  ↑
bsp_usart (BSP 层)
daemon    (模块层，缓冲区满超时检测)
```

## Circular 与重启接收

选用 **DMA Circular** 后，写指针在环缓冲内连续前进，写满绕回头部继续写，**无需每次把指针重置到缓冲区头**，少一次拆/建 DMA，效率更高；本模块把 Idle 回调里的 `Size` 当作环内写位置（`recv_pos`），用 `last_pos` 做差或跨尾拼接。

**为何还要重启接收（`USART::serviceInit` / `ReceiveToIdle`）：**

- Idle 事件表示「这一小段收完了」；要继续用 ToIdle 方式盯下一帧，BSP 侧会再调一次接收启动。
- **Normal 模式**：Idle 后 HAL 会 Abort DMA，必须重启，否则后面字节收不到；重启后从缓冲起点重新开收。
- **Circular 模式（本模块要求）**：Idle 时 HAL **不** Abort DMA，环上写指针继续往后（绕回头部也是接着写，不是业务上的「复位到头重来」）。此时再调 `ReceiveToIdle` 往往因 `RxState` 仍为 Busy 而返回 `HAL_BUSY`（空操作），真正继续收靠的是还在转的 Circ DMA；调用重启是与 Normal 路径对齐、并确保 ToIdle/Idle 相关状态在可重启时被重新武装，**不要理解成「每次把写指针打回 buffer[0]」**。

## API

```c
Serial_Instance *Serial_Register(Serial_Init_Config_s *init_config);
void Serial_Send(Serial_Instance *instance, uint8_t *send_buf, uint16_t send_size);
```

## recv_size 配置

| recv_size | 模式 | 行为 |
|-----------|------|------|
| 0 或未设置 | 单块 | 默认为 1 |
| = 1 | 单块 | `parse_buf` 直接指向 BSP `recv_buff`，零拷贝 |
| > 1 | 多块 | 分配 `recv_size × 256` 字节缓冲区 |

## 数据流

```
UART 硬件 → DMA 接收 → BSP 回调 → Serial_USART_RX_Callback
                                         ↓
                              ┌───────────┴───────────┐
                              │                       │
                         空闲中断                  全满中断
                              ↓                       ↓
                    搬运到 recv_buf            切换块，喂 daemon
                    触发 rx_callback
```

## full_daemon 机制

`full_daemon` 不是"串口离线检测"，而是**不完整数据帧处理机制**：

1. 全满中断触发 → 喂狗，开始 2ms 倒计时
2. 如果 2ms 内收到空闲中断 → 正常搬运数据
3. 如果 2ms 内没收到空闲中断 → `Serial_FullTimeout()` 强制搬运已收到的数据

用途：处理发送端在数据帧中间停止发送的情况。

## 使用示例

```c
Serial_Init_Config_s config = {
    .usart_handle = &huart6,
    .recv_size = 1,             // 单块模式（零拷贝）
    .rx_callback = my_callback,
};
Serial_Instance *serial = Serial_Register(&config);

// 发送
Serial_Send(serial, data, sizeof(data));

// 接收（在回调中）
void my_callback(uint16_t len) {
    // serial->recv_buf 中有 len 字节数据
}
```
