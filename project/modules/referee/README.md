# referee 模块（裁判系统通信）

## 概述

RoboMaster 裁判系统通信模块，负责机器人主控与裁判系统之间的数据交互：

- **接收**：裁判系统周期性下发的比赛数据（血量、热量、功率、位置、增益等），解析成 C 结构体供上层直接读取。
- **发送**：向裁判系统发送自定义 UI 图形、机器人间通信（交互数据），显示到选手端界面。

一句话理解：裁判系统是「官方裁判」，每 10Hz / 3Hz / 1Hz 往你机器人的串口里塞一堆状态字节；本模块负责把这串字节解析成你能直接用的结构体，并帮你把要画的东西（UI）发回选手端。

## 硬件与初始化

- 串口：裁判系统官方设计的接口是 **UART6**（丝印标注「UART1」）。infantry_cpp 原车因 UART6 被小电脑占用，把裁判接在 **USART1**；
  **本车(Sentry) 按原厂接法把裁判接在 USART6（`huart6`），小电脑/视觉接 USART1（`huart1`）**——与原车正好相反。
- 初始化入口：`Referee_Init(&huart6)`，在 `cmd.cpp` 的 `Cmd_Init()` 中调用（换串口时改这里的句柄即可；串口分配说明也写在该处注释里）。
- 无需在主循环里手动轮询；数据到达会通过回调自动解析。

## 依赖

```
referee
  ↑
  ├── serial   (串口收发：不定长接收 + 空闲中断回调)
  ├── daemon   (在线检测 / 看门狗)
  ├── crc      (CRC-8/MAXIM + CRC-16 校验)
  └── bsp_dwt  (DWT，见 include)
```

- **serial**：负责「收一包字节 + 触发回调」。referee 不直接碰 UART/DMA，只实现一个 `rx_callback`。
- **daemon**：100ms 内收到有效帧即喂狗，超时判离线。
- **crc**：帧头 CRC8 + 整帧 CRC16 双重校验。

## 帧格式

裁判系统的数据是一帧一帧的，每帧结构（字节序均为**小端**）：

```
+------+-------------+-----+------+--------+-------------+--------+
| SOF  | data_length | seq | CRC8 | cmd_id |    data     | CRC16  |
| 1B   | 2B          | 1B  | 1B   | 2B     | nB          | 2B     |
+------+-------------+-----+------+--------+-------------+--------+
  帧头(5B)                        cmd_id(2B)   数据段(nB)    帧尾(2B)
```

| 字段 | 长度 | 说明 |
|---|---|---|
| SOF | 1 | 帧头字节，固定 `0xA5` |
| data_length | 2 | 数据段长度 n（**不含**头尾） |
| seq | 1 | 包序号 |
| CRC8 | 1 | 对「SOF + data_length + seq」共 4 字节的校验 |
| cmd_id | 2 | 命令码，决定 data 里装的是什么 |
| data | n | 数据段 |
| CRC16 | 2 | 对「帧头 + cmd_id + data」的校验（不含帧尾） |

完整帧长 = **9 + n** 字节（5 帧头 + 2 cmd_id + n 数据 + 2 帧尾）。

## 接收流程（重点：粘包）

裁判系统会**连续**下发多帧（比如 10Hz 的 `robot_status` + 10Hz 的 `power_heat` 紧挨着发），串口空闲中断经常把好几帧当成「一次突发」一起上报。这叫**粘包**。

因此解析**不能一次只处理一帧**，必须**循环**：从缓冲区头开始，一帧一帧地剥，剥完一帧往前跳一帧，直到剩下字节不够一帧为止。

`Referee_ParseFrame` 核心逻辑：

```
offset = 0
while 剩余字节 >= 9:
    1. SOF == 0xA5 ?          否 → offset++      （逐字节找下一帧头）
    2. CRC8 校验通过 ?         否 → offset++      （可能是假帧头）
    3. 读 data_length，算整帧长度
    4. 剩余不够这一帧 ?         → break           （等下一包补齐）
    5. CRC16 校验通过 ?        否 → offset += 帧长  （跳过坏帧）
    6. 按 cmd_id 分发，memcpy 到 referee_info
    7. 喂狗 Daemon_Reset
    8. offset += 帧长
```

每一步校验失败都有对应的「逃生」策略（重找帧头 / 等下一包 / 跳过坏帧），保证一帧损坏不会让整个缓冲区卡死。

## 数据结构：referee_info

所有解析结果都存进**同一个静态结构体** `referee_info`（类型 `referee_info_t`，定义在 `referee.h`）。各 `cmd_id` 与结构体成员的对应关系：

| cmd_id | 结构体成员 | 内容 | 频率 |
|---|---|---|---|
| 0x0001 | `game_state` | 比赛状态 | 1Hz |
| 0x0002 | `game_result` | 比赛结果 | — |
| 0x0003 | `game_robot_HP` | 全队血量 | 3Hz |
| 0x0101 | `event_data` | 场地事件 | 1Hz |
| 0x0104 | `referee_warning` | 裁判警告 | — |
| 0x0105 | `dart_info` | 飞镖数据 | 1Hz |
| 0x0201 | `robot_status` | 本机器人状态（血量/热量/功率上限） | 10Hz |
| 0x0202 | `power_heat` | 功率热量（缓冲能量/枪口热量） | 10Hz |
| 0x0203 | `robot_pos` | 位置 + 朝向 | 1Hz |
| 0x0204 | `buff_musk` | 增益 | 3Hz |
| 0x0206 | `robot_hurt` | 伤害状态 | 即时 |
| 0x0207 | `shoot_data` | 射击数据 | — |
| 0x0208 | `projectile_allowance` | 允许发弹量 | 10Hz |

此外 `referee_info.id` 是在解析 `robot_status` 时顺带算出的机器人 ID / 颜色 / 客户端 ID。

**获取数据**：任何地方调用 `Referee_GetData()` 拿到指针，直接读成员：

```c
referee_info_t *info = Referee_GetData();
uint16_t hp   = info->robot_status.current_HP;          // 当前血量
uint16_t heat = info->power_heat.shooter_17mm_barrel_heat; // 17mm 当前热量
uint16_t power_limit = info->robot_status.chassis_power_limit; // 底盘功率上限(W)
```

每个结构体的字段定义在 `referee_protocol.h`，字段含义与单位参考官方协议 `references/referee.md`。

## API

### 生命周期

| 函数 | 说明 |
|---|---|
| `referee_info_t *Referee_Init(UART_HandleTypeDef *huart)` | 初始化（注册 serial + daemon），返回数据指针 |
| `referee_info_t *Referee_GetData()` | 获取数据指针 |
| `uint8_t Referee_Online()` | 是否在线（1/0） |

### 发送

| 函数 | 说明 |
|---|---|
| `Referee_SendFrame(cmd_id, data, len)` | 发送一帧原始数据 |
| `Referee_SendInteractive(sub_cmd_id, receiver_id, data, len)` | 发送交互数据（0x0301） |

### UI 绘图（画到选手端界面）

| 函数 | 说明 |
|---|---|
| `Referee_UIDelete(operate, layer)` | 删除图层 |
| `Referee_UIDraw(graph_name, operate, type, layer, color, ...)` | 画图形（线/矩形/圆/浮点数/整数） |
| `Referee_UIRefresh(cnt, ...)` | 批量刷新多个图形 |
| `Referee_UIChar(graph_name, operate, layer, color, size, width, x, y, fmt, ...)` | 画字符（类似 printf） |
| `Referee_UICharRefresh(data)` | 刷新字符 |

## 使用示例

### 读数据 + 判在线

```c
if (!Referee_Online()) return;   // 离线不做处理

referee_info_t *info = Referee_GetData();
uint16_t hp  = info->robot_status.current_HP;
uint16_t max = info->robot_status.maximum_HP;
```

### 画一个浮点数到选手端

```c
// 在 (500, 300) 处显示 yaw 角度，图层 0，白色，字号 12，线宽 2
Referee_UIDraw("yaw", UI_Graph_Add, UI_Graph_Float, 0, UI_Color_White,
               /*font_size*/ 12, /*width*/ 2, /*x*/ 500, /*y*/ 300,
               /*value*1000*/ (uint32_t)(yaw_deg * 1000));
```

### 画一行字符

```c
Referee_UIChar("dbg", UI_Graph_Add, 0, UI_Color_White, 12, 2, 500, 400,
               "HP=%d heat=%d", hp, heat);
```

## 注意事项（容易踩的坑）

1. **字节对齐必须 pack**：数据段是紧密排列的，所有结构体必须 `#pragma pack(1)`，否则 uint16/float 前会被编译器填 padding，导致 `sizeof` 大于实际数据长度、`memcpy` 越界读。`referee_protocol.h` 里的 13 个数据结构已统一 pack，新增字段时别忘了。

2. **位域无法在 Ozone 里直接 watch**：`game_type`、`power_management_*_output`、`armor_id` 等是位域，没有独立地址，Ozone 会显示 `<outofscope>`。想看就 watch 所在整字节再手工掩码；调试时 `cmd.c` 里有个 `referee_dbg` 结构体专门把这些位域拆成了字节字段。

3. **`referee_info.online` 字段无人维护**：真正的在线状态由 daemon 管理，请用 `Referee_Online()`（读 `referee_daemon->online`），别读 `referee_info.online`。

4. **0x0202 前 8 字节是保留位**：2026 协议不再下发底盘当前功率，`power_heat.reserved1/2/3` 恒为 0。要拿底盘当前功率得自己从电机电流估算（P = Σ Kt·I·ω，机械功率，负功率/倒灌算 0）。

5. **缓冲能量上限是 60J**：`power_heat.buffer_energy` 是当前值（0~60），不是几百焦耳。

6. **射击热量到顶会锁枪**：热量超上限会锁枪且必须冷却到 0 才解锁，超「上限+100」永久锁死。功率管理时别碰这条红线，详见 `references/功率管理规则速查.md`。

7. **帧长与结构体大小必须一致**：`memcpy(&referee_info.xxx, data, sizeof(xxx))` 要求 `sizeof(结构体) == data_length`。改结构体字段后记得对照 `DataLen_e` 枚举核对。
