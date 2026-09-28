# 哨兵 ⇄ 小电脑（视觉）通信协议

> 来源：本仓 `project/modules/master_machine/`（`seasky_protocol.cpp` / `master_process.cpp`），
> 由原 `Modules/master_machine/` 逐字节移植，**协议未改动**。
> 全部字段 **小端（little-endian）**，`float` 为 IEEE-754 单精度。

## 0. 物理层

| 项 | 值 |
| --- | --- |
| 串口 | `USART1`（车上 4pin 口，和小电脑相连） |
| 参数 | **115200, 8 数据位, 1 停止位, 无校验, 无流控** |
| 车 → 小电脑 | 车主动发（TX） |
| 小电脑 → 车 | 车接收（RX），**已开启严格校验**（长度 + CRC16，见 §3） |

⚠️ 帧本身按 **1 kHz** 触发发送（INS 任务每 1ms 一帧，另有 200Hz 的 cmd 任务共用同一缓冲再发一次），
但 41 字节/帧 × 1 kHz = 41 kB/s 已超过 115200 的 11.5 kB/s，串口 DMA 忙时**直接丢帧**（不阻塞）。
**实际到达小电脑的帧率上限约 280 帧/s**（115200 ÷ 10 bit/字节 ÷ 41 字节）。
小电脑端请按"随时可能丢帧、不要依赖固定帧率"来设计（用最新一帧即可）。
若需要更高帧率，把 `Src/usart.c` 里 `huart1.Init.BaudRate` 调高（双方同步改）。

---

## 1. 车 → 小电脑（上行，**41 字节固定长**）

```
偏移  类型      字段                说明 / 单位
----  --------  ------------------  --------------------------------------------
 0    uint8     sof                 固定 0xA5
 1    uint8     detect_color        敌方颜色：0 = 红, 1 = 蓝
 2    uint8     reserved            保留，置 0
 3    float     roll                横滚角 [度]
 7    float     pitch               俯仰角 [度]
11    float     yaw                 偏航角 [度]
15    float     vx                  x 轴线速度（与小电脑下发的 linear_velocity_x 同量纲/同坐标系）
19    float     vy                  y 轴线速度（同上）
23    uint16    self_sentry_hp      己方哨兵血量
25    uint16    self_hero_hp        己方英雄血量
27    uint16    self_infantry_hp    己方步兵血量
29    uint16    remain_time         剩余时间 [s]
31    uint16    remain_bullet       剩余允许发弹量
33    uint8     match_progress      比赛进度
34    uint8     occupation          占领状态（**当前恒为 0**：裁判协议无对应字段）
35    float     bullet_speed        弹速 [m/s]
39    uint8     armor_id            最近一次受击的装甲板 ID（0 = 开机以来无受击）
40    uint8     armor_reason        扣血原因（0=弹丸命中, 1=撞击, 4=超射速…）
41    uint8     armor_hit_cnt       累计受击次数（8 位回绕）
42    uint8     armor_hit_seq       事件序号：**每检测到一次新受击 +1**（对方据此判断"新的一次"）
43    float     armor_angle_chassis 受击方向, 相对底盘正前方 [度], 逆时针为正
47    float     armor_angle_world   叠加当前 yaw 后的场地方位 [度], 0~360
51    uint16    crc16               帧校验，放在最后 2 字节，**低字节在前**
----  --------
总长 53 字节（原 41 字节的字段与偏移**完全不变**，受击字段追加在末尾）
```

**CRC16 覆盖范围**：`字节[0..50]`（即除最后 2 字节 CRC 外的全帧，共 51 字节）。

Python struct 格式串（含帧头与 CRC）：

```python
import struct

def crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS：多项式 0xA001(反射)、初值 0xFFFF、无结果异或"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def parse_uplink(frame: bytes) -> dict:
    assert len(frame) == 53, f"长度应为 53，实际 {len(frame)}"
    assert frame[0] == 0xA5, "帧头应为 0xA5"
    assert crc16_modbus(frame[:51]) == (frame[51] | (frame[52] << 8)), "CRC16 校验失败"
    (detect_color, reserved, roll, pitch, yaw, vx, vy,
     hp_sentry, hp_hero, hp_infantry, remain_time, remain_bullet,
     match_progress, occupation, bullet_speed,
     armor_id, armor_reason, armor_hit_cnt, armor_hit_seq,
     armor_angle_chassis, armor_angle_world) = struct.unpack_from('<BBfffffHHHHHBBfBBBBff', frame, 1)
    return dict(detect_color=detect_color, roll=roll, pitch=pitch, yaw=yaw,
                vx=vx, vy=vy, hp_sentry=hp_sentry, hp_hero=hp_hero,
                hp_infantry=hp_infantry, remain_time=remain_time,
                remain_bullet=remain_bullet, match_progress=match_progress,
                occupation=occupation, bullet_speed=bullet_speed)
```

字段来源（车里怎么填的，便于对方理解语义）：

| 字段 | 来源 |
| --- | --- |
| roll / pitch / yaw | 车载 IMU（AHRS）欧拉角，单位**度**，每帧刷新 |
| vx / vy | 底盘**指令**速度（云台系）反算到视觉域：`vx = -chassis_cmd_recv.vx / (4*REDUCTION_RATIO_WHEEL*360/PERIMETER_WHEEL*1000)`；即"把小电脑下发的 velocity 换算进底盘指令域"那条公式的逆运算 → 与对方下发的 `linear_velocity_x/y` **同量纲、可直接比较**。注意是**指令值**不是轮速实测值 |
| self_*_hp / remain_time / remain_bullet / match_progress | 裁判系统（`referee_data`），无数据时为 0 |
| bullet_speed | 裁判系统弹速 |
| detect_color | 由遥控器/UI 设定的敌方颜色 |
| occupation | 无来源，恒 0 |

---

## 2. 小电脑 → 车（下行，**27 字节**，本车按此解析）

```
偏移  类型      字段                  说明 / 单位
----  --------  --------------------  ------------------------------------------
 0    uint8     header                固定 0x5A
 1    float     linear_velocity_x     底盘 x 轴线速度指令（云台系）
 5    float     linear_velocity_y     底盘 y 轴线速度指令（云台系）
 9    int32     gimbal_mode           小陀螺模式：0 = 关，非 0 = 开
13    float     yaw                   需要云台转动的 **相对** yaw 角 [度]
17    float     pitch                 需要云台转动的 **相对** pitch 角 [度]
21    int32     can_fire              开火许可：0 = 无目标，非 0 = 可开火
25    uint16    checksum              CRC16，最后 2 字节，**低字节在前**
----  --------
总长 27 字节
```

**CRC16 覆盖范围**：`字节[0..24]`（除最后 2 字节外的全帧），算法与上行**完全相同**（CRC-16/MODBUS，
初值 0xFFFF、多项式 0xA001、低字节在前）。

```python
def build_downlink(linear_velocity_x=0.0, linear_velocity_y=0.0,
                   gimbal_mode=0, yaw=0.0, pitch=0.0, can_fire=0) -> bytes:
    body = struct.pack('<Bffiffi', 0x5A, linear_velocity_x, linear_velocity_y,
                       int(gimbal_mode), yaw, pitch, int(can_fire))   # 25 字节
    return body + struct.pack('<H', crc16_modbus(body))                # 27 字节
```

车的处理：

| 字段 | 车端行为 |
| --- | --- |
| `yaw` / `pitch` | 哨兵模式下按相对角驱动云台（`FoundEnermy()`）；`pitch` 会再被软限位 `[PITCH_MIN_ANGLE, PITCH_MAX_ANGLE]` 夹一次 |
| `gimbal_mode` | 非 0 切小陀螺（底盘旋转） |
| `can_fire` | 非 0 且满足发射条件时允许开火 |
| `linear_velocity_x/y` | 哨兵模式下作为底盘速度指令 |

**校验规则（重要，已开启严格模式）**：

1. 收到的帧长必须 **≥ 25 字节**，否则丢帧；
2. 取最后 2 字节作为 CRC16（**低字节在前**），对 `整帧除最后 2 字节` 计算 CRC16，两者不等则**整帧丢弃**；
3. 因此：**不能只发 0x5A 开头的任意数据**，必须带正确的整帧 CRC16；
4. 帧长按实际收到的长度参与校验（不写死 27），但**推荐固定 27 字节**；
5. 结构体里 `checksum` 字段在车端是按大端读入的，仅作记录、不参与判断——校验一律以最后 2 字节的小端为准。

> 历史坑：原代码注释记着"开启校验后现场收不到小电脑数据"。若联调收不到，先确认对方的
> CRC 是 **初值 0xFFFF / 多项式 0xA001 / 反射 / 低字节在前 / 覆盖除末 2 字节外的整帧**。
> 特别注意：**不要**用 RoboMaster 官方裁判系统那套 CRC（CRC8 初值 0xFF、CRC16 多项式 0x8408/CCITT），
> 两者不通用。

---

## 3. 一页速查

| | 上行（车 → PC） | 下行（PC → 车） |
| --- | --- | --- |
| 帧长 | 53 字节（固定） | 27 字节（推荐固定） |
| 帧头 | `0xA5` | `0x5A` |
| CRC | CRC-16/MODBUS，低字节在前，末 2 字节 | 同左 |
| CRC 覆盖 | `frame[0:51]` | `frame[0:25]` |
| 频率 | 触发 1 kHz，实际 ≤ ~217 帧/s（115200） | 建议 ≤ 100 Hz |
| 端序 | 小端 | 小端 |
