# CRC 模块

## 概述

RoboMaster 裁判系统专用 CRC 校验模块，仅用于实现 RM 场景下的协议校验。不具备泛用性和可移植性。

## CRC 类型

本模块实现了裁判系统协议用到的两种 CRC，均为**反射（LSB-first）实现**：

| 类型 | 多项式 | 反射多项式 | 初值 | 用途 |
|------|--------|-----------|------|------|
| CRC-8/MAXIM | 0x31 | 0x8C | 0xFF | 裁判系统帧头校验 |
| CRC-16（反射 CCITT） | 0x1021 | 0x8408 | 0xFFFF | 裁判系统 + 图传整包校验 |

### CRC-8：CRC-8/MAXIM

- 多项式：`x^8 + x^5 + x^4 + 1 = 0x31`，反射多项式 `0x8C`
- 初始值：`0xFF`
- 输入/输出均反射（LSB-first）
- 表驱动：`crc = crc8_tab[crc ^ data]`
- 校验向量：`CRC-8/MAXIM("123456789", init=0x00) = 0xA1`

> 与标准 CRC-8/MAXIM 的唯一区别是**初值**：标准 MAXIM 用 `0x00`，裁判系统用 `0xFF`（多项式与反射方式完全相同）。

### CRC-16：裁判系统官方实现（反射 CCITT）

- 多项式：`x^16 + x^12 + x^5 + 1 = 0x1021`，反射多项式 `0x8408`
- 初始值：`0xFFFF`
- 输入/输出均反射（LSB-first）
- 表驱动：`crc = (crc >> 8) ^ crc16_tab[(crc ^ data) & 0xFF]`
- 校验向量：`("123456789") = 0x6F91`

> **注意**：该实现常被误称为「CRC-16/CCITT-FALSE」。真正的 CRC-16/CCITT-FALSE 是**非反射（MSB-first）**实现，校验向量为 `0x29B1`。裁判系统官方代码用的是反射实现（`0x6F91`），本模块与其保持一致。

## API

```c
// CRC-8
uint8_t CRC8_Calculate(const uint8_t *data, uint16_t len);
uint8_t CRC8_Verify(const uint8_t *data, uint16_t len);

// CRC-16
uint16_t CRC16_Calculate(const uint8_t *data, uint16_t len);
uint8_t  CRC16_Verify(const uint8_t *data, uint16_t len);
```

## 使用方式

### CRC-8（裁判系统帧头校验）

```c
// 计算前 4 字节的 CRC8（SOF + data_length + seq）
uint8_t crc = CRC8_Calculate(frame, 4);

// 验证帧（data 包含 CRC，len=5）
uint8_t ok = CRC8_Verify(frame, 5);  // 1=正确, 0=错误
```

### CRC-16（裁判系统 + 图传整包校验）

```c
// 计算前 N 字节的 CRC16（不含末尾 2 字节 CRC）
uint16_t crc = CRC16_Calculate(frame, N);

// 验证帧（data 包含 CRC，len=N+2）
uint8_t ok = CRC16_Verify(frame, N + 2);  // 1=正确, 0=错误
```

## Verify 函数说明

`CRC*_Verify(data, len)` 假设 CRC 在数据末尾：
- CRC-8：`data[len-1]` 是 CRC
- CRC-16：`data[len-2]` 和 `data[len-1]` 是 CRC（小端序）

计算前 `len-1` 或 `len-2` 字节的 CRC，与末尾比较。

## 实现方式

通过 `CRC_USE_TABLE` 宏控制：
- `CRC_USE_TABLE = 1`：查表法（快，占 ROM）——**默认，且已对齐裁判系统官方附录**
- `CRC_USE_TABLE = 0`：位运算法（慢，不占 ROM）

两种方式结果一致（反射实现）。

## 验证

CRC8 / CRC16 查找表可通过 Python 脚本验证：

```bash
python tools/crc8_gen.py
```

脚本会：
1. 生成 CRC-8/MAXIM 反射表，并用标准 check 值 `0xA1` 交叉验证；
2. 生成裁判系统 CRC16 反射表，并用 check 值 `0x6F91` 交叉验证；
3. 打印可直接粘贴到 `crc.c` 的 C 数组。

## 注意事项

- **仅用于 RoboMaster 项目**，不适用于其他场景。
- 两种 CRC 均为**反射（LSB-first）**实现，勿与教科书上的非反射实现混淆。
