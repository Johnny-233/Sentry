# PID 模块

## 概述

通用 PID 控制器模块，位于 `alg/` 纯算法层，**无外设依赖**。支持位置式 / 增量式两种模式，以及 8 个可选功能特性（滤波、限幅、死区、变增益、微分先行、前馈等）。

## 依赖

无外部依赖（仅标准库 `malloc` / `string`）。

## 数据结构

### 初始化配置 `PID_Init_Config_s`

| 字段 | 说明 |
|------|------|
| kp / ki / kd | 比例 / 积分 / 微分系数 |
| mode | `PID_MODE_POSITION` 或 `PID_MODE_DELTA` |
| features | 功能特性位掩码（见下表） |
| period | 采样周期（ms），**必须与实际调用周期一致** |
| integral_limit | 积分限幅（仅位置式） |
| derivative_limit | 微分限幅 |
| output_min / output_max | 输出限幅上下界 |
| dead_zone | 死区范围 |
| feedforward_gain | 前馈增益（0 时默认 1.0） |
| filter_alpha | 滤波系数 (0~1)，越小滤波越强（≤0 时默认 0.3） |
| kp_extra / kd_extra | 变增益斜率（仅 kp/kd，ki 不支持变增益） |

### 运行时实例 `PID_Instance`

由 `PID_Init()` 返回，后续所有操作都通过该指针。内部字段（integral、last_error、last_feedback、output 等）不应直接读写，统一走 API。

## 两种工作模式

### 位置式 `PID_MODE_POSITION`

```
integral += error · period            （累加器）
output    = kp·error + ki·integral + kd·derivative
```

- `integral` 是**累加器**，跨周期累积。
- 输出是**绝对量**。
- 抗积分饱和：条件积分法（见下文「关键实现细节」）。

### 增量式 `PID_MODE_DELTA`

```
Δoutput = kp·(error - last_error) + ki·(error·period) + kd·二阶差分
output += Δoutput
```

- `integral` 字段**不是累加器**，每个周期被覆盖为 `error·period`（`PID_Reset_Integral` 对它无意义）。
- 输出是**增量累积**，且每周期被输出限幅钳住，天然不会像位置式那样积分饱和失控。
- 适合执行器带积分特性（如速度环驱动电流环）或需要无扰切换的场景。

> ⚠️ 两种模式下 `integral` 字段语义不同：位置式 = 累加器，增量式 = 当周期项。跨模式理解会出错。

## 功能特性

`features` 是位掩码，通过 `PID_Set_Features` / `PID_Clear_Features` 开关。

| 特性 | 位 | 说明 | 适用 |
|------|----|------|------|
| `PID_FEATURE_INTEGRAL_LIMIT` | 0x01 | 积分限幅 | 仅位置式 |
| `PID_FEATURE_DERIVATIVE_LIMIT` | 0x02 | 微分限幅 | 两者 |
| `PID_FEATURE_OUTPUT_LIMIT` | 0x04 | 输出限幅（位置式同时触发抗积分饱和） | 两者 |
| `PID_FEATURE_DEAD_ZONE` | 0x08 | 死区：`\|error\| < dead_zone` 时 error 归零 | 两者 |
| `PID_FEATURE_FILTER` | 0x10 | 反馈一阶低通滤波 | 两者 |
| `PID_FEATURE_VARIABLE_GAIN` | 0x20 | 变增益：kp/kd 随 `\|error\|` 线性增大 | 两者 |
| `PID_FEATURE_DERIVATIVE_ON_MEASUREMENT` | 0x40 | 微分先行（对测量值微分，抑制设定值阶跃的微分冲击） | 两者 |
| `PID_FEATURE_FEEDFORWARD` | 0x80 | 前馈 | 两者 |

## API

```c
PID_Instance * PID_Init(PID_Init_Config_s *config);              // 初始化，返回实例
void PID_Update(PID_Instance *instance, float feedback);         // 周期更新，结果写入 output
void PID_Set_Parameters(PID_Instance *instance, float kp, float ki, float kd);
void PID_Set_Setpoint(PID_Instance *instance, float setpoint);
void PID_Set_Feedforward(PID_Instance *instance, float feedforward);
void PID_Reset(PID_Instance *instance);                          // 复位全部状态
void PID_Reset_Integral(PID_Instance *instance);                 // 仅清积分（=置 0）
PID_FEATURE PID_Get_Features(PID_Instance *instance, PID_FEATURE f);
void PID_Set_Features(PID_Instance *instance, PID_FEATURE f);
void PID_Clear_Features(PID_Instance *instance, PID_FEATURE f);
```

## 使用方式

```c
PID_Init_Config_s cfg = {
    .kp = 4.0f, .ki = 0.0f, .kd = 0.01f,
    .mode = PID_MODE_POSITION,
    .features = PID_FEATURE_OUTPUT_LIMIT | PID_FEATURE_DERIVATIVE_ON_MEASUREMENT,
    .output_min = -360.0f, .output_max = 360.0f,
    .period = 10,                 // 每 10ms 调用一次 PID_Update
};
PID_Instance *pid = PID_Init(&cfg);

// 周期任务（调用周期必须 = period ms）
PID_Set_Setpoint(pid, target);
PID_Update(pid, feedback);
current_cmd = pid->output;
```

## 关键实现细节

### 微分项

- 普通：`derivative = (error - last_error) / period`（对误差微分）
- 微分先行：`derivative = -(feedback - last_feedback) / period`（对测量值微分，设定值阶跃不产生微分冲击）
- 增量式的微分是二阶差分：`(error - 2·last_error + last_2_error) / period`

### 前馈

- 位置式：`output += feedforward · feedforward_gain`（直接加）
- 增量式：`output += (feedforward - last_feedforward) · feedforward_gain`（加增量）

### 抗积分饱和（仅位置式）

采用**条件积分法**：开启 `OUTPUT_LIMIT` 且输出已饱和、误差仍在把输出往外推时，冻结积分。判断用**上一周期**的 `output`。

> 局限：只处理**自身**输出限幅导致的饱和，**不感知外部的二次限流**（例如底盘功率控制对电流的缩放）。这种场景需要在库外拿真实输出回灌做反算（back-calculation），库内没有此能力。

## 注意事项 / 已知问题

1. **`period` 必须等于实际调用周期**（单位 ms，uint8_t，最大 255）。它同时影响积分（×period）和微分（÷period），配错会导致 ki/kd 实际作用强度改变。
2. **两种模式 `integral` 语义不同**（见上文），不要跨模式理解。
3. **`feedforward_gain == 0` 会被静默置为 1.0**（当作「未设置」处理）。
4. **`filter_alpha ≤ 0` 默认 0.3，`> 1` 钳到 1.0**；该默认值在未开 `FILTER` 特性时也会写入（无副作用）。
5. **`period == 0` 默认 1**（即 1ms）。
6. **`PID_Reset` 不重置 `last_feedforward`**：复位后立即在增量式下用前馈，首帧可能多一个 `-last_feedforward·gain` 的假增量（罕见，一般无感）。
7. **`PID_Init` 不检查 `config` / `malloc` 是否为 NULL**（嵌入式 malloc 极少失败，故省略）。
8. **`pid_test.h` 是过时副本**：其 `PID_Instance` / `PID_Init_Config_s` 缺 `feedforward` 相关字段，枚举名 `PID_FEATURE_FEED_FORWARD` 与 `pid.h` 的 `PID_FEATURE_FEEDFORWARD` 不一致；`pid_test.c` 引的是这份旧定义，因只测 `PID_Set_Parameters` 未暴露。建议测试直接 `#include "pid.h"`。
