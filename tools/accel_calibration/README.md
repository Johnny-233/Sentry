# 加速度计椭球拟合校准

## 原理

BMI088 加速度计静止时，三轴读数的轨迹理论上是一个球面（半径 = 1g）。由于制造误差，实际轨迹是一个椭球。椭球拟合可以计算出修正参数，将椭球"压"成球。

修正公式：
```
corrected = M × (raw - offset)
```

- `offset[3]`：椭球中心偏移（零偏）
- `M[9]`：3×3 修正矩阵（scale + 非正交性修正）

## 快速开始（开发者）

### 1. 采集数据

在 Ozone 调试时观察 `bmi088->accel.x/y/z`，记录 **6 个以上姿态**的静态值：

| 姿态 | ax | ay | az |
|------|----|----|-----|
| 水平朝上 | 0.12 | -0.45 | 9.67 |
| 水平朝下 | -0.08 | 0.32 | -9.71 |
| 朝前 | 9.65 | 0.11 | 0.23 |
| 朝后 | -9.72 | -0.09 | 0.18 |
| 朝左 | 0.15 | 9.68 | -0.12 |
| 朝右 | -0.11 | -9.73 | 0.08 |
| ... | ... | ... | ... |

每个姿态记 5-10 组值，总共 30-60 行数据。

### 2. 保存为 CSV

保存到 `accel_data.csv`，格式：
```csv
ax,ay,az
0.12,-0.45,9.67
-0.08,0.32,-9.71
9.65,0.11,0.23
...
```

### 3. 运行拟合

```bash
pip install numpy scipy  # 首次需要
python tools/accel_calibration/calibrate.py --csv accel_data.csv
```

### 4. 应用校准

把输出的 C 代码填入 `BMI088_Init_Config_s`：
```c
BMI088_Init_Config_s bmi088_config = {
    .spi_acc_config = { ... },
    .spi_gyro_config = { ... },
    .accel_range = BMI088_ACC_RANGE_6G_E,
    .gyro_range = BMI088_GYRO_RANGE_2000_E,
    .accel_offset = {0.012f, -0.008f, 0.034f},           // 从输出复制
    .accel_M = {1.002f, 0.001f, -0.003f, ...},            // 从输出复制
};
```

## 输出示例

```
拟合完成!

偏移量 (offset):
  [0.012, -0.008, 0.034]

修正矩阵 (M):
  [+1.002 +0.001 -0.003]
  [-0.001 +0.998 +0.002]
  [+0.003 -0.002 +1.001]

校准效果:
  校准前: 均值=9.7912, 标准差=0.042100
  校准后: 均值=9.8066, 标准差=0.000312

C 代码:
.accel_offset = {0.012000f, -0.008000f, 0.034000f},
.accel_M = {1.002000f, 0.001000f, -0.003000f, -0.001000f, 0.998000f, 0.002000f, 0.003000f, -0.002000f, 1.001000f},
```

## 输出文件

- `output/calibration.h`：C 头文件，定义 `ACCEL_OFFSET` 和 `ACCEL_M` 宏
- `output/calibration.json`：JSON 格式

## 依赖

```bash
pip install numpy scipy  # 必需
pip install matplotlib   # 可选（可视化）
```

## 可视化验证

```bash
python tools/accel_calibration/calibrate.py --csv accel_data.csv --plot
```

## FAQ

**Q: 数据点需要多少？**
A: 至少 6 个姿态，每个 5-10 组值，总共 30-60 行即可。

**Q: 必须取平均再拟合吗？**
A: 不需要。椭球拟合用最小二乘法，天然处理噪声。原始数据点越多越好。

**Q: 什么时候需要重新校准？**
A: 通常不需要。更换 BMI088 芯片后建议重新校准。
