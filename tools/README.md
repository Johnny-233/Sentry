# tools/ - 数学工具集

离线工具目录，存放仿真模型、拟合计算等数学工具。

## 目录结构

```
tools/
├── README.md                   # 本文件
├── accel_calibration/          # 加速度计椭球拟合校准
│   ├── README.md               # 使用说明
│   ├── calibrate.py            # 主脚本：数据采集 + 拟合 + 输出
│   ├── ellipsoid_fit.py        # 椭球拟合算法
│   └── output/                 # 输出校准参数
└── simulation/                 # 仿真模型（预留）
    └── README.md
```

## 依赖

- Python 3.8+
- numpy
- scipy
- matplotlib（可选，用于可视化验证）

安装：
```bash
pip install numpy scipy matplotlib
```
