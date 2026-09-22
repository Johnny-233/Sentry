"""
加速度计校准工具

用法:
    python calibrate.py --csv accel_data.csv
    python calibrate.py --port COM3 --baud 115200 --samples 2000
    python calibrate.py --csv accel_data.csv --plot
"""

import argparse
import csv
import json
import sys
import os
import numpy as np
from pathlib import Path

from ellipsoid_fit import ellipsoid_fit, verify_calibration, to_c_code, to_json


def load_csv(filepath: str) -> np.ndarray:
    """从 CSV 文件加载数据"""
    data = []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            # 跳过表头
            try:
                vals = [float(x.strip()) for x in row]
                if len(vals) == 3:
                    data.append(vals)
            except ValueError:
                continue

    if len(data) < 10:
        print(f"错误: 数据太少 ({len(data)} 行)，至少需要 10 行")
        sys.exit(1)

    return np.array(data)


def collect_from_serial(port: str, baud: int, samples: int) -> np.ndarray:
    """从串口采集数据"""
    try:
        import serial
    except ImportError:
        print("错误: 需要安装 pyserial: pip install pyserial")
        sys.exit(1)

    print(f"连接串口 {port} (波特率 {baud})...")
    ser = serial.Serial(port, baud, timeout=1)

    print(f"采集中，请缓慢转动板子... (目标 {samples} 个样本)")
    data = []
    while len(data) < samples:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            try:
                # 期望格式: ax,ay,az 或 ax ay az
                parts = line.replace(',', ' ').split()
                if len(parts) == 3:
                    vals = [float(x) for x in parts]
                    data.append(vals)
                    if len(data) % 100 == 0:
                        print(f"  已采集 {len(data)}/{samples}")
            except ValueError:
                continue

    ser.close()
    print(f"采集完成: {len(data)} 个样本")
    return np.array(data)


def save_output(offset: np.ndarray, M: np.ndarray, output_dir: str):
    """保存校准结果"""
    os.makedirs(output_dir, exist_ok=True)

    # C 头文件
    c_path = os.path.join(output_dir, "calibration.h")
    with open(c_path, 'w') as f:
        f.write("#ifndef ACCEL_CALIBRATION_H\n")
        f.write("#define ACCEL_CALIBRATION_H\n\n")
        f.write("// 自动生成的加速度计校准参数\n")
        f.write("// 使用 tools/accel_calibration/calibrate.py 生成\n\n")
        f.write("// BMI088_Init_Config_s 初始化示例:\n")
        f.write("// .accel_offset = ACCEL_OFFSET,\n")
        f.write("// .accel_M = ACCEL_M,\n\n")
        f.write(f"#define ACCEL_OFFSET {{{offset[0]:.6f}f, {offset[1]:.6f}f, {offset[2]:.6f}f}}\n")
        m_flat = M.flatten()
        m_str = ", ".join(f"{v:.6f}f" for v in m_flat)
        f.write(f"#define ACCEL_M {{{m_str}}}\n\n")
        f.write("#endif\n")
    print(f"已保存 C 头文件: {c_path}")

    # JSON
    json_path = os.path.join(output_dir, "calibration.json")
    with open(json_path, 'w') as f:
        json.dump(to_json(offset, M), f, indent=2)
    print(f"已保存 JSON: {json_path}")


def main():
    parser = argparse.ArgumentParser(description="加速度计椭球拟合校准工具")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--csv", help="从 CSV 文件加载数据")
    group.add_argument("--port", help="串口号 (如 COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="串口波特率 (默认 115200)")
    parser.add_argument("--samples", type=int, default=2000, help="采集样本数 (默认 2000)")
    parser.add_argument("--plot", action="store_true", help="显示校准前后对比图")
    parser.add_argument("--output", default="output", help="输出目录 (默认 output/)")
    args = parser.parse_args()

    # 加载数据
    if args.csv:
        data = load_csv(args.csv)
        print(f"从 {args.csv} 加载了 {len(data)} 个样本")
    else:
        data = collect_from_serial(args.port, args.baud, args.samples)

    print(f"\n数据范围:")
    print(f"  X: [{data[:,0].min():.3f}, {data[:,0].max():.3f}]")
    print(f"  Y: [{data[:,1].min():.3f}, {data[:,1].max():.3f}]")
    print(f"  Z: [{data[:,2].min():.3f}, {data[:,2].max():.3f}]")

    # 椭球拟合
    print("\n拟合中...")
    offset, M = ellipsoid_fit(data)

    # 验证
    stats = verify_calibration(data, offset, M)

    print(f"\n拟合完成!")
    print(f"\n偏移量 (offset):")
    print(f"  [{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}]")
    print(f"\n修正矩阵 (M):")
    for row in M:
        print(f"  [{row[0]:+.6f} {row[1]:+.6f} {row[2]:+.6f}]")

    print(f"\n校准效果:")
    print(f"  校准前: 均值={stats['before']['mean']:.4f}g, 标准差={stats['before']['std']:.6f}")
    print(f"  校准后: 均值={stats['after']['mean']:.4f}g, 标准差={stats['after']['std']:.6f}")

    print(f"\nC 代码:")
    print(to_c_code(offset, M))

    # 保存输出
    save_output(offset, M, args.output)

    # 可视化
    if args.plot:
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D

            corrected = (M @ (data - offset).T).T

            fig = plt.figure(figsize=(12, 5))

            ax1 = fig.add_subplot(121, projection='3d')
            ax1.scatter(data[:, 0], data[:, 1], data[:, 2], s=1, alpha=0.5)
            ax1.set_title("校准前")
            ax1.set_xlabel("X")
            ax1.set_ylabel("Y")
            ax1.set_zlabel("Z")

            ax2 = fig.add_subplot(122, projection='3d')
            ax2.scatter(corrected[:, 0], corrected[:, 1], corrected[:, 2], s=1, alpha=0.5)
            ax2.set_title("校准后")
            ax2.set_xlabel("X")
            ax2.set_ylabel("Y")
            ax2.set_zlabel("Z")

            plt.tight_layout()
            plt.show()
        except ImportError:
            print("\n提示: 安装 matplotlib 可显示可视化图表: pip install matplotlib")


if __name__ == "__main__":
    main()
