"""
椭球拟合算法

基于最小二乘法拟合椭球参数，输出 offset 和修正矩阵 M。
参考论文: "Least-Squares Fitting of Ellipsoids" by Yang et al.
"""

import numpy as np


def ellipsoid_fit(data: np.ndarray) -> tuple:
    """
    椭球拟合

    参数:
        data: N×3 数组，每行是 [ax, ay, az]

    返回:
        offset: 3 维偏移量
        M: 3×3 修正矩阵
    """
    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]

    # 椭球方程: a11*x^2 + a22*y^2 + a33*z^2 + 2*a12*x*y + 2*a13*x*z + 2*a23*y*z + 2*b1*x + 2*b2*y + 2*b3*z + c = 0
    # 构建最小二乘问题: D × params = -1
    D = np.column_stack([
        x**2, y**2, z**2,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, z
    ])

    # 最小二乘求解
    params, residuals, rank, sv = np.linalg.lstsq(D, -np.ones(len(x)), rcond=None)

    a11, a22, a33, a12, a13, a23, b1, b2, b3 = params
    c = 1.0

    # 构建矩阵 A 和向量 B
    A = np.array([
        [a11, a12, a13],
        [a12, a22, a23],
        [a13, a23, a33]
    ])
    B = np.array([b1, b2, b3])

    # 椭球中心 (偏移量)
    offset = -np.linalg.solve(A, B)

    # 椭球的轴长和旋转
    # 标准化: (x - center)^T × A × (x - center) = -c + B^T × A^(-1) × B
    center_term = B.T @ np.linalg.solve(A, B)
    A_normalized = A / (-c + center_term)

    # 特征值分解
    eigenvalues, eigenvectors = np.linalg.eigh(A_normalized)

    # 修正矩阵: M = eigenvectors × diag(1/sqrt(eigenvalues)) × eigenvectors^T
    scales = 1.0 / np.sqrt(np.abs(eigenvalues))
    M = eigenvectors @ np.diag(scales) @ eigenvectors.T

    return offset, M


def verify_calibration(data: np.ndarray, offset: np.ndarray, M: np.ndarray) -> dict:
    """
    验证校准效果

    参数:
        data: 原始数据 N×3
        offset: 偏移量
        M: 修正矩阵

    返回:
        包含校准前后统计信息的字典
    """
    # 校准前
    norms_before = np.linalg.norm(data, axis=1)

    # 校准后
    corrected = (M @ (data - offset).T).T
    norms_after = np.linalg.norm(corrected, axis=1)

    return {
        "before": {
            "mean": float(np.mean(norms_before)),
            "std": float(np.std(norms_before)),
            "min": float(np.min(norms_before)),
            "max": float(np.max(norms_before)),
        },
        "after": {
            "mean": float(np.mean(norms_after)),
            "std": float(np.std(norms_after)),
            "min": float(np.min(norms_after)),
            "max": float(np.max(norms_after)),
        }
    }


def to_c_code(offset: np.ndarray, M: np.ndarray) -> str:
    """
    生成 C 代码格式的校准参数
    """
    lines = []
    lines.append(f"// 加速度计椭球拟合校准参数")
    lines.append(f".accel_offset = {{{offset[0]:.6f}f, {offset[1]:.6f}f, {offset[2]:.6f}f}},")
    m_flat = M.flatten()
    m_str = ", ".join(f"{v:.6f}f" for v in m_flat)
    lines.append(f".accel_M = {{{m_str}}},")
    return "\n".join(lines)


def to_json(offset: np.ndarray, M: np.ndarray) -> dict:
    """
    生成 JSON 格式的校准参数
    """
    return {
        "offset": offset.tolist(),
        "M": M.tolist()
    }
