# 卡尔曼滤波模块重构计划

## 一、模块定位

通用卡尔曼滤波库，通过回调函数扩展为 EKF。不实现 UKF。

- `KF_Predict` / `KF_Update`：线性 KF
- `KF_Predict_EKF` / `KF_Update_EKF`：EKF（回调计算 F/H 矩阵）
- `KF_ChiSquare`：卡方检验

## 二、五步公式分解

### 公式 1：状态预测 x⁻ = F × x̂，P⁻ = F × P × F^T + Q

| 子步骤 | 运算 | 读入 | 写出 | 状态 |
|--------|------|------|------|------|
| 1.1 | x⁻ = F × x̂ | F, x̂ | tmp_n → x̂ | ☐ |
| 1.2 | FP = F × P | F, P | tmp_nn(FP) | ☐ |
| 1.3 | FT = F^T | F | tmp_nn2(FT) | ☐ |
| 1.4 | FPT = FP × FT | tmp_nn(FP), tmp_nn2(FT) | tmp_nn2(FPT) | ☐ |
| 1.5 | P⁻ = FPT | tmp_nn2(FPT) | kf->P | ☐ |
| 1.6 | P⁻ += Q | P, Q | kf->P | ☐ |
| 1.7 | P⁻ *= λ | P | kf->P | ☐ |

### 公式 2：新息 y = z - H × x⁻

| 子步骤 | 运算 | 读入 | 写出 | 状态 |
|--------|------|------|------|------|
| 2.1 | Hx = H × x⁻ | H, x⁻ | tmp_m | ☐ |
| 2.2 | y = z - Hx | z, tmp_m | tmp_m（原地sub） | ☐ |

### 公式 3：新息协方差 S = H × P⁻ × H^T + R

| 子步骤 | 运算 | 读入 | 写出 | 状态 |
|--------|------|------|------|------|
| 3.1 | HP = H × P⁻ | H, P | tmp_nm(HP) | ☐ |
| 3.2 | HT = H^T | H | tmp_nn(HT) | ☐ |
| 3.3 | S = HP × HT | tmp_nm, tmp_nn | tmp_mm(S) | ☐ |
| 3.4 | S += R | tmp_mm, R | tmp_mm | ☐ |

### 公式 4：卡尔曼增益 K = P⁻ × H^T × S⁻¹

| 子步骤 | 运算 | 读入 | 写出 | 状态 |
|--------|------|------|------|------|
| 4.1 | PHt = P⁻ × HT | P, tmp_nn(HT) | tmp_nm(PHt) | ☐ |
| 4.2 | S⁻¹ = inv(S) | tmp_mm(S) | tmp_nn(S⁻¹) | ☐ |
| 4.3 | K = PHt × S⁻¹ | tmp_nm, tmp_nn | tmp_nn(K) | ☐ |

### 公式 5a：状态更新 x̂ = x̂⁻ + K × y

| 子步骤 | 运算 | 读入 | 写出 | 状态 |
|--------|------|------|------|------|
| 5a.1 | Ky = K × y | tmp_nn(K), tmp_m(y) | tmp_n(Ky) | ☐ |
| 5a.2 | x̂ += Ky | x̂, tmp_n | x̂ | ☐ |

### 公式 5b：协方差更新 P = (I - K × H) × P⁻

| 子步骤 | 运算 | 读入 | 写出 | 状态 |
|--------|------|------|------|------|
| 5b.1 | KH = K × H | tmp_nn(K), H | tmp_nn2(KH) | ☐ |
| 5b.2 | IKH = I - KH | tmp_nn2, I | tmp_nn(IKH) | ☐ |
| 5b.3 | P = IKH × P⁻ | tmp_nn, P | tmp_nn2 | ☐ |
| 5b.4 | copy | tmp_nn2 | P | ☐ |

## 三、缓冲区需求分析

**n×n 缓冲区**：最多同时需要 **2 个**（如步骤 1.2+1.3 需要 FP 和 FT 同时存在）

| 缓冲区 | 维度 | 用途 |
|--------|------|------|
| kf->x | n×1 | 状态向量 |
| kf->P | n×n | 协方差矩阵 |
| tmp_nn | n×n | 临时矩阵 |
| tmp_nn2 | n×n | 备用临时矩阵 |
| tmp_n | n×1 | 临时向量 |
| tmp_m | m×1 | 临时向量 |
| tmp_nm | n×m | 临时矩阵（HP/PHt/K） |
| tmp_mm | m×m | 临时矩阵（S） |

**实际最小需求：2 个 n×n + 1 个 n×m + 1 个 m×m + 1 个 n×1 + 1 个 m×1 = 6 个缓冲区**

tmp_nn 和 tmp_nn2 是仅有的两个 n×n 临时矩阵，周转使用。

## 四、CMSIS DSP 原地操作规则

| 函数 | 支持原地 | 说明 |
|------|---------|------|
| arm_mat_add_f32 | ✓ | C=A+B，A/B/C 可重叠 |
| arm_mat_sub_f32 | ✓ | C=A-B，A/B/C 可重叠 |
| arm_mat_trans_f32 | ✓（方阵） | 非方阵需独立缓冲 |
| arm_mat_mult_f32 | ✗ | 输出不能与任一输入重叠 |
| arm_mat_inverse_f32 | ✓ | 内部使用临时缓冲 |

## 五、仍需修复的原地 mat_mul

| 位置 | 当前代码 | 修复 |
|------|---------|------|
| 公式1 步骤1.4 | `MAT_MUL(tmp_nn2, tmp_nn, tmp_nn2)` | 先算到 tmp_nn，再 copy 到 tmp_nn2 |
| 公式4 步骤4.3 | `MAT_MUL(tmp_nn, tmp_nn, tmp_nm)` | 先算到 tmp_nn2，再 copy 到 tmp_nn |

## 六、状态

- [ ] 公式 1 修复原地 mat_mul
- [ ] 公式 4 修复原地 mat_mul
- [ ] 编译验证
- [ ] 调试验证无 HardFault
- [ ] 姿态解算输出正常
