/**
 * @file    kalman_filter.cpp
 * @brief   卡尔曼滤波器模板实现 + 显式实例化
 * @note    模板定义集中于此，通过显式实例化支持本项目用到的维度组合，
 *          替代 C 版运行时 malloc 缓冲与手写 CMSIS 包装。
 */
#include "kalman_filter.h"

#include "matrix.hpp"
// ============================== 生命周期 ==============================

template <int _n, int _m>
void KalmanFilter<_n, _m>::init(const Config& config) {
    lambda_ = (config.lambda > 0.0f && config.lambda <= 1.0f) ? config.lambda : 1.0f;

    // 注：C 版误用 mat_zero(x, n) 清零 n×n，越界写入 x 之后的 30 个 float；
    //     此处只清 _n 个（保持“x 清零”本意，消除越界）。
    if (config.x_init) {
        for (int i = 0; i < _n; i++) x_[i][0] = config.x_init[i];
    } else {
        x_ = Matrixf<_n, 1>::zeros();
    }

    if (config.P_init) {
        for (int r = 0; r < _n; r++)
            for (int c = 0; c < _n; c++)
                P_[r][c] = config.P_init[r * _n + c];
    } else {
        P_ = Matrixf<_n, _n>::eye();
    }
}

template <int _n, int _m>
void KalmanFilter<_n, _m>::reset(const float* x_new, const float* P_new) {
    if (x_new) {
        for (int i = 0; i < _n; i++) x_[i][0] = x_new[i];
    } else {
        x_ = Matrixf<_n, 1>::zeros();
    }
    if (P_new) {
        for (int r = 0; r < _n; r++)
            for (int c = 0; c < _n; c++)
                P_[r][c] = P_new[r * _n + c];
    } else {
        P_ = Matrixf<_n, _n>::eye();
    }
}

template <int _n, int _m>
void KalmanFilter<_n, _m>::setPredictCallback(PredictCallback cb) {
    predict_callback_ = cb;
}

template <int _n, int _m>
void KalmanFilter<_n, _m>::setUpdateCallback(UpdateCallback cb) {
    update_callback_ = cb;
}

// ============================== 线性卡尔曼滤波 ==============================

template <int _n, int _m>
void KalmanFilter<_n, _m>::predict(const Matrixf<_n, _n>& F, const Matrixf<_n, _n>& Q) {
    // 1. x = F × x
    x_ = F * x_;
    // 2. P = F × P × F^T
    P_ = F * P_ * F.trans();
    // 渐消因子
    if (lambda_ < 1.0f) P_ *= lambda_;
    // P = P + Q
    P_ += Q;
}

template <int _n, int _m>
bool KalmanFilter<_n, _m>::update(const Matrixf<_m, 1>& z, const Matrixf<_m, _n>& H, const Matrixf<_m, _m>& R) {
    // 1. y = z - H × x
    Matrixf<_m, 1> y = z - H * x_;
    // 2. S = H × P × H^T + R
    Matrixf<_m, _m> S = H * P_ * H.trans() + R;
    // 3. K = P × H^T × S^(-1)
    Matrixf<_m, _m> S_inv = S.inv();
    if (S_inv == Matrixf<_m, _m>::zeros()) return false;   // 奇异
    Matrixf<_n, _m> K = P_ * H.trans() * S_inv;
    // 4. x = x + K × y
    x_ += K * y;
    // 5. P = (I - K × H) × P
    P_ = (Matrixf<_n, _n>::eye() - K * H) * P_;
    return true;
}

// ============================== EKF ==============================

template <int _n, int _m>
void KalmanFilter<_n, _m>::predictEKF(const Matrixf<_n, _n>& Q, const float* u) {
    if (!predict_callback_) return;
    Matrixf<_n, _n> F;
    predict_callback_(*this, F[0], u);   // 回调计算 F（写入 F 原始缓冲）
    predict(F, Q);
}

template <int _n, int _m>
bool KalmanFilter<_n, _m>::updateEKF(const Matrixf<_m, 1>& z, const Matrixf<_m, _m>& R, const float* u) {
    if (!update_callback_) return false;

    // 回调计算 H 和 h(x)（写入 H/h_x 原始缓冲；注：C 版此处第 4 参传的是 u 而非 z，原样保留）
    Matrixf<_m, _n> H;
    Matrixf<_m, 1> h_x;
    update_callback_(*this, H[0], h_x[0], u);

    // 1. y = z - h(x)
    Matrixf<_m, 1> y = z - h_x;
    // 2. S = H × P × H^T + R
    Matrixf<_m, _m> S = H * P_ * H.trans() + R;
    // 3. K = P × H^T × S^(-1)
    Matrixf<_m, _m> S_inv = S.inv();
    if (S_inv == Matrixf<_m, _m>::zeros()) return false;
    Matrixf<_n, _m> K = P_ * H.trans() * S_inv;
    // 4. x = x + K × y
    x_ += K * y;
    // 5. P = (I - K × H) × P
    P_ = (Matrixf<_n, _n>::eye() - K * H) * P_;
    return true;
}

// ============================== 卡方检验 ==============================

template <int _n, int _m>
float KalmanFilter<_n, _m>::chiSquare(const Matrixf<_m, 1>& z, const Matrixf<_m, _n>& H) {
    // y = z - H × x
    Matrixf<_m, 1> y = z - H * x_;
    // S = H × P × H^T
    Matrixf<_m, _m> S = H * P_ * H.trans();
    // S^(-1)
    Matrixf<_m, _m> S_inv = S.inv();
    if (S_inv == Matrixf<_m, _m>::zeros()) return -1.0f;
    // y^T × S^(-1) × y
    Matrixf<1, 1> chi = y.trans() * S_inv * y;
    return chi[0][0];
}

// ============================== 显式实例化 ==============================
// 本项目仅 ahrs 使用，维度组合由 AHRS_USE_MAGNETOMETER 决定（见 ahrs.h）：
//   无磁力计 n=6, m=3；有磁力计 n=7, m=6。

template class KalmanFilter<6, 3>;
template class KalmanFilter<7, 6>;
