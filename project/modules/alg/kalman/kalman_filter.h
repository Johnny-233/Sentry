/**
 * @file    kalman_filter.h
 * @brief   卡尔曼滤波器（线性 KF / EKF）模板类声明
 * @note    从 C 版 kalman_filter 迁移：状态维度 _n、观测维度 _m 为编译期常量，
 *          矩阵运算基于 Matrixf（matrix.hpp）。本类为「模板 + 逻辑」，实现见 .cpp
 *          （显式实例化），非 header-only。
 */
#pragma once

#include "matrix.hpp"

/**
 * @brief 卡尔曼滤波器（线性 KF 与 EKF）
 * @tparam _n 状态维度
 * @tparam _m 观测维度
 */
template <int _n, int _m>
class KalmanFilter {
    static_assert(_n > 0 && _m > 0, "KF 维度必须为正");

public:
    // 计算钩子：第一参数为强类型实例引用（对应 C 版 KF_Instance*）
    using PredictCallback = void (*)(KalmanFilter& kf, float* F, const float* u);
    using UpdateCallback  = void (*)(KalmanFilter& kf, float* H, float* h_x, const float* z);

    // 编译期维度（对应 C 版 kf->n / kf->m，供外部读取）
    static constexpr int kStateDim = _n;
    static constexpr int kObsDim   = _m;

    /// 初始化配置
    struct Config {
        const float* x_init;   ///< 初始状态 [_n]，可为 nullptr（默认零向量）
        const float* P_init;   ///< 初始协方差 [_n×_n]，可为 nullptr（默认单位阵）
        float lambda;          ///< 渐消因子 (0~1]，<=0 或 >1 取 1（标准 KF）
    };

    // —— 状态数据（公开，供外部直接读，对应 C 版 kf->x / kf->P）——
    Matrixf<_n, 1> x_;         ///< 状态向量（列向量）
    Matrixf<_n, _n> P_;        ///< 协方差矩阵

    // —— 生命周期 ——
    void init(const Config& config);                     ///< 替代 KF_Register（无堆，无需 free）
    void reset(const float* x_new, const float* P_new);  ///< 替代 KF_Reset

    // —— 回调 ——
    void setPredictCallback(PredictCallback cb);  ///< 替代 KF_Set_Predict_Callback
    void setUpdateCallback(UpdateCallback cb);    ///< 替代 KF_Set_Update_Callback

    // —— 滤波 ——
    void predict(const Matrixf<_n, _n>& F, const Matrixf<_n, _n>& Q);          ///< 线性预测
    bool update(const Matrixf<_m, 1>& z, const Matrixf<_m, _n>& H, const Matrixf<_m, _m>& R);  ///< 线性更新
    void predictEKF(const Matrixf<_n, _n>& Q, const float* u);                 ///< EKF 预测
    bool updateEKF(const Matrixf<_m, 1>& z, const Matrixf<_m, _m>& R, const float* u);         ///< EKF 更新
    float chiSquare(const Matrixf<_m, 1>& z, const Matrixf<_m, _n>& H);       ///< 卡方检验

private:
    float lambda_ = 1.0f;                        ///< 渐消因子
    PredictCallback predict_callback_ = nullptr;
    UpdateCallback update_callback_ = nullptr;
};
