/**
 * @file    ahrs.h
 * @brief   AHRS 姿态解算（EKF 六轴，C → C++）
 * @note    从 C 版 ahrs 迁移：struct AHRS_Instance → class AHRS；Register → init、
 *          Preheat → preheat、Calibrate → calibrate、Start → start、Update → update；
 *          卡尔曼滤波器改用 KalmanFilter<6,3> 模板类（禁堆），BMI088/IMUTemp 内嵌为成员。
 *          磁力计融合默认关闭（AHRS_USE_MAGNETOMETER=0）。
 */
#pragma once

#include "bmi088.h"
#include "kalman_filter.h"
#include "bsp_dwt.h"
#include "imu_temp.h"

/*============================================
 * 功能开关
 * AHRS_USE_MAGNETOMETER: 启用/禁用磁力计融合
 *   0 = 仅使用陀螺仪+加速度计 (俯仰/横滚修正)
 *   1 = 使用陀螺仪+加速度计+磁力计 (偏航也修正)
 ============================================*/

#ifndef AHRS_USE_MAGNETOMETER
#define AHRS_USE_MAGNETOMETER    0
#endif

#if AHRS_USE_MAGNETOMETER
#include "ist8310.h"
#endif

/*============================================
 * 校验开关
 * AHRS_ENABLE_VALIDATION: 启用/禁用 EKF 校验
 *   0 = 不校验（默认，简洁快速）
 *   1 = 启用模长校验、协方差边界检查等
 ============================================*/

#ifndef AHRS_ENABLE_VALIDATION
#define AHRS_ENABLE_VALIDATION    1
#endif

/*============================================
 * 动态依赖切换
 ============================================*/

#if defined(AHRS_DISABLE_RTOS)
    #define AHRS_RTOS_SUPPORT 0
#elif __has_include("cmsis_os2.h")
    #include "cmsis_os2.h"
    #define AHRS_RTOS_SUPPORT 2
#elif __has_include("cmsis_os.h")
    #include "cmsis_os.h"
    #define AHRS_RTOS_SUPPORT 1
#else
    #define AHRS_RTOS_SUPPORT 0
#endif

/*============================================
 * 状态维度定义
 *
 * 无磁力计: n=6 (q0,q1,q2,q3, bx,by)
 * 有磁力计: n=7 (q0,q1,q2,q3, bx,by, bz)
 *
 * 观测维度定义
 * 无磁力计: m=3 (ax,ay,az)
 * 有磁力计: m=6 (ax,ay,az, mx,my,mz)
 ============================================*/

#if AHRS_USE_MAGNETOMETER
    #define AHRS_STATE_DIM     7
    #define AHRS_OBS_DIM       6
#else
    #define AHRS_STATE_DIM     6
    #define AHRS_OBS_DIM       3
#endif

/*============================================
 * 温控周期
 ============================================*/

#define AHRS_TEMP_CTRL_PERIOD_MS    750     // 温控采样周期 [ms]

/**
 * @brief AHRS 姿态解算（EKF 六轴）
 */
class AHRS {
public:
    /// EKF 类型（编译期维度，无磁力计 n=6/m=3）
    using KF = KalmanFilter<AHRS_STATE_DIM, AHRS_OBS_DIM>;

    /// 输出结构体（跨模块读取）
    struct Output {
        float q[4];                 // 四元数 (q0, q1, q2, q3)
        float euler[3];             // 欧拉角 (roll, pitch, yaw) [deg]
        float yaw_total;            // 累计偏航角 [-inf, +inf] deg

        // 角速度（机体坐标系，安装角旋转+零偏去除后）[rad/s]
        float gyro_b[3];            // (x, y, z)

        // 加速度（机体坐标系，安装角旋转+低通滤波后，含重力）[m/s²]
        float accel_b[3];           // (x, y, z)

#if AHRS_USE_MAGNETOMETER
        // 磁力计（机体坐标系，安装角旋转后）
        float mag_b[3];             // (x, y, z) [原始单位]
#endif

        // 运动加速度（去重力）[m/s²]
        float motion_accel_b[3];    // 机体坐标系
        float motion_accel_n[3];    // 世界坐标系
    };

    /// 初始化配置
    struct Config {
        BMI088::AccRange accel_range;
        BMI088::GyroRange gyro_range;
#if AHRS_USE_MAGNETOMETER
        IST8310::Config ist8310_config;
#endif
        float board_yaw;         // 板子相对云台的偏航角 [deg]（正值=板子逆时针转）
        float board_pitch;       // 板子相对云台的俯仰角 [deg]（正值=板子低头）
        float board_roll;        // 板子相对云台的横滚角 [deg]（正值=板子右倾）
        float accel_lpf_coef;     // 加速度低通滤波系数 [s]

        // EKF 噪声配置 (<=0 时使用默认值)
        float process_noise_quat;    // Q: 四元数过程噪声系数，默认 10.0f
        float process_noise_bias;    // Q: 零偏随机游走系数，默认 0.001f
        float obs_noise_accel;       // R: 加速度计观测噪声，默认 1000000.0f
#if AHRS_USE_MAGNETOMETER
        float obs_noise_mag;         // R: 磁力计观测噪声，默认 1000000.0f
#endif

        // 预热配置 (<=0 时使用默认值)
        uint32_t preheat_timeout_ms; // 预热超时 [ms]，默认 30000
    };

    // —— 状态数据（公开，跨模块直接读）——
    Output output_ = {};            ///< 姿态/角速度/加速度输出
    uint32_t preheat_elapsed_ms_ = 0;   ///< 预热实际耗时 [ms]

    // —— 生命周期 ——
    void init(const Config& config);   ///< 替代 AHRS_Register（外设初始化，禁堆）
    void preheat();                    ///< 替代 AHRS_Preheat（阻塞加热到 40°C）
    void calibrate();                  ///< 替代 AHRS_Calibrate（阻塞陀螺仪校准）
    void start();                      ///< 替代 AHRS_Start（创建 RTOS 任务）
#if !AHRS_RTOS_SUPPORT
    void task();                       ///< 替代裸机 AHRS_Task（1ms 周期调用 update）
#endif

private:
    // —— 内部机制 ——
    BMI088 bmi088_;                    ///< 六轴 IMU（内嵌，禁堆）
#if AHRS_USE_MAGNETOMETER
    IST8310 ist8310_;                  ///< 磁力计（内嵌，禁堆）
#endif
    KF kf_;                            ///< EKF（编译期维度，无堆）
    IMUTemp imu_temp_;                 ///< 温控（内嵌，禁堆）

    float install_rot_[9] = {};        ///< 安装角旋转矩阵 (S→B) 3×3
    float accel_lpf_coef_ = 0.0f;      ///< 加速度低通滤波系数 [s]
    float gyro_bias_[3] = {};          ///< 陀螺仪零偏 [rad/s]
    uint32_t dwt_cnt_ = 0;             ///< DWT 计数器
    int16_t yaw_round_count_ = 0;      ///< 偏航圈数计数
    float yaw_last_ = 0.0f;            ///< 上次偏航角 [deg]

    float process_noise_quat_ = 0.0f;
    float process_noise_bias_ = 0.0f;
    float obs_noise_accel_ = 0.0f;
#if AHRS_USE_MAGNETOMETER
    float obs_noise_mag_ = 0.0f;
#endif
    uint32_t preheat_timeout_ms_ = 0;

    Matrixf<AHRS_OBS_DIM, AHRS_OBS_DIM> r_accel_;   ///< 加速度计观测噪声矩阵 R（常量）

#if !AHRS_RTOS_SUPPORT
    uint32_t task_last_tick_ = 0;      ///< 裸机 task 节流计数
#endif

    // —— 内部方法 ——
    void initQuaternion();                        ///< 用加速度计初始化四元数
    void update();                                ///< 核心 EKF 更新（1kHz）
    static void predictCallback(KF& kf, float* F, const float* u);
    static void updateAccelCallback(KF& kf, float* H, float* h_x, const float* z);
#if AHRS_USE_MAGNETOMETER
    static void updateMagCallback(KF& kf, float* H, float* h_x, const float* z);
    static void update6DCallback(KF& kf, float* H, float* h_x, const float* z);
#endif
#if AHRS_RTOS_SUPPORT
    static void internalTask(void* arg);          ///< RTOS 任务入口
#endif
};
