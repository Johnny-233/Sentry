/**
 * @file    ahrs.cpp
 * @brief   AHRS 姿态解算实现（C → C++）
 */
#include "ahrs.h"
#include <string.h>
#include <math.h>

namespace {

constexpr float DEG_TO_RAD = 0.017453292519943295f;
constexpr float RAD_TO_DEG = 57.29577951308232f;

// 快速逆平方根（保持 C 原版位操作，memcpy 规避严格别名 UB）
float invSqrt(float x) {
    float halfx = 0.5f * x;
    int32_t i;
    memcpy(&i, &x, sizeof(i));
    i = 0x5f375a86 - (i >> 1);
    memcpy(&x, &i, sizeof(x));
    x = x * (1.5f - halfx * x * x);
    return x;
}

// 四元数归一化
void quatNormalize(float* q) {
    float norm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-6f) {
        q[0] *= norm; q[1] *= norm; q[2] *= norm; q[3] *= norm;
    }
}

// 四元数转欧拉角 (ZYX 顺序: Yaw-Pitch-Roll)
void quatToEuler(const float* q, float* roll, float* pitch, float* yaw) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    *roll = atan2f(2.0f*(q0*q1 + q2*q3),
                   1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;

    float sinp = 2.0f*(q0*q2 - q3*q1);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(90.0f, sinp);
    } else {
        *pitch = asinf(sinp) * RAD_TO_DEG;
    }

    *yaw = atan2f(2.0f*(q0*q3 + q1*q2),
                  1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
}

// 3D 旋转矩阵乘法: out = R × v
void rotVec(const float* R, const float* v, float* out) {
    out[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
    out[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
    out[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}

}  // namespace

/*============================================
 * EKF 回调
 *
 * 状态向量（无磁力计）: x = [q0, q1, q2, q3, bx, by]^T  (n=6)
 * 状态向量（有磁力计）: x = [q0, q1, q2, q3, bx, by, bz]^T  (n=7)
 ============================================*/

// EKF 预测: 计算 F 矩阵
void AHRS::predictCallback(KF& kf, float* F, const float* u) {
    /*
     * u[0] = dt；u[1] = wx；u[2] = wy；u[3] = wz
     */
    float dt = u[0];
    float wx = u[1];
    float wy = u[2];
    float wz = u[3];
    const int n = AHRS_STATE_DIM;

    memset(F, 0, n * n * sizeof(float));
    for (int i = 0; i < n; i++)
        F[i*n + i] = 1.0f;

    float half_dt = 0.5f * dt;

    F[0*n+1] = -half_dt * wx;
    F[0*n+2] = -half_dt * wy;
    F[0*n+3] = -half_dt * wz;

    F[1*n+0] =  half_dt * wx;
    F[1*n+2] =  half_dt * wz;
    F[1*n+3] = -half_dt * wy;

    F[2*n+0] =  half_dt * wy;
    F[2*n+1] = -half_dt * wz;
    F[2*n+3] =  half_dt * wx;

    F[3*n+0] =  half_dt * wz;
    F[3*n+1] =  half_dt * wy;
    F[3*n+2] = -half_dt * wx;

#if AHRS_USE_MAGNETOMETER
    float q0 = kf.x_[0][0], q1 = kf.x_[1][0], q2 = kf.x_[2][0], q3 = kf.x_[3][0];
    F[0*n+4] =  q1 * half_dt;   F[0*n+5] =  q2 * half_dt;   F[0*n+6] =  q3 * half_dt;
    F[1*n+4] = -q0 * half_dt;   F[1*n+5] =  q3 * half_dt;   F[1*n+6] = -q2 * half_dt;
    F[2*n+4] = -q3 * half_dt;   F[2*n+5] = -q0 * half_dt;   F[2*n+6] =  q1 * half_dt;
    F[3*n+4] =  q2 * half_dt;   F[3*n+5] = -q1 * half_dt;   F[3*n+6] = -q0 * half_dt;
#else
    float q0 = kf.x_[0][0], q1 = kf.x_[1][0], q2 = kf.x_[2][0], q3 = kf.x_[3][0];
    F[0*n+4] =  q1 * half_dt;   F[0*n+5] =  q2 * half_dt;
    F[1*n+4] = -q0 * half_dt;   F[1*n+5] =  q3 * half_dt;
    F[2*n+4] = -q3 * half_dt;   F[2*n+5] = -q0 * half_dt;
    F[3*n+4] =  q2 * half_dt;   F[3*n+5] = -q1 * half_dt;
#endif
}

// EKF 更新: 加速度计观测 Jacobian
void AHRS::updateAccelCallback(KF& kf, float* H, float* h_x, const float* z) {
    (void)z;
    float q0 = kf.x_[0][0], q1 = kf.x_[1][0], q2 = kf.x_[2][0], q3 = kf.x_[3][0];
    const int n = AHRS_STATE_DIM;

    h_x[0] = 2.0f*(q1*q3 - q0*q2);
    h_x[1] = 2.0f*(q0*q1 + q2*q3);
    h_x[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    memset(H, 0, 3 * n * sizeof(float));

    H[0*n+0] = -2.0f*q2;  H[0*n+1] =  2.0f*q3;  H[0*n+2] = -2.0f*q0;  H[0*n+3] =  2.0f*q1;
    H[1*n+0] =  2.0f*q1;  H[1*n+1] =  2.0f*q0;  H[1*n+2] =  2.0f*q3;  H[1*n+3] =  2.0f*q2;
    H[2*n+0] =  2.0f*q0;  H[2*n+1] = -2.0f*q1;  H[2*n+2] = -2.0f*q2;  H[2*n+3] =  2.0f*q3;
}

#if AHRS_USE_MAGNETOMETER
// EKF 更新: 磁力计观测 Jacobian
void AHRS::updateMagCallback(KF& kf, float* H, float* h_x, const float* z) {
    (void)z;
    float q0 = kf.x_[0][0], q1 = kf.x_[1][0], q2 = kf.x_[2][0], q3 = kf.x_[3][0];
    const int n = AHRS_STATE_DIM;

    h_x[0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    h_x[1] = 2.0f*(q1*q2 + q0*q3);
    h_x[2] = 2.0f*(q1*q3 - q0*q2);

    memset(H, 0, 3 * n * sizeof(float));

    H[0*n+0] =  2.0f*q0;  H[0*n+1] =  2.0f*q1;
    H[0*n+2] = -2.0f*q2;  H[0*n+3] = -2.0f*q3;
    H[1*n+0] =  2.0f*q3;  H[1*n+1] =  2.0f*q2;
    H[1*n+2] =  2.0f*q1;  H[1*n+3] =  2.0f*q0;
    H[2*n+0] = -2.0f*q2;  H[2*n+1] =  2.0f*q3;
    H[2*n+2] = -2.0f*q0;  H[2*n+3] =  2.0f*q1;
}

// EKF 更新: 6 维联合观测 (加速度计+磁力计)
void AHRS::update6DCallback(KF& kf, float* H, float* h_x, const float* z) {
    (void)z;
    float q0 = kf.x_[0][0], q1 = kf.x_[1][0], q2 = kf.x_[2][0], q3 = kf.x_[3][0];
    const int n = AHRS_STATE_DIM;

    h_x[0] = 2.0f*(q1*q3 - q0*q2);
    h_x[1] = 2.0f*(q0*q1 + q2*q3);
    h_x[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    h_x[3] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    h_x[4] = 2.0f*(q1*q2 + q0*q3);
    h_x[5] = 2.0f*(q1*q3 - q0*q2);

    memset(H, 0, 6 * n * sizeof(float));

    H[0*n+0] = -2.0f*q2;  H[0*n+1] =  2.0f*q3;  H[0*n+2] = -2.0f*q0;  H[0*n+3] =  2.0f*q1;
    H[1*n+0] =  2.0f*q1;  H[1*n+1] =  2.0f*q0;  H[1*n+2] =  2.0f*q3;  H[1*n+3] =  2.0f*q2;
    H[2*n+0] =  2.0f*q0;  H[2*n+1] = -2.0f*q1;  H[2*n+2] = -2.0f*q2;  H[2*n+3] =  2.0f*q3;
    H[3*n+0] =  2.0f*q0;  H[3*n+1] =  2.0f*q1;  H[3*n+2] = -2.0f*q2;  H[3*n+3] = -2.0f*q3;
    H[4*n+0] =  2.0f*q3;  H[4*n+1] =  2.0f*q2;  H[4*n+2] =  2.0f*q1;  H[4*n+3] =  2.0f*q0;
    H[5*n+0] = -2.0f*q2;  H[5*n+1] =  2.0f*q3;  H[5*n+2] = -2.0f*q0;  H[5*n+3] =  2.0f*q1;
}
#endif

/*============================================
 * 初始化
 ============================================*/

void AHRS::init(const Config& config) {
    BMI088::Config bmi088_config = {
        .spi_acc_config = {
            .hspi = &hspi1,
            .cs_port = GPIOA,
            .cs_pin = GPIO_PIN_4,
        },
        .spi_gyro_config = {
            .hspi = &hspi1,
            .cs_port = GPIOB,
            .cs_pin = GPIO_PIN_0,
        },
        .accel_range = config.accel_range,
        .gyro_range = config.gyro_range,
    };
    bmi088_.init(bmi088_config);

#if AHRS_USE_MAGNETOMETER
    IST8310::Config ist8310_config = {
        .i2c_config = {
            .hi2c = &hi2c3,
            .dev_addr = IST8310_I2C_ADDR << 1,
        },
        .rst_config = {
            .gpio_x = GPIOG,
            .pin = GPIO_PIN_6,
        },
    };
    ist8310_.init(ist8310_config);
#endif

    // 安装角旋转矩阵（顺序：先 roll(X) → 再 pitch(Y) → 再 yaw(Z)；Rz × Ry × Rx）
    float cy = cosf(config.board_yaw * DEG_TO_RAD);
    float sy = sinf(config.board_yaw * DEG_TO_RAD);
    float cp = cosf(config.board_pitch * DEG_TO_RAD);
    float sp = sinf(config.board_pitch * DEG_TO_RAD);
    float cr = cosf(config.board_roll * DEG_TO_RAD);
    float sr = sinf(config.board_roll * DEG_TO_RAD);

    install_rot_[0] = cy*cp;
    install_rot_[1] = -sy*cr + cy*sp*sr;
    install_rot_[2] = sy*sr + cy*sp*cr;
    install_rot_[3] = sy*cp;
    install_rot_[4] = cy*cr + sy*sp*sr;
    install_rot_[5] = -cy*sr + sy*sp*cr;
    install_rot_[6] = -sp;
    install_rot_[7] = cp*sr;
    install_rot_[8] = cp*cr;

    accel_lpf_coef_ = (config.accel_lpf_coef > 0.0f) ? config.accel_lpf_coef : 0.0085f;

    // EKF 噪声配置
    process_noise_quat_ = (config.process_noise_quat > 0.0f) ? config.process_noise_quat : 10.0f;
    process_noise_bias_ = (config.process_noise_bias > 0.0f) ? config.process_noise_bias : 0.001f;
    obs_noise_accel_ = (config.obs_noise_accel > 0.0f) ? config.obs_noise_accel : 1000000.0f;
#if AHRS_USE_MAGNETOMETER
    obs_noise_mag_ = (config.obs_noise_mag > 0.0f) ? config.obs_noise_mag : 1000000.0f;
#endif

    // 预热配置
    preheat_timeout_ms_ = (config.preheat_timeout_ms > 0) ? config.preheat_timeout_ms : 30000;

    // R 矩阵（常量，只算一次）
    r_accel_ = Matrixf<AHRS_OBS_DIM, AHRS_OBS_DIM>::zeros();
    r_accel_[0][0] = obs_noise_accel_;
    r_accel_[1][1] = obs_noise_accel_;
    r_accel_[2][2] = obs_noise_accel_;

    // 卡尔曼滤波器
    const int n = AHRS_STATE_DIM;
    float x_init[AHRS_STATE_DIM];
    memset(x_init, 0, sizeof(x_init));
    x_init[0] = 1.0f;
    float P_init[AHRS_STATE_DIM * AHRS_STATE_DIM];
    memset(P_init, 0, sizeof(P_init));
    for (int i = 0; i < n; i++)
        P_init[i*n + i] = (i < 4) ? 100000.0f : 100.0f;

    KF::Config kf_config = {
        .x_init = x_init,
        .P_init = P_init,
        .lambda = 0.9996f,
    };
    kf_.init(kf_config);
    kf_.setPredictCallback(predictCallback);

    // 温度控制 (TIM10 CH1, PF6)
    extern TIM_HandleTypeDef htim10;
    IMUTemp::Config imu_temp_config = {
        .htim = &htim10,
        .channel = TIM_CHANNEL_1,
    };
    imu_temp_.init(imu_temp_config);

    // 对齐 C 版 AHRS_Register：注册完成后立即预热 + 陀螺仪校准 + 启动 RTOS 任务
    preheat();
    calibrate();
    start();
}

void AHRS::preheat() {
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < preheat_timeout_ms_) {
        bmi088_.readAll();
        imu_temp_.update(bmi088_.temperature);
        if (imu_temp_.isReady()) break;
        HAL_Delay(AHRS_TEMP_CTRL_PERIOD_MS);
    }
    preheat_elapsed_ms_ = HAL_GetTick() - start; // 记录预热实际耗时 [ms]
    // 超时也继续，不阻塞启动
}

void AHRS::calibrate() {
    bmi088_.calibrate();
    initQuaternion();
    DWT_GetDeltaT(&dwt_cnt_);
}

void AHRS::start() {
#if AHRS_RTOS_SUPPORT
    osThreadAttr_t attr = {};
    attr.name = "AHRS";
    attr.stack_size = 4096;
    attr.priority = osPriorityAboveNormal;
    osThreadNew(internalTask, this, &attr);
#endif
}

#if !AHRS_RTOS_SUPPORT
void AHRS::task() {
    uint32_t tick = DWT_GetTimeline_us();
    if (tick - task_last_tick_ < 1000) return;  // 1ms 最小周期
    task_last_tick_ = tick;
    update();
}
#endif

/*============================================
 * 四元数初始化
 ============================================*/

void AHRS::initQuaternion() {
    float acc_sum[3] = {0};
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        bmi088_.readAll();
        acc_sum[0] += bmi088_.accel.x;
        acc_sum[1] += bmi088_.accel.y;
        acc_sum[2] += bmi088_.accel.z;
        DWT_Delay_ms(1);
    }

    float ax = acc_sum[0] / samples;
    float ay = acc_sum[1] / samples;
    float az = acc_sum[2] / samples;

    // 安装角旋转 (S → B)
    float accel_raw[3] = {ax, ay, az};
    float accel_rot[3];
    rotVec(install_rot_, accel_raw, accel_rot);
    ax = accel_rot[0];
    ay = accel_rot[1];
    az = accel_rot[2];

    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm; ay *= norm; az *= norm;

    float gravity[3] = {0, 0, 1};
    float axis[3];
    axis[0] = ay*gravity[2] - az*gravity[1];
    axis[1] = az*gravity[0] - ax*gravity[2];
    axis[2] = ax*gravity[1] - ay*gravity[0];

    float cos_angle = ax*gravity[0] + ay*gravity[1] + az*gravity[2];
    float angle = acosf(fabsf(cos_angle) > 0.9999f ? copysignf(0.9999f, cos_angle) : cos_angle);
    float half_sin = sinf(angle * 0.5f);
    float half_cos = cosf(angle * 0.5f);

    float axis_norm = invSqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);

    if (axis_norm > 1e-6f) {
        axis[0] *= axis_norm;
        axis[1] *= axis_norm;
        axis[2] *= axis_norm;
        kf_.x_[0][0] = half_cos;
        kf_.x_[1][0] = axis[0] * half_sin;
        kf_.x_[2][0] = axis[1] * half_sin;
        kf_.x_[3][0] = axis[2] * half_sin;
    } else {
        kf_.x_[0][0] = 1.0f;
        kf_.x_[1][0] = 0.0f;
        kf_.x_[2][0] = 0.0f;
        kf_.x_[3][0] = 0.0f;
    }

    float q[4] = {kf_.x_[0][0], kf_.x_[1][0], kf_.x_[2][0], kf_.x_[3][0]};
    quatNormalize(q);
    kf_.x_[0][0] = q[0];
    kf_.x_[1][0] = q[1];
    kf_.x_[2][0] = q[2];
    kf_.x_[3][0] = q[3];
}

/*============================================
 * 核心更新
 ============================================*/

void AHRS::update() {
    // 1. 计算时间步长 [s]
    float dt = DWT_GetDeltaT(&dwt_cnt_);
    const int n = AHRS_STATE_DIM;

    // 2. 读取传感器数据
    bmi088_.readAll();
    float gx_raw = bmi088_.gyro.x;
    float gy_raw = bmi088_.gyro.y;
    float gz_raw = bmi088_.gyro.z;
    float ax_raw = bmi088_.accel.x;
    float ay_raw = bmi088_.accel.y;
    float az_raw = bmi088_.accel.z;

    // 3. 安装角旋转 (S → B)
    float gyro_raw[3] = {gx_raw, gy_raw, gz_raw};
    float accel_raw[3] = {ax_raw, ay_raw, az_raw};
    float gyro_rot[3], accel_rot[3];
    rotVec(install_rot_, gyro_raw, gyro_rot);
    rotVec(install_rot_, accel_raw, accel_rot);

    // 4. 减去零偏（零偏也旋转到 body frame）
    float bias_rot[3];
    rotVec(install_rot_, gyro_bias_, bias_rot);
    float gx = gyro_rot[0] - bias_rot[0];
    float gy = gyro_rot[1] - bias_rot[1];
    float gz = gyro_rot[2];
#if AHRS_USE_MAGNETOMETER
    gz -= bias_rot[2];
#endif

    // 存储纠偏后角速度 [rad/s]
    output_.gyro_b[0] = gx;
    output_.gyro_b[1] = gy;
    output_.gyro_b[2] = gz;

    // 5. 更新过程噪声 Q = Q_base * dt
    Matrixf<AHRS_STATE_DIM, AHRS_STATE_DIM> Q = Matrixf<AHRS_STATE_DIM, AHRS_STATE_DIM>::zeros();
    for (int i = 0; i < n; i++) {
        float noise = (i < 4) ? process_noise_quat_ : process_noise_bias_;
        Q[i][i] = noise * dt;
    }

    // 6. EKF 预测
    float u[4] = {dt, gx, gy, gz};
    kf_.predictEKF(Q, u);

    // 7. 加速度低通滤波
    float alpha = dt / (dt + accel_lpf_coef_);
    output_.accel_b[0] = output_.accel_b[0] * (1-alpha) + accel_rot[0] * alpha;
    output_.accel_b[1] = output_.accel_b[1] * (1-alpha) + accel_rot[1] * alpha;
    output_.accel_b[2] = output_.accel_b[2] * (1-alpha) + accel_rot[2] * alpha;

    // 8. 归一化加速度观测
    float norm_acc = invSqrt(output_.accel_b[0]*output_.accel_b[0]
                           + output_.accel_b[1]*output_.accel_b[1]
                           + output_.accel_b[2]*output_.accel_b[2]);
    Matrixf<AHRS_OBS_DIM, 1> z_accel;
    z_accel[0][0] = output_.accel_b[0] * norm_acc;
    z_accel[1][0] = output_.accel_b[1] * norm_acc;
    z_accel[2][0] = output_.accel_b[2] * norm_acc;

#if AHRS_USE_MAGNETOMETER
    // 9. 读取磁力计数据
    IST8310::Data mag_data;
    ist8310_.acquire(mag_data);

    float mag_raw[3] = {mag_data.mag[0], mag_data.mag[1], mag_data.mag[2]};
    float mag_rot[3];
    rotVec(install_rot_, mag_raw, mag_rot);

    output_.mag_b[0] = mag_rot[0];
    output_.mag_b[1] = mag_rot[1];
    output_.mag_b[2] = mag_rot[2];

    // 10. 归一化磁力计观测
    float norm_mag = invSqrt(mag_rot[0]*mag_rot[0] + mag_rot[1]*mag_rot[1] + mag_rot[2]*mag_rot[2]);
    float z_mag[3];
    z_mag[0] = mag_rot[0] * norm_mag;
    z_mag[1] = mag_rot[1] * norm_mag;
    z_mag[2] = mag_rot[2] * norm_mag;

    // 11. 构建 6 维联合观测向量 + 6×6 R 矩阵
    Matrixf<AHRS_OBS_DIM, 1> z_6d;
    z_6d[0][0] = z_accel[0][0];
    z_6d[1][0] = z_accel[1][0];
    z_6d[2][0] = z_accel[2][0];
    z_6d[3][0] = z_mag[0];
    z_6d[4][0] = z_mag[1];
    z_6d[5][0] = z_mag[2];

    Matrixf<AHRS_OBS_DIM, AHRS_OBS_DIM> R_6d = Matrixf<AHRS_OBS_DIM, AHRS_OBS_DIM>::zeros();
    R_6d[0][0] = obs_noise_accel_;
    R_6d[1][1] = obs_noise_accel_;
    R_6d[2][2] = obs_noise_accel_;
    R_6d[3][3] = obs_noise_mag_;
    R_6d[4][4] = obs_noise_mag_;
    R_6d[5][5] = obs_noise_mag_;

    // 13. EKF 更新 - 6 维联合观测
    kf_.setUpdateCallback(update6DCallback);
    kf_.updateEKF(z_6d, R_6d, nullptr);
#else
    // 仅加速度计观测
    kf_.setUpdateCallback(updateAccelCallback);
    kf_.updateEKF(z_accel, r_accel_, nullptr);
#endif

#if AHRS_ENABLE_VALIDATION
    // 协方差边界检查：防止 P 矩阵发散
    for (int i = 0; i < n; i++) {
        if (kf_.P_[i][i] > 100000.0f) {
            kf_.P_[i][i] = 100000.0f;
        }
    }
#endif

    // 12. 归一化四元数
    float q[4] = {kf_.x_[0][0], kf_.x_[1][0], kf_.x_[2][0], kf_.x_[3][0]};
    quatNormalize(q);
    kf_.x_[0][0] = q[0];
    kf_.x_[1][0] = q[1];
    kf_.x_[2][0] = q[2];
    kf_.x_[3][0] = q[3];

    // 13. 更新零偏估计
    gyro_bias_[0] = kf_.x_[4][0];
    gyro_bias_[1] = kf_.x_[5][0];
#if AHRS_USE_MAGNETOMETER
    gyro_bias_[2] = kf_.x_[6][0];
#endif

    // 14. 输出四元数
    memcpy(output_.q, q, 4 * sizeof(float));

    // 15. 四元数转欧拉角
    quatToEuler(output_.q, &output_.euler[0], &output_.euler[1], &output_.euler[2]);

    // 16. 累计偏航角 (处理角度跳变)
    float yaw = output_.euler[2];
    float diff = yaw - yaw_last_;
    if (diff > 180.0f) yaw_round_count_--;
    else if (diff < -180.0f) yaw_round_count_++;
    output_.yaw_total = 360.0f * yaw_round_count_ + yaw;
    yaw_last_ = yaw;

    // 17. 计算运动加速度 [m/s²] (去除重力)
    float q0 = output_.q[0], q1 = output_.q[1];
    float q2 = output_.q[2], q3 = output_.q[3];
    float gravity_b[3] = {
        2.0f*(q1*q3 - q0*q2) * 9.81f,
        2.0f*(q0*q1 + q2*q3) * 9.81f,
        (q0*q0 - q1*q1 - q2*q2 + q3*q3) * 9.81f
    };
    output_.motion_accel_b[0] = accel_rot[0] - gravity_b[0];
    output_.motion_accel_b[1] = accel_rot[1] - gravity_b[1];
    output_.motion_accel_b[2] = accel_rot[2] - gravity_b[2];

    // 18. 转换到世界坐标系
    float R_mat[9] = {
        1.0f - 2.0f*(q2*q2 + q3*q3),  2.0f*(q1*q2 - q0*q3),        2.0f*(q1*q3 + q0*q2),
        2.0f*(q1*q2 + q0*q3),         1.0f - 2.0f*(q1*q1 + q3*q3), 2.0f*(q2*q3 - q0*q1),
        2.0f*(q1*q3 - q0*q2),         2.0f*(q2*q3 + q0*q1),        1.0f - 2.0f*(q1*q1 + q2*q2)
    };
    rotVec(R_mat, output_.motion_accel_b, output_.motion_accel_n);
}

/*============================================
 * RTOS 任务
 ============================================*/

#if AHRS_RTOS_SUPPORT
void AHRS::internalTask(void* arg) {
    AHRS* ahrs = static_cast<AHRS*>(arg);
    uint32_t last_temp_tick = 0;
    for (;;) {
        ahrs->update();
        if (HAL_GetTick() - last_temp_tick >= AHRS_TEMP_CTRL_PERIOD_MS) {
            last_temp_tick = HAL_GetTick();
            ahrs->imu_temp_.update(ahrs->bmi088_.temperature);
        }
        osDelay(1);
    }
}
#endif
