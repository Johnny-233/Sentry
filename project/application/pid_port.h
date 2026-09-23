#ifndef PID_PORT_H
#define PID_PORT_H

/**
 * @file    pid_port.h
 * @brief   旧 C `Modules/algorithm/controller.c` 的 PID 配置 → 对方 `PID` 类配置的换算
 *
 * 为什么要换算：两套 PID 的**输入/输出量纲**和**积分时间单位**都不一样，直接照抄数字
 * 会让增益差几个数量级。三处差异（都已在源码里核对）：
 *
 *   1) 输出量纲
 *      旧: `PIDCalculate()` 的 Output 直接当 **CAN 电流计数** 发出去
 *          （`Modules/motor/DJImotor/dji_motor.c`: `set = (int16_t)pid_ref;` 直接填报文）
 *      新: `PID::output_` 喂给 `DJIMotor` 的是 **安培**，由 `DJIM_CURRENT_TO_CNT()` 再换算成计数
 *          （`DJIMotor::currentCommand()`；3508 计数范围 16384 对应 20A，2006 是 10000/10A，6020 是 25000/1.0）
 *      → 输出量纲系数 S = 电流量程 / 计数范围
 *        应用层自闭环的 PID（例如云台 yaw 的角度/速度环）输出仍是"计数"，取 S = 1。
 *
 *   2) 积分时间单位
 *      旧: `pid->dt = DWT_GetDeltaT()` 单位**秒**，`ITerm = Ki*Err*dt`
 *      新: `integral_ += error * period`，`period` 单位**毫秒**
 *      → ki' = Ki*S/1000，kd' = Kd*S*1000（微分是 /dt）
 *
 *   3) 输出限幅与死区
 *      旧: `f_Output_Limit()` 在 `PIDCalculate()` 末尾**无条件**调用；死区是 `abs(Err) > DeadBand`
 *      新: 都要靠 features 打开（`FeatureOutputLimit` / `FeatureDeadZone`），且 output_min/max 必须自己写对称
 *
 * 积分限幅的换算（旧 IntegralLimit 限的是积分**输出**，新限的是积分**状态**）：
 *   旧 Iout_max = Ki * Σ(Err*dt_s)_max = IntegralLimit
 *   → 新 integral_max = Σ(Err*period_ms)_max = 1000 * IntegralLimit / Ki
 *   （注意这个值与 S 无关，因为两边都是"积分输出"对齐）
 *
 * 用法（保留旧 C 的数字，一字不改，只把量纲/单位交给这里换算）：
 *   static const PidPort kChassisSpeed = {
 *       .Kp = 10, .Ki = 0, .Kd = 0,
 *       .IntegralLimit = 3000,
 *       .MaxOut = 12000,
 *       .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
 *   };
 *   ... .pid_velocity = pidPort(kChassisSpeed, PID_SCALE_M3508, 1),
 *
 * 注意（前馈）：旧 C 的前馈是 `Output += FF_Gain*(Measure - Last_Measure)`（反馈差分，自动、无 dt）。
 *   新 PID 只有在 features 带 `FeatureFeedforward` **且**每周期调用 `setFeedforward(本次反馈-上次反馈)`
 *   时才会加 `feedforward * feedforward_gain`。所以 `FF_Gain != 0` 的 PID 由调用方负责喂差分
 *   （本适配层只负责打开 feature 和填 `feedforward_gain`），`FF_Gain == 0` 的不打开。
 */

#include "pid.h"
#include <stdint.h>

/* ---- 旧 controller.h 的 Improve 位（名字保持原样，方便照抄旧配置） ---- */
#define PID_Trapezoid_Intergral        (1u << 0)
#define PID_Integral_Limit             (1u << 1)
#define PID_Derivative_On_Measurement  (1u << 2)
#define PID_OutputFilter               (1u << 3)
#define PID_ChangingIntegrationRate    (1u << 4)
#define PID_DerivativeFilter           (1u << 5)
#define PID_ErrorHandle                (1u << 6)

/* ---- 输出量纲系数 S = 电流量程(A) / 计数范围 ---- */
#define PID_SCALE_M3508 (20.0f / 16384.0f) // 3508: 16384 计数 = 20A
#define PID_SCALE_M2006 (10.0f / 10000.0f) // 2006: 10000 计数 = 10A
#define PID_SCALE_GM6020 (1.0f / 25000.0f) // 6020 电压控制: 25000 计数 = 1.0
#define PID_SCALE_COUNTS 1.0f              // 输出本身就是计数(应用层自闭环)时用这个

/* ---- 内环电流 PID（旧 current_PID）-----------------------------------------
 * 旧底盘/摩擦轮/拨盘是"速度环 → 电流环"串级；本框架内环的参考与反馈都在**安培**域，
 * 所以用 pidPort(旧配置, 1.0f, period_ms)：Kp/Ki 数值与旧版一致，只有 MaxOut 要从
 * ESC 计数换算成安培（如 15000*PID_SCALE_M3508 = 18.31A），直接在 PidPort 里写。 */

/** 旧 C PID 的配置（字段名与 Motor_Init_Config_s 里的一致，便于照抄） */
struct PidPort {
    float Kp;
    float Ki;
    float Kd;
    float IntegralLimit;
    float MaxOut;
    uint16_t Improve;
    float DeadBand;   // 旧默认 0
    float FF_Gain;    // 旧默认 0
    float Derivative_LPF_RC; // 旧默认 0（只在 PID_DerivativeFilter 打开时有意义）
    float Output_LPF_RC;     // 旧默认 0（只在 PID_OutputFilter 打开时有意义）
};

/** 旧配置 → 对方 PID::Config。output_scale 取 PID_SCALE_*，period_ms 是这个 PID 实际被调用的周期。 */
static inline PID::Config pidPort(const PidPort &o, float output_scale, uint8_t period_ms)
{
    PID::Config c = {};

    c.kp = o.Kp * output_scale;

    /* 单位：旧 Ki 配的是"秒"，新 integral 累加的是 error*period(ms) */
    c.ki = o.Ki * output_scale / 1000.0f;
    /* 微分：旧 Kd 配的是 /dt(s)，新是 /period(ms) */
    c.kd = o.Kd * output_scale * 1000.0f;

    /* 积分状态限幅：见文件头推导 integral_max = 1000*IntegralLimit/Ki */
    c.integral_limit = (o.Ki != 0.0f) ? (1000.0f * o.IntegralLimit / o.Ki) : 0.0f;

    /* 旧版无条件按 MaxOut 限幅 → 新必须打开 FeatureOutputLimit 且对称写 min/max */
    c.output_min = -o.MaxOut * output_scale;
    c.output_max = o.MaxOut * output_scale;

    c.mode = PID::Mode::Position; // 旧 controller.c 只有位置式
    c.period = period_ms;

    uint16_t f = PID::FeatureOutputLimit;

    if (o.Improve & PID_Trapezoid_Intergral)
        f |= PID::FeatureTrapezoidIntegral;
    if (o.Improve & PID_Integral_Limit)
        f |= PID::FeatureIntegralLimit;
    if (o.Improve & PID_Derivative_On_Measurement)
        f |= PID::FeatureDerivativeOnMeasurement;
    if (o.Improve & PID_DerivativeFilter)
        f |= PID::FeatureDerivativeLimit; // 近似：对方没有微分滤波，只有微分限幅
    if (o.Improve & PID_OutputFilter)
        f |= PID::FeatureFilter;
    if (o.Improve & PID_ChangingIntegrationRate)
        f |= PID::FeatureVariableGain; // 近似：对方的变增益是线性增益，与旧"变速积分"不同

    if (o.DeadBand > 0.0f)
        f |= PID::FeatureDeadZone;
    c.dead_zone = o.DeadBand;

    if (o.FF_Gain != 0.0f)
        f |= PID::FeatureFeedforward;
    c.feedforward_gain = o.FF_Gain;

    c.features = f;
    return c;
}

#endif // PID_PORT_H
