/**
 * @file    dji_motor.cpp
 * @brief   DJI 电机实现（C → C++）
 * @note    原 C 版逻辑不变；malloc 实例改为应用层全局对象，发送组 CAN 用静态实例池（禁堆）。
 */
#include "dji_motor.h"
#include "bsp_tim.h"

DJIMotor* DJIMotor::instances_[DJIM_MAX_INSTANCE] = {nullptr};
uint8_t DJIMotor::idx_ = 0;
CAN DJIMotor::tx_group_pool_[DJIM_MAX_GROUP] = {};
uint8_t DJIMotor::group_idx_ = 0;
TIM DJIMotor::tim_;

void DJIMotor::decodeCallback(void* device)
{
    DJIMotor* instance = static_cast<DJIMotor*>(device);

    // 解码CAN数据，大端模式
    uint8_t* CAN_ReceiveData = instance->djim_can_.rx_buff_;
    instance->feedback_raw_.ecd = (uint16_t)(CAN_ReceiveData[0] << 8 | CAN_ReceiveData[1]);
    instance->feedback_raw_.rpm = (int16_t)(CAN_ReceiveData[2] << 8 | CAN_ReceiveData[3]);
    instance->feedback_raw_.curr = (int16_t)(CAN_ReceiveData[4] << 8 | CAN_ReceiveData[5]);
    instance->feedback_raw_.temp = CAN_ReceiveData[6];

    // 速度 → 用户坐标系
    float velocity_user = DJIM_VELOCITY(instance->feedback_raw_.rpm) / instance->reduction_ratio_;
    // 电流 → 用户坐标系（电流不随减速比变化）
    float current_user = DJIM_CURRENT(instance->motor_type_, instance->feedback_raw_.curr);

    if (instance->direction_ == DJIM_DIRECTION_REVERT) {
        velocity_user = -velocity_user;
        current_user = -current_user;
    }

    // 累计角度 → 用户坐标系
    int16_t error = instance->feedback_raw_.ecd - instance->ecd_last_;  // 角度处理，最短路径
    // 角度最短路径处理，防止角度突变超过一半周期
    if (error > DJIM_ENCODER_LINES / 2)
        error -= DJIM_ENCODER_LINES;
    else if (error < -DJIM_ENCODER_LINES / 2)
        error += DJIM_ENCODER_LINES;

    float angle_delta = DJIM_ANGLE(error) / instance->reduction_ratio_;
    if (instance->direction_ == DJIM_DIRECTION_REVERT)
        angle_delta = -angle_delta;

    instance->angle_ += angle_delta;

    instance->velocity_raw_ = velocity_user;   // 原始值（未滤波）
    instance->velocity_ += DJIM_VELOCITY_LPF_ALPHA * (velocity_user - instance->velocity_);   // 一阶低通
    // 电流同样做一阶低通（旧固件 CURRENT_SMOOTH_COEF = 0.9），电流内环的反馈就是它
    instance->current_ += DJIM_CURRENT_LPF_ALPHA * (current_user - instance->current_);
    instance->temperature_ = instance->feedback_raw_.temp;

    // 记录这次的位置给下次使用
    instance->ecd_last_ = instance->feedback_raw_.ecd;

    instance->daemon_lose_.reset();  // 重置失联守护计时器
    instance->motor_valid_ = 1;      // 收到反馈帧，标记电机通信有效
}

void DJIMotor::currentCommand()
{
    int16_t current_cnt = DJIM_CURRENT_TO_CNT(motor_type_, target_current_);

    // 用户坐标系 → 电机坐标系
    if (direction_ == DJIM_DIRECTION_REVERT)
        current_cnt = -current_cnt;

    if (current_cnt > djim_current_cnt_range[motor_type_])
        current_cnt = djim_current_cnt_range[motor_type_];  // 限流保护
    else if (current_cnt < -djim_current_cnt_range[motor_type_])
        current_cnt = -djim_current_cnt_range[motor_type_];

    // 发送电流指令，大端模式
    command_ptr_[0] = (uint8_t)(current_cnt >> 8);
    command_ptr_[1] = (uint8_t)(current_cnt & 0xFF);
}

CAN* DJIMotor::getGroup(DJIMotor* instance)
{
    for (uint8_t i = 0; i < group_idx_; i++)
        if (tx_group_pool_[i].can_handle_ == instance->djim_can_.can_handle_ &&
            tx_group_pool_[i].tx_conf_.StdId == DJIM_TX_ID(DJIM_TX_GROUP(instance->motor_type_, instance->motor_id_)))
            return &tx_group_pool_[i];
    return nullptr;
}

void DJIMotor::update()
{
    if (!motor_enable_ || !motor_valid_)
    {
        target_current_ = 0.0f;
        currentCommand();
        return;
    }

    if (pid_mode_ == PidMode::Position)  // 位置环，分频更新，更新频率=1000Hz/pos_freq_div
    {
        if (++div_cnt_ == pos_freq_div_)
        {
            div_cnt_ = 0;

            pid_angle_.setSetpoint(target_angle_);  // 设定值前馈，角度目标
            pid_angle_.update(angle_);              // 角度控制
            target_velocity_ = pid_angle_.output_;
        }
    }

    if (pid_mode_ != PidMode::Current)  // 速度环，只要不处于开环控制状态（非纯电流控制），速度环都要更新 1000Hz
    {
        pid_velocity_.setSetpoint(target_velocity_ + velocity_feedforward_);  // 设定值前馈，速度前馈
        pid_velocity_.setFeedforward(current_feedforward_);                    // 输出值前馈，力矩前馈

        pid_velocity_.update(velocity_);
        target_current_ = pid_velocity_.output_;
    }

    if (current_loop_enable_)  // 电流内环：目标 = 速度环输出[A]，反馈 = 电调实测电流[A]
    {
        pid_current_.setSetpoint(target_current_);
        pid_current_.update(current_);
        target_current_ = pid_current_.output_;
    }

    currentCommand();
}

void DJIMotor::setEnable(uint8_t motor_enable)
{
    motor_enable = motor_enable != 0;
    motor_enable_ = motor_enable;
}

void DJIMotor::loseCallback(void* device)
{
    DJIMotor* instance = static_cast<DJIMotor*>(device);
    instance->motor_valid_ = 0;  // 失联，标记通信无效（Update 会输出 0 电流）

    // 用于debug检查失联电机的can通道、型号和ID，确认是哪个电机失联了。
    CAN_HandleTypeDef* channel = instance->djim_can_.can_handle_;
    DJIMotor_Type type = instance->motor_type_;
    uint8_t id = instance->motor_id_;

    // 隐藏 unused variable警告
    (void)channel;
    (void)type;
    (void)id;
}

void DJIMotor::timCallback(void* device)
{
    (void)device;
    taskUpdate();
}

/* 调试用：电机环执行次数与相邻两次的 DWT 周期差（168MHz 下 5ms = 840000 cycle），
   调试器读一次即可确认电机环确实跑在 200Hz 且没有长间隔 */
volatile uint32_t g_motor_loop_cnt = 0;
volatile uint32_t g_motor_loop_dcyc = 0;
volatile uint32_t g_motor_loop_maxdcyc = 0;

/* 电机环本体：跑一遍各实例速度环 PID，再按组发一帧 CAN。调用周期见 robot.cpp 的 5 分频 */
void DJIMotor::taskUpdate()
{
    static uint32_t last_cyc = 0;
    uint32_t now_cyc = DWT->CYCCNT;
    if (last_cyc != 0)
    {
        g_motor_loop_dcyc = now_cyc - last_cyc;
        if (g_motor_loop_dcyc > g_motor_loop_maxdcyc)
            g_motor_loop_maxdcyc = g_motor_loop_dcyc;
    }
    last_cyc = now_cyc;

    g_motor_loop_cnt++;
    for (uint8_t i = 0; i < idx_; i++)
        instances_[i]->update();  // 更新所有注册的电机实例的PID

    for (uint8_t i = 0; i < group_idx_; i++)
        tx_group_pool_[i].transmit(0.01f);
}

void DJIMotor::setAngle(float angle)
{
    pid_mode_ = PidMode::Position;
    while (angle - angle_ > 180.0f)
        angle -= 360.0f;
    while (angle - angle_ < -180.0f)
        angle += 360.0f;
    target_angle_ = angle;
}

void DJIMotor::setAngleCircular(float angle)
{
    pid_mode_ = PidMode::Position;
    target_angle_ = angle;
}

void DJIMotor::setAngleIncrement(float angle_increment)
{
    pid_mode_ = PidMode::Position;
    target_angle_ = angle_ + angle_increment;
}

void DJIMotor::setVelocity(float velocity)
{
    pid_mode_ = PidMode::Velocity;
    target_velocity_ = velocity;
}

void DJIMotor::setCurrent(float current)
{
    pid_mode_ = PidMode::Current;
    target_current_ = current;
}

void DJIMotor::setVelocityFF(float velocity)
{
    velocity_feedforward_ = velocity;
}

void DJIMotor::setCurrentFF(float current)
{
    current_feedforward_ = current;
}

// 自己配一个1000Hz的定时器，用于更新PID。硬实时可靠性比RTOS软件定时器更高，且不受其他代码的影响。
// DJI电机控制对实时性要求较高，尤其是位置环，建议使用定时器中断来更新PID。
/* 不再把电机环挂到定时器中断上：框架原本挂 1ms，采样率是旧固件的 5 倍，会让噪声整段进
   PID（电流抖/顶 MaxOut）。改由应用层每 5ms 调 taskUpdate()，见 robot.cpp。 */
void DJIMotor::timbaseSelect(TIM_HandleTypeDef* htim)
{
    (void)htim;
}

void DJIMotor::init(const Config& config)
{
    if (idx_ >= DJIM_MAX_INSTANCE)
        return;

    if (config.motor_id > 8 || config.motor_id == 0)
        return;  // 能进这里家里得请高人了

    Daemon::Config daemon_config = {
        .tim_config = { .htim = nullptr },  // 原 C 未设 tim_config，htim 为 NULL
        .cycle = 100,                        // 守护进程 ms
        .daemon_callback = loseCallback,
        .device = this,
    };

    CAN::Config can_config = {
        .can_handle = config.can_handle,
        .rx_id = static_cast<uint32_t>(DJIM_RX_ID(config.motor_type, config.motor_id)),
    };

    motor_type_ = config.motor_type;
    motor_id_ = config.motor_id;
    direction_ = config.direction;
    reduction_ratio_ = config.reduction_ratio > 0.0f ? config.reduction_ratio : 1.0f;
    angle_offset_ = -config.initial_angle;  // 取反：config 传入初始角度，offset 用于补偿

    float angle_init = config.initial_angle;
    while (angle_init < 0)
        angle_init += 360.0f;
    while (angle_init >= 360.0f)
        angle_init -= 360.0f;
    ecd_last_ = (uint16_t)(angle_init * reduction_ratio_ * DJIM_ENCODER_LINES / 360.0f);  // 初始角度对应的编码器值

    pos_freq_div_ = config.pos_freq_div == 0 ? 1 : config.pos_freq_div;  // 分频不能为0，不能更新了，默认设为3
    if (pos_freq_div_ > 20)  // 分频过大可能导致位置环响应过慢，默认最大设为20
        pos_freq_div_ = 20;
    div_cnt_ = 0;

    pid_angle_.init(config.pid_angle);
    pid_angle_.clearFeatures(PID::FeatureFeedforward);
    // 位置环不启用输出量前馈，位置环更新频率低，为保证速度前馈更新频率，自行实现速度环设定值前馈

    PID::Config vel_config = config.pid_velocity;
    vel_config.feedforward_gain = 1.0f;  // 力矩前馈增益，默认1.0f
    pid_velocity_.init(vel_config);
    pid_velocity_.setFeatures(PID::FeatureFeedforward);

    /* 电流内环（旧固件 close_loop_type 含 CURRENT_LOOP 的电机才有）：速度环输出作为电流
       参考，反馈是电调上报的实测电流，电流 PID 的输出才发给电调。旧底盘/摩擦轮/拨盘都是
       这个两级结构，缺了它等效前向增益会差 2~3 倍（表现为抖动、顶 MaxOut）。 */
    current_loop_enable_ = config.current_loop_enable;
    pid_current_.init(config.pid_current);

    djim_can_.setCallback(decodeCallback, this);
    djim_can_.init(can_config);

    daemon_lose_.init(daemon_config);

    CAN* djim_tx_instance = getGroup(this);
    if (djim_tx_instance == nullptr)
    {
        if (group_idx_ >= DJIM_MAX_GROUP)  // 目前设计最多支持 DJIM_MAX_GROUP 组发送实例
            return;

        djim_tx_instance = &tx_group_pool_[group_idx_];

        CAN::Config can_tx_config = {
            .can_handle = config.can_handle,
            .rx_id = 0,  // 发送不需要设置接收id
        };
        djim_tx_instance->init(can_tx_config);
        djim_tx_instance->tx_conf_.StdId = DJIM_TX_ID(DJIM_TX_GROUP(config.motor_type, config.motor_id));

        group_idx_++;
    }
    command_ptr_ = djim_tx_instance->tx_buff_ + 2 * ((motor_id_ - 1) % 4);  // 每4个电机共用一个发送实例，每个电机占2字节

    instances_[idx_++] = this;
}
