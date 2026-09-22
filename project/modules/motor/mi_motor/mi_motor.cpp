/**
 * @file    mi_motor.cpp
 * @brief   小米（MI / CyberGear）微电机实现（C → C++）
 * @note    由 Modules/motor/MImotor/mi_motor.c 移植，bsp 换成对方的 CAN 类（project/bsp/bsp_can）：
 *            CANInstance → CAN、CANRegister → CAN::init、CANTransmit → CAN::transmit、
 *            CANAddEXFilter → CAN::Config::ext_flag（扩展帧掩码滤波器 + 通信类型 2 分发）。
 *          协议逻辑逐行照搬原实现（FloatToUint / RangeRestrict / 各通信类型报文格式 / 反馈解包 / 三环级联）。
 *          malloc 实例改为应用层静态对象（禁堆）；
 *          "填进共享发送实例 + MiMotorControl() 周期重发" 改为每个实例一份待发帧
 *          （can_.tx_conf_.ExtId + can_.tx_buff_ + tx_pending_），由 MiMotorControl() 遍历注册表重发。
 */
#include "mi_motor.h"

#include <string.h>

/* 原 static uint8_t idx=0;
 * 原 static MIMotorInstance *mi_motor_instance[MI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组 */
MIMotor* MIMotor::mi_motor_instance_[MI_MOTOR_CNT] = {nullptr};
uint8_t MIMotor::idx_ = 0;

/**
  * @brief          float转int，数据打包用
  * @param[in]      x float数值
  * @param[in]      x_min float数值的最小值
  * @param[in]      x_max float数值的最大值
  * @param[in]      bits  int的数据位数
  * @retval         打包后的整数
  */
static uint32_t FloatToUint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (uint32_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
  * @brief          输入范围限制（原实现的工具函数, 原文件里当前也没有调用点, 原样保留）
  * @param[in]      x 输入数值
  * @param[in]      x_min 输入数值的最小值
  * @param[in]      x_max 输入数值的最大值
  * @retval         限幅后的数值
  */
static float RangeRestrict(float x, float x_min, float x_max)
{
    float res;
    if (x > x_max)
        res = x_max;
    else if (x < x_min)
        res = x_min;
    else
        res = x;
    return res;
}

/**
  * @brief          小米电机反馈帧解码（通信类型2）
  * @param[in]      device 拥有该 CAN 实例的 MIMotor 对象（原 _instance->id）
  * @note           标度/表达式与原实现逐字相同（原式为 double 运算, 这里保持原样不改数值行为）
  * @retval         none
  */
void MIMotor::decodeCallback(void* device)
{
    MIMotor* motor = static_cast<MIMotor*>(device);  // 原: MIMotorInstance *motor = (MIMotorInstance *)_instance->id;
    uint8_t* rxbuff = motor->can_.rx_buff_;          // 原: uint8_t *rxbuff = _instance->rx_buff;
    MI_Motor_Measure_s* measure = &motor->measure_;  // 原: MI_Motor_Measure_s *measure=&motor->measure;
    measure->angle = ((float)(rxbuff[0] << 8 | rxbuff[1]) - 32767.5) / 32767.5 * 4 * 3.1415926f;
    measure->speed = ((float)(rxbuff[2] << 8 | rxbuff[3]) - 32767.5) / 32767.5 * 30.0f;
    measure->torque = ((float)(rxbuff[4] << 8 | rxbuff[5]) - 32767.5) / 32767.5 * 12.0f;
    measure->temperature = (float)(rxbuff[6] << 8 | rxbuff[7]) / 10.0f;

    /* 冻结 API 直接读 angle_/speed_/torque_/temperature_, 这里把原 measure 的 4 个字段同步过去 */
    motor->angle_ = measure->angle;
    motor->speed_ = measure->speed;
    motor->torque_ = measure->torque;
    motor->temperature_ = measure->temperature;

    motor->valid_ = 1; // 收到反馈帧，标记通信有效（原实现无此标志, 移植时新增）

    /* 原 DecodeMiMotor 只解 4 个浮点; 反馈帧里的故障位/模式位在扩展 ID 上, 对方 bsp_can 的
     * fifoCallback 只把 8 字节数据传给回调, 因此 motor_state_/motor_mode_state_ 与原实现一致地保持初值 */
}

/**
  * @brief          扩展 ID 打包：motor_id:8 | data:16 | mode:5 | res:3
  * @param[in]      mode 通信类型
  * @param[in]      data 数据域（16 位）
  * @param[in]      motor_field 低 8 位（原 EXT_ID_t.motor_id 域；主机→电机帧硬编码 127, 通信类型 0 为 0）
  * @retval         29 位扩展 ID（res 域恒为 0）
  * @note           等价于原实现 EXT_ID_t 位域赋值后再 *(uint32_t*)&EXT_ID 强转取整字
  */
uint32_t MIMotor::packExtId(uint8_t mode, uint16_t data, uint8_t motor_field)
{
    return (uint32_t)motor_field | ((uint32_t)data << 8) | ((uint32_t)(mode & 0x1F) << 24);
}

/**
  * @brief          发送缓冲 8 字节清零（原实现里那些 for(i<8) tx_buff[i]=0 的循环）
  * @retval         none
  */
void MIMotor::clearTxBuff()
{
    for (uint8_t i = 0; i < 8; i++)
        can_.tx_buff_[i] = 0;
}

/**
  * @brief          把一帧填进本实例的待发帧（原实现填进共享的 mi_sender_assignment[0]）
  * @param[in]      mode 通信类型
  * @param[in]      data 扩展 ID 的数据域
  * @param[in]      motor_field 扩展 ID 低 8 位（默认宏 MI_EXT_ID_MOTOR_FIELD=127）
  * @retval         none
  * @note           真正的发送在 MiMotorControl() 里按 100Hz 周期做, 保证电机不因收不到帧超时掉使能
  */
void MIMotor::stageFrame(uint8_t mode, uint16_t data, uint8_t motor_field)
{
    can_.tx_conf_.IDE = CAN_ID_EXT; // MI 电机全部使用扩展帧
    can_.tx_conf_.RTR = CAN_RTR_DATA;
    can_.tx_conf_.DLC = 0x08;
    can_.tx_conf_.ExtId = packExtId(mode, data, motor_field);
    tx_pending_ = 1; // 原 sender_enable_flag：有帧待发才发, 避免发送空帧
}

/**
  * @brief          一次性帧：立即发送（不进周期重发缓冲）
  * @param[in]      mode 通信类型
  * @param[in]      data 扩展 ID 的数据域
  * @param[in]      motor_field 扩展 ID 低 8 位
  * @param[in]      timeout 发送超时 ms
  * @retval         none
  * @note           对应原 MI_motor_GetID() 里那次直接的 CANTransmit(&mi_sender_assignment[0], 5)
  */
void MIMotor::sendFrameNow(uint8_t mode, uint16_t data, uint8_t motor_field, float timeout)
{
    if (can_.can_handle_ == nullptr)
        return; // CAN 未注册成功（句柄为空 / 同总线同 ID 重复注册）, 不上总线

    can_.tx_conf_.IDE = CAN_ID_EXT;
    can_.tx_conf_.RTR = CAN_RTR_DATA;
    can_.tx_conf_.DLC = 0x08;
    can_.tx_conf_.ExtId = packExtId(mode, data, motor_field);
    can_.transmit(timeout);
}

/*-------------------- 按照小米电机文档写的各种通信类型 --------------------*/

/**
  * @brief          获取设备ID （通信类型0），需在电机使能前使用
  * @retval         none
  * @note           原 MI_motor_GetID()：EXT_ID.mode=0 / data=0 / motor_id=0 / res=0, tx_buff 全 0,
  *                 然后 CANTransmit(&mi_sender_assignment[0], 5) 立即发送
  */
void MIMotor::getID()
{
    clearTxBuff();
    sendFrameNow(0, 0, 0, 5); // 注意低 8 位是 0（原实现此帧没有用 127）
}

/**
  * @brief          运控模式电机控制指令（通信类型1）
  * @param[in]      torque 目标力矩
  * @param[in]      mech_position 目标机械位置
  * @param[in]      speed 目标速度
  * @param[in]      kp 位置环 kp
  * @param[in]      kd 速度环 kd
  * @retval         none
  * @note           原 MI_motor_Control()：数据区 = 位置|速度|kp|kd 各 16 位大端, 扩展 ID.data = 力矩
  */
void MIMotor::control(float torque, float mech_position, float speed, float kp, float kd)
{
    can_.tx_buff_[0] = FloatToUint(mech_position, P_MIN, P_MAX, 16) >> 8;
    can_.tx_buff_[1] = FloatToUint(mech_position, P_MIN, P_MAX, 16);
    can_.tx_buff_[2] = FloatToUint(speed, V_MIN, V_MAX, 16) >> 8;
    can_.tx_buff_[3] = FloatToUint(speed, V_MIN, V_MAX, 16);
    can_.tx_buff_[4] = FloatToUint(kp, KP_MIN, KP_MAX, 16) >> 8;
    can_.tx_buff_[5] = FloatToUint(kp, KP_MIN, KP_MAX, 16);
    can_.tx_buff_[6] = FloatToUint(kd, KD_MIN, KD_MAX, 16) >> 8;
    can_.tx_buff_[7] = FloatToUint(kd, KD_MIN, KD_MAX, 16);

    // 原: EXT_ID.mode=1, data=FloatToUint(torque,T_MIN,T_MAX,16), motor_id=127, res=0
    stageFrame(1, (uint16_t)FloatToUint(torque, T_MIN, T_MAX, 16));
}

/**
  * @brief          小米电机使能（通信类型 3）
  * @param[in]      en 1=使能；0=停止（原 MIMotorEnable 只有使能帧, 与上一版一致把 0 映射到停止帧）
  * @retval         none
  * @note           原 MIMotorEnable(): EXT_ID.mode=3, data=1, motor_id=127, tx_buff 全 0
  */
void MIMotor::setEnable(uint8_t en)
{
    if (en == 0)
    {
        stop();
        return;
    }

    clearTxBuff();
    stageFrame(3, 1);
    motor_mode_state_ = RUN_MODE; // 原实现未维护该状态, 移植保留上一版行为
}

/**
  * @brief          电机停止运行帧（通信类型4）
  * @retval         none
  * @note           原 MIMotorInstancestop(): EXT_ID.mode=4, motor_id=127, data=1, tx_buff[0]=1
  */
void MIMotor::stop()
{
    clearTxBuff();
    can_.tx_buff_[0] = 1;
    stageFrame(4, 1);
    motor_mode_state_ = RESET_MODE; // 原实现未维护该状态, 移植保留上一版行为
}

/**
  * @brief          设置电机机械零位（通信类型6）会把当前电机位置设为机械零位（掉电丢失）
  * @retval         none
  * @note           原 MIMotorInstanceetMechPositionToZero(): EXT_ID.mode=6, motor_id=127, data=1, tx_buff[0]=1
  */
void MIMotor::setMechPositionToZero()
{
    clearTxBuff();
    can_.tx_buff_[0] = 1;
    stageFrame(6, 1);
}

/**
  * @brief          设置电机CAN_ID（通信类型7）更改当前电机CAN_ID , 立即生效，需在电机使能前使用
  * @param[in]      now_id 电机现在的ID
  * @param[in]      target_id 想要改成的电机ID
  * @retval         none
  * @note           原 MI_motor_ChangeID(): motor->motor_id=Now_ID; EXT_ID.mode=7, motor_id=Now_ID,
  *                 data=Target_ID<<8|1, tx_buff 全 0
  */
void MIMotor::changeID(uint8_t now_id, uint8_t target_id)
{
    motor_id_ = now_id; // 原: motor->motor_id = Now_ID;

    clearTxBuff();
    sendFrameNow(7, (uint16_t)(target_id << 8 | 1), now_id, 1);
}

/**
  * @brief          单个参数读取（通信类型17）
  * @param[in]      index 功能码
  * @retval         none
  * @note           原 MI_motor_ReadParam(): EXT_ID.mode=17, motor_id=127, data=1,
  *                 tx_buff[0..1]=index(小端), tx_buff[2..7]=0
  */
void MIMotor::readParam(uint16_t index)
{
    clearTxBuff();
    memcpy(&can_.tx_buff_[0], &index, 2);
    sendFrameNow(17, 1, MI_EXT_ID_MOTOR_FIELD, 1);
}

/**
  * @brief          小米电机运行模式切换
  * @param[in]      run_mode 更改的模式
  * @note           通信类型18（掉电丢失）；原 MIMotorModeSwitch(): 功能码 0x7005 写在 tx_buff[0..1],
  *                 模式值写在 tx_buff[4]
  * @retval         none
  */
void MIMotor::modeSwitch(uint8_t run_mode)
{
    uint16_t index = 0X7005;

    clearTxBuff();
    memcpy(&can_.tx_buff_[0], &index, 2);
    memcpy(&can_.tx_buff_[4], &run_mode, 1);

    sendFrameNow(18, 1, MI_EXT_ID_MOTOR_FIELD, 1);
}

/**
  * @brief          小米电机控制参数写入
  * @param[in]      index 功能码
  * @param[in]      param 写入的参数
  * @note           通信类型18（掉电丢失）；原 MI_motor_WritePram(): 功能码在 tx_buff[0..1],
  *                 tx_buff[2..3]=0, float 参数在 tx_buff[4..7]
  * @retval         none
  */
void MIMotor::writeParam(uint16_t index, float param)
{
    memcpy(&can_.tx_buff_[0], &index, 2);
    can_.tx_buff_[2] = 0;
    can_.tx_buff_[3] = 0;
    memcpy(&can_.tx_buff_[4], &param, 4);

    sendFrameNow(18, 1, MI_EXT_ID_MOTOR_FIELD, 1);
}

/**
  * @brief          小米电机控制参数批量写入（原 MIMotorSetPid）
  * @note           功能码/顺序与原实现一致：0x7017 限速, 0x701E 位置 kp, 0x701F 速度 kp, 0x7020 速度 ki
  * @retval         none
  */
void MIMotor::setPid(float location_kp, float limit_speed, float speed_kp, float speed_ki)
{
    writeParam(0x7017, limit_speed);
    writeParam(0x701E, location_kp);
    writeParam(0x701F, speed_kp);
    writeParam(0x7020, speed_ki);
}

/**
  * @brief          写位置模式角度指令（原 MiMotorSetRef）
  * @param[in]      location_ref 位置指令
  * @retval         none
  */
void MIMotor::setRef(float location_ref)
{
    // motor->motor_controller.pid_ref = location_ref;  // 原实现里这行是注释掉的, 原样保留

    writeParam(0x7016, location_ref);
}

/**
  * @brief          原 MIMotorInit 里从 Motor_Init_Config_s 取的控制器设置/PID/反馈指针
  * @param[in]      setting 原 controller_setting_init_config（正反转, 闭环类型等）
  * @param[in]      controller_init 原 controller_param_init_config（三环 PID + 反馈/前馈指针）
  * @retval         none
  * @note           冻结的 MIMotor::Config 里装不下这些字段, 单独给一个入口; 不调用则保持默认（开环）
  */
void MIMotor::setControllerConfig(const Motor_Control_Setting_s& setting, const Motor_Controller_Init_s& controller_init)
{
    motor_settings_ = setting; // 原: motor->motor_settings = config->controller_setting_init_config;

    motor_controller_.other_angle_feedback_ptr = controller_init.other_angle_feedback_ptr;
    motor_controller_.other_speed_feedback_ptr = controller_init.other_speed_feedback_ptr;
    motor_controller_.current_feedforward_ptr = controller_init.current_feedforward_ptr;
    motor_controller_.speed_feedforward_ptr = controller_init.speed_feedforward_ptr;

    // 原实现只 PIDInit 了角度环与速度环（current_PID 没初始化）, 这里保持一致
    motor_controller_.angle_PID.init(controller_init.angle_PID);
    motor_controller_.speed_PID.init(controller_init.speed_PID);
}

/*-------------------- 封装的一些控制函数 --------------------*/

/**
  * @brief          小米电机力矩控制模式控制指令
  * @param[in]      torque 目标力矩
  * @retval         none
  * @note           原 MI_motor_TorqueControl(): MI_motor_Control(motor, torque, 0, 0, 0, 0)
  */
void MIMotor::torqueControl(float torque)
{
    control(torque, 0, 0, 0, 0);
}

/**
  * @brief          小米电机“位置”模式控制指令
  * @param[in]      location 控制位置 rad
  * @param[in]      kp 响应速度(到达位置快慢)，一般取1-10
  * @param[in]      kd 电机阻尼，过小会震荡，过大电机会震动明显。一般取0.5左右
  * @retval         none
  * @note           原 MI_motor_LocationControl(): MI_motor_Control(motor, 0, location, 0, kp, kd)
  */
void MIMotor::locationControl(float location, float kp, float kd)
{
    control(0, location, 0, kp, kd);
}

/**
  * @brief          小米电机速度模式控制指令
  * @param[in]      speed 控制速度
  * @param[in]      kd 响应速度，一般取0.1-1
  * @retval         none
  * @note           原 MIMotorInstancepeedControl(): MI_motor_Control(motor, 0, 0, speed, 0, kd)
  */
void MIMotor::speedControl(float speed, float kd)
{
    control(0, 0, speed, 0, kd);
}

/**
  * @brief          小米电机初始化
  * @param[in]      config 初始化配置
  * @retval         none
  * @note           原 MIMotorInit() 用 malloc 建实例并登记进静态数组; 本版实例由应用层静态持有（禁堆）。
  *                 CAN 实例注册成扩展帧实例（ext_flag=1）：滤波器 + 通信类型 2 反馈分发都靠它。
  *                 注意：对方 bsp_can 的 fifoCallback 对扩展帧是"同一条总线上第一个 ext 实例收全部
  *                 类型 2 帧"（它自己也标了 TODO：按 ExtId 低 8 位再分辨）, 所以**一条总线只挂一台
  *                 MI 电机**时反馈才是对的（原实现同样是单发送实例 + 单扩展接收）。本工程 pitch 只有一台。
  */
void MIMotor::init(const Config& config)
{
    if (config.can_handle == nullptr)
        return;

    motor_id_ = config.motor_id;
    motor_state_ = OK;
    motor_mode_state_ = RESET_MODE;

    /* 原: config->can_init_config.can_module_callback = DecodeMiMotor;
     *     config->can_init_config.id = motor;
     *     motor->motor_can_instace = CANRegister(&config->can_init_config);
     *     motor->motor_can_instace->ext_flag = config->can_init_config.ext_flag; */
    CAN::Config can_config = {
        .can_handle = config.can_handle,
        .rx_id = config.motor_id,
        .ext_flag = 1, // MI 电机反馈是**扩展帧**：32 位掩码滤波器 + fifoCallback 的 EXT 分发
    };

    can_.can_handle_ = nullptr; // CAN::init 遇到重复注册会提前 return, 先清空以便判断注册结果
    can_.setCallback(decodeCallback, this);
    can_.init(can_config);

    if (can_.can_handle_ != nullptr)
    {
        // 原 mi_sender_assignment[0]={.txconf.IDE=CAN_ID_EXT, .txconf.RTR=CAN_RTR_DATA, .txconf.DLC=0x08}
        can_.tx_conf_.IDE = CAN_ID_EXT; // CAN::init 会把 IDE 设回 CAN_ID_STD, 这里改回扩展帧
        can_.tx_conf_.RTR = CAN_RTR_DATA;
        can_.tx_conf_.DLC = 0x08;
    }

    /* 原: mi_motor_instance[idx++] = motor;（MiMotorControl() 遍历它做周期重发） */
    for (uint8_t i = 0; i < idx_; i++)
        if (mi_motor_instance_[i] == this)
            return; // 同一对象重复 init 不重复登记（原实现每次 malloc 新实例, 不会重复）
    if (idx_ < MI_MOTOR_CNT)
        mi_motor_instance_[idx_++] = this;
}

/**
  * @brief          原 MiMotorControl()：把待发帧周期性重发出去
  * @retval         none
  * @note           原实现只有一行 CANTransmit(&mi_sender_assignment[0], 1)（发送缓冲是共享的）;
  *                 本移植遍历注册表, 按各实例当前的待发帧分别重发（超时同样取 1ms）。
  *                 在 1kHz 的 MOTOR 任务里按 cnt%10==0 调用 = 原来的 100Hz。
  */
void MiMotorControl(void)
{
    for (uint8_t i = 0; i < MIMotor::idx_; i++)
    {
        MIMotor* motor = MIMotor::mi_motor_instance_[i];
        if (motor == nullptr || motor->tx_pending_ == 0 || motor->can_.can_handle_ == nullptr)
            continue;
        motor->can_.transmit(1); // 原: CANTransmit(&mi_sender_assignment[0], 1);
    }
}

/**
  * @brief          原 CalMiMotorTorque()：三环级联计算（位置环 → 速度环 → 电流前馈/反馈反向）
  * @retval         最终输出（电机控制量）
  * @note           逐行照搬原实现, 只把 PIDCalculate(&pid, measure, ref) 换成对方 PID 类的
  *                 setSetpoint(ref) + update(measure)（输出取 output_, 语义等价）;
  *                 原实现直接解引用 mi_motor_instance[0]（没有实例时会硬故障）, 这里加了空指针保护。
  */
float CalMiMotorTorque(void)
{
    MIMotor* motor = MIMotor::mi_motor_instance_[0];
    if (motor == nullptr)
        return 0.0f;

    Motor_Control_Setting_s* motor_setting = &motor->motor_settings_; // 电机控制参数
    Motor_Controller_s* motor_controller = &motor->motor_controller_; // 电机控制器
    float pid_measure = 0.0f, pid_ref, set; // 电机PID测量值和设定值
    // 原实现里 pid_measure 在 MOTOR_FEED 时未被赋值（未初始化即使用, UB）, 这里初始化为 0

    pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改

    if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        pid_ref *= -1; // 设置反转

    // pid_ref会顺次通过被启用的闭环充当数据的载体
    // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
    if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
    {
        if (motor_setting->angle_feedback_source == OTHER_FEED)
            pid_measure = *motor_controller->other_angle_feedback_ptr;
        // 更新pid_ref进入下一个环
        motor_controller->angle_PID.setSetpoint(pid_ref);
        motor_controller->angle_PID.update(pid_measure);
        pid_ref = motor_controller->angle_PID.output_;
    }

    // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
    if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
            pid_ref += *motor_controller->speed_feedforward_ptr;

        if (motor_setting->speed_feedback_source == OTHER_FEED)
            pid_measure = *motor_controller->other_speed_feedback_ptr;
        // 更新pid_ref进入下一个环
        motor_controller->speed_PID.setSetpoint(pid_ref);
        motor_controller->speed_PID.update(pid_measure);
        pid_ref = motor_controller->speed_PID.output_;
    }

    // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
    if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
        pid_ref += *motor_controller->current_feedforward_ptr;

    if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
        pid_ref *= -1;

    // 获取最终输出
    set = (float)pid_ref;
    return set;
}
