/**
 * @file    master_process.cpp
 * @brief   视觉/小电脑通信（由 Modules/master_machine/master_process.c 移植）
 * @note    协议（seasky）逐字节照搬，只用对方的 bsp：
 *            - 串口: 对方的 `Serial`（内部 USART + full_daemon_ 整帧超时）
 *            - 离线: 对方的 `Daemon`（周期挂在 htim5 的 1ms 时基上，本工程由 1kHz MOTOR 任务合成 tick）
 *          姿态来源由应用层传入（原来直接读 QEKF_INS）。
 */
#include "master_process.h"
#include "seasky_protocol.h"

#include "serial.h"
#include "daemon.h"
#include "bsp_log.h"
#include "tim.h"

static Minipc_Recv_s minipc_recv_data;
static Minipc_Send_s minipc_send_data;
static Daemon minipc_daemon;
static Serial minipc_serial;
static AHRS *minipc_ahrs = nullptr; // 姿态来源(应用层传入的全局 AHRS 实例)

/* 视觉帧计数: 调试采样用(每次成功解出一帧 +1), 不影响控制逻辑 */
volatile uint32_t g_vision_frame_cnt = 0;

void VisionSetFlag(uint8_t color)
{
    minipc_send_data.Vision.detect_color = color;
}

void VisionSetAltitude()
{
    if (minipc_ahrs == nullptr)
        return;
    minipc_send_data.Vision.pitch = minipc_ahrs->output_.euler[1];
    minipc_send_data.Vision.roll = minipc_ahrs->output_.euler[0];
    minipc_send_data.Vision.yaw = minipc_ahrs->output_.euler[2];
}

/**
 * @brief 离线回调函数
 * @attention 原实现里这里调 USARTServiceInit() 重启串口接收, 用于绕开 HAL "DMA 收 + 发" 的
 *            __HAL_LOCK 死锁(发过一次就再也进不了接收中断)。
 *            对方的 Serial 已经用 full_daemon_ + USART::errorCallback 处理重开接收
 *            (见 project/modules/serial/serial.cpp、project/bsp/bsp_usart/bsp_usart.cpp),
 *            这里只做记录。
 *            TODO(移植): 若实测仍出现"收不到", 需要给 Serial/USART 加一个公开的重启接口。
 */
static void VisionOfflineCallback(void *id)
{
    (void)id;
    /* 与遥控同理: 掉线时真正重启一次串口接收(旧 C 版就是在这里调 USARTServiceInit) */
    minipc_serial.restart();
    LOG_ERR(LOG_MOD_COMM, "vision", "[vision] vision offline, restart communication.\r\n");
}

/**
 * @brief 接收解包回调函数, 由对方的 Serial 在整帧到达/空闲时调用
 */
static void DecodeMinpc(uint16_t len)
{
    uint16_t flag_register;
    minipc_daemon.reset(); // 喂狗
    get_protocol_info_vision(minipc_serial.recv_buf_, len, &flag_register, &minipc_recv_data);
    g_vision_frame_cnt++; // 调试用计数: 供云台跟踪采样判断新帧
}

Minipc_Recv_s *Minipc_Init(UART_HandleTypeDef *handle, AHRS *ahrs)
{
    minipc_ahrs = ahrs;

    /* 为 master process 注册 daemon: 判断视觉通信是否离线(原 reload_count = 10) */
    Daemon::Config daemon_config = {
        .tim_config = {.htim = &htim5},
        .cycle = 10,
        .daemon_callback = VisionOfflineCallback,
        .device = nullptr,
    };
    minipc_daemon.init(daemon_config);

    /* 接收缓冲在原实现里是 Minipc_Recv_sIZE(36) 字节; 对方的 Serial 固定 256 字节, 够用 */
    Serial::Config serial_config = {
        .usart_handle = handle,
        .htim = &htim5,
        .rx_callback = DecodeMinpc,
    };
    minipc_serial.init(serial_config);

    return &minipc_recv_data;
}

Minipc_Recv_s *Minipc_GetData(void)
{
    return &minipc_recv_data;
}

/* 模块持有唯一那份发送数据: 应用层(robot_cmd)只填敌我颜色这类它负责的字段,
   姿态由 SendMinipcData() 统一填。1kHz(INS 任务)与 200Hz(RobotCMDTask)两条路径共用,
   这样 1kHz 那帧也带正确的敌我颜色(原实现里那份从没被填过, 一直发 0=红)。 */
Minipc_Send_s *Minipc_GetSendData(void)
{
    return &minipc_send_data;
}

uint8_t Minipc_Online(void)
{
    return minipc_daemon.online_;
}

void Minipc_Recover(void)
{
    minipc_serial.restart();
}

/**
 * @brief 发送函数
 *
 * @param send_data 待发送数据(传 NULL 用模块内部那份静态副本, 兼容 1kHz INS 路径)
 */
void SendMinipcData(Minipc_Send_s *send_data)
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint16_t flag_register;
    static uint8_t send_buff[Minipc_Send_sIZE];
    static uint16_t tx_len;

    // 若调用方未提供数据, 使用内部静态副本 (兼容1kHz INS路径)
    if (send_data == NULL)
    {
        send_data = &minipc_send_data;
    }

    // 填充IMU姿态数据
    if (minipc_ahrs != nullptr)
    {
        send_data->Vision.roll = minipc_ahrs->output_.euler[0];
        send_data->Vision.pitch = minipc_ahrs->output_.euler[1];
        send_data->Vision.yaw = minipc_ahrs->output_.euler[2];
    }

    // 将数据转化为seasky协议的数据包
    flag_register = 30 << 8 | 0b00000001;
    get_protocol_send_Vision_data(0x02, flag_register, send_data, 1, send_buff, &tx_len);
    minipc_serial.send(send_buff, tx_len);
}
