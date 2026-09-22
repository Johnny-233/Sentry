/**
 * @file    super_cap.cpp
 * @brief   超级电容实现（C → C++）
 * @note    原 C 版逻辑不变，禁堆：SuperCapInit 的 malloc 改为应用层全局对象。
 *          接收固定用标准帧（rx_id 由 Config 配置），发送 id 由 Config 配置。
 */
#include "super_cap.h"

/**
  * @brief          超级电容反馈帧解析
  * @param[in]      device 拥有该 CAN 实例的 SuperCap 对象
  * @note           6 字节大端：vol | current | power。旧实现只解了 vol（且接收长度配置成 4 字节时
  *                 仍固定读 6 字节），这里按协议把三个量都解出来，并加长度保护。
  */
void SuperCap::rxCallback(void* device)
{
    SuperCap* cap = static_cast<SuperCap*>(device);
    uint8_t* rxbuff = cap->can_.rx_buff_;

    if (cap->can_.rx_len_ < 6)
        return; // 帧长不足，保持上一次的值

    cap->cap_msg_.vol = (uint16_t)(rxbuff[0] << 8 | rxbuff[1]);
    cap->cap_msg_.current = (uint16_t)(rxbuff[2] << 8 | rxbuff[3]);
    cap->cap_msg_.power = (uint16_t)(rxbuff[4] << 8 | rxbuff[5]);
}

/**
  * @brief          初始化超级电容
  * @param[in]      config 初始化配置
  * @retval         none
  */
void SuperCap::init(const Config& config)
{
    if (config.can_handle == nullptr)
        return;

    CAN::Config can_config = {
        .can_handle = config.can_handle,
        .rx_id = config.rx_id,
    };

    can_.can_handle_ = nullptr; // CAN::init 重复注册时会提前返回，这里先清空以便判断注册结果
    can_.setCallback(rxCallback, this);
    can_.init(can_config);

    if (can_.can_handle_ == nullptr)
        return; // 重复注册（同一总线同一 rx_id）或实例已满

    can_.tx_conf_.StdId = config.tx_id; // CAN::init 只配了 IDE/RTR/DLC，发送 id 需要自己填
}

/**
  * @brief          发送超级电容控制信息（功率限制指令）
  * @param[in]      data 发送数据
  * @param[in]      len 有效数据长度（字节，最大 8）
  * @retval         none
  * @note           旧实现 memcpy(tx_buff, data, 8) 会越界读 8 字节且恒以 DLC=8 发送；
  *                 这里按 len 拷贝（其余补 0）并用 len 设定 DLC，避免越界读。
  */
void SuperCap::send(uint8_t* data, uint16_t len)
{
    if (can_.can_handle_ == nullptr || data == nullptr || len == 0)
        return;

    if (len > 8)
        len = 8;

    for (uint8_t i = 0; i < 8; i++)
        can_.tx_buff_[i] = 0;

    for (uint16_t i = 0; i < len; i++)
        can_.tx_buff_[i] = data[i];

    can_.tx_conf_.DLC = (uint8_t)len;
    can_.transmit(1.0f);
}
