/**
 * @file    bsp_can.h
 * @brief   CAN 总线（C → C++）
 * @note    从 C 版 bsp_can 迁移：struct CAN_Instance → class CAN，CANRegister → init、
 *          CANTransmit → transmit，逻辑不变，禁堆（原 malloc 实例改为栈/全局对象）。
 */
#pragma once

#include <stdint.h>
#include "can.h"

#define CAN_MX_REGISTER_CNT 24     // CAN 实例注册表容量（本工程用 13；溢出后新实例是静默丢弃、很难查，故放宽到 24）
#define MX_CAN_FILTER_CNT (2 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多

#ifdef __cplusplus
extern "C" {
#endif
/** ai-debug：写 1 触发 CAN2 Bus-Off 恢复（Stop→Start），软件自动清 0 */
extern volatile uint8_t can2_busoff_recover_req;
void CAN_PollBusOffRecover(void);
#ifdef __cplusplus
}
#endif

class CAN {
public:
    using Callback = void (*)(void*);

    /// 初始化配置
    struct Config {
        CAN_HandleTypeDef* can_handle;  ///< CAN 句柄
        uint32_t rx_id;                 ///< 接收 ID（0 表示 TX 组）
        uint8_t ext_flag;               ///< 1 = 该实例收**扩展帧**（如 MI 电机），用 32 位掩码滤波器
    };

    void init(const Config& config);    ///< 替代 CANRegister（禁堆）
    uint8_t transmit(float timeout);    ///< 替代 CANTransmit
    void setCallback(Callback callback, void* device);  ///< 设置接收回调

    // —— 跨模块直接读写的状态（对齐 C 版字段）——
    CAN_HandleTypeDef* can_handle_;     ///< CAN 句柄
    CAN_TxHeaderTypeDef tx_conf_;       ///< 发送配置
    uint8_t tx_buff_[8];                ///< 发送缓存
    uint8_t rx_buff_[8];                ///< 接收缓存
    uint8_t rx_len_;                    ///< 接收长度

    static void fifoCallback(CAN_HandleTypeDef* hcan, uint32_t fifox);  ///< FIFO 中断分发

private:
    void addFilter();                   ///< 添加过滤器
    static void serviceInit();          ///< 首次注册时初始化 CAN 硬件

    uint32_t rx_id_;                    ///< 接收 ID
    uint8_t ext_flag_ = 0;              ///< 1 = 扩展帧实例（见 Config::ext_flag）
    Callback callback_ = nullptr;       ///< 接收回调
    void* device_ = nullptr;            ///< 回调 device

    static CAN* instances_[CAN_MX_REGISTER_CNT];    ///< 实例注册表
    static uint8_t idx_;                            ///< 已注册数
};
