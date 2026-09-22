/**
 * @file    remote.cpp
 * @brief   遥控模块门面实现：校验机型/串口参数后转调对应驱动
 */
#include "remote.h"

#include "bsp_log.h"

#if defined(REMOTE_DEVICE_VT13)
#include "vt13.h"
#elif defined(REMOTE_DEVICE_DT7)
#include "dt7.h"
#elif defined(REMOTE_DEVICE_HT10A)
#include "ht10a.h"
#endif

Remote& Remote::instance()
{
    static Remote remote;
    return remote;
}

bool Remote::uartParamsMatch(UART_HandleTypeDef* huart) const
{
    if (huart == nullptr) return false;
    return huart->Init.BaudRate == REMOTE_UART_BAUD &&
           huart->Init.WordLength == REMOTE_UART_WORDLENGTH &&
           huart->Init.Parity == REMOTE_UART_PARITY;
}

void Remote::init(const Config& config)
{
#if defined(REMOTE_DEVICE_VT13)
    if (config.device != Device::Vt13) {
        LOG_ERR(LOG_MOD_COMM, "Remote",
                "Config.device mismatch: compiled VT13, got %u",
                static_cast<unsigned>(config.device));
        return;
    }
#elif defined(REMOTE_DEVICE_DT7)
    if (config.device != Device::Dt7) {
        LOG_ERR(LOG_MOD_COMM, "Remote",
                "Config.device mismatch: compiled DT7, got %u",
                static_cast<unsigned>(config.device));
        return;
    }
#elif defined(REMOTE_DEVICE_HT10A)
    if (config.device != Device::Ht10a) {
        LOG_ERR(LOG_MOD_COMM, "Remote",
                "Config.device mismatch: compiled HT10A, got %u",
                static_cast<unsigned>(config.device));
        return;
    }
#endif

    if (!uartParamsMatch(config.usart_handle)) {
        LOG_ERR(LOG_MOD_COMM, "Remote",
                "UART params mismatch: got baud=%lu wl=0x%lx parity=0x%lx, "
                "expect baud=%lu wl=0x%lx parity=0x%lx (fix CubeMX)",
                (unsigned long)(config.usart_handle
                                    ? config.usart_handle->Init.BaudRate
                                    : 0u),
                (unsigned long)(config.usart_handle
                                    ? config.usart_handle->Init.WordLength
                                    : 0u),
                (unsigned long)(config.usart_handle
                                    ? config.usart_handle->Init.Parity
                                    : 0u),
                (unsigned long)REMOTE_UART_BAUD,
                (unsigned long)REMOTE_UART_WORDLENGTH,
                (unsigned long)REMOTE_UART_PARITY);
        return;
    }

#if defined(REMOTE_DEVICE_VT13)
    remote_vt13::init(config.usart_handle);
#elif defined(REMOTE_DEVICE_DT7)
    remote_dt7::init(config.usart_handle);
#elif defined(REMOTE_DEVICE_HT10A)
    remote_ht10a::init(config.usart_handle);
#endif
}

uint8_t Remote::online() const
{
#if defined(REMOTE_DEVICE_VT13)
    return remote_vt13::online();
#elif defined(REMOTE_DEVICE_DT7)
    return remote_dt7::online();
#elif defined(REMOTE_DEVICE_HT10A)
    return remote_ht10a::online();
#else
    return 0;
#endif
}

/* ========== 兼容层 ========== */

#if defined(REMOTE_DEVICE_VT13)

const remote_frame_t* Remote_Init(UART_HandleTypeDef* huart)
{
    Remote::instance().init({
        .device = Remote::Device::Vt13,
        .usart_handle = huart,
    });
    return remote_vt13::data;
}

#elif defined(REMOTE_DEVICE_DT7)

const dt7_rc_t* Remote_Init(UART_HandleTypeDef* huart)
{
    Remote::instance().init({
        .device = Remote::Device::Dt7,
        .usart_handle = huart,
    });
    return remote_dt7::data;
}

#elif defined(REMOTE_DEVICE_HT10A)

const ht10a_rc_t* Remote_Init(UART_HandleTypeDef* huart)
{
    Remote::instance().init({
        .device = Remote::Device::Ht10a,
        .usart_handle = huart,
    });
    return remote_ht10a::data;
}

#endif

uint8_t Remote_Online()
{
    return Remote::instance().online();
}

void Remote_Recover()
{
#if defined(REMOTE_DEVICE_VT13)
    remote_vt13::restart();
#elif defined(REMOTE_DEVICE_DT7)
    remote_dt7::restart();
#elif defined(REMOTE_DEVICE_HT10A)
    remote_ht10a::restart();
#endif
}
