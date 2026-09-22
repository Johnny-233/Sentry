/**
 * @file    daemon.cpp
 * @brief   守护进程 / 设备离线看门狗实现（C → C++）
 * @note    原 C 版 TIM 模式逻辑不变，仅去 malloc、去死注册表、去 RTOS 分支。
 */
#include "daemon.h"

void Daemon::timerCallback(void* arg) {
    Daemon* instance = static_cast<Daemon*>(arg);

    if (instance->online_) {
        if (instance->count_ > 0)
            instance->count_--;
        else {
            if (instance->daemon_callback_ != nullptr)
                instance->daemon_callback_(instance->device_);
            instance->online_ = 0;
        }
    }
}

void Daemon::reset() {
    online_ = 1;
    count_ = cycle_;
}

void Daemon::init(const Config& config) {
    cycle_ = config.cycle;
    daemon_callback_ = config.daemon_callback;
    device_ = config.device;

    // 将 Daemon 实例指针作为定时器回调 device（替代 config 里塞 device/tim_callback）
    tim_.setCallback(timerCallback, this);
    tim_.init(config.tim_config);

    reset();
}
