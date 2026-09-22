/**
 * @file    remote.h
 * @brief   遥控模块门面：class Remote + 编译期路由到 VT13/DT7/HT10A
 * @note    机型由 remote_config.h 宏选定；应用层仍可用 Remote_Init / REMOTE_* 宏。
 */
#pragma once

#include "remote_config.h"
#include "bsp_usart.h"
#include "bsp_tim.h"
#include <stdint.h>

class Remote {
public:
    enum class Device : uint8_t {
        Vt13 = 0,
        Dt7,
        Ht10a,
    };

    struct Config {
        Device device;
        UART_HandleTypeDef* usart_handle;
    };

    void init(const Config& config);
    uint8_t online() const;

    static Remote& instance();

private:
    Remote() = default;
    bool uartParamsMatch(UART_HandleTypeDef* huart) const;
};

/* ========== 编译期路由：机型数据与访问宏 ========== */

#if defined(REMOTE_DEVICE_VT13)

#include "vt13.h"

#define REMOTE_FRAME_LEN REMOTE_VT13_FRAME_LEN

#define remote_data_flag remote_vt13::data_flag
#define remote_data       remote_vt13::data

#define REMOTE_RC_CH_MID          1024
#define REMOTE_RC_CH_MIN          364
#define REMOTE_RC_CH_MAX          1684

#define REMOTE_RC_RH()            ((int16_t)remote_data->ch_0 - REMOTE_RC_CH_MID)
#define REMOTE_RC_RV()            ((int16_t)remote_data->ch_1 - REMOTE_RC_CH_MID)
#define REMOTE_RC_LV()            ((int16_t)remote_data->ch_2 - REMOTE_RC_CH_MID)
#define REMOTE_RC_LH()            ((int16_t)remote_data->ch_3 - REMOTE_RC_CH_MID)

#define REMOTE_RC_SW_C            0
#define REMOTE_RC_SW_N            1
#define REMOTE_RC_SW_S            2
#define REMOTE_RC_SWITCH()        (remote_data->mode_sw)

#define REMOTE_RC_PAUSE()         (remote_data->pause)
#define REMOTE_RC_FN_LEFT()       (remote_data->fn_1)
#define REMOTE_RC_FN_RIGHT()      (remote_data->fn_2)
#define REMOTE_RC_TRIGGER()       (remote_data->trigger)

#define REMOTE_RC_WHEEL()         ((int16_t)remote_data->wheel - REMOTE_RC_CH_MID)

#define REMOTE_KEY_W              (1 << 0)
#define REMOTE_KEY_S              (1 << 1)
#define REMOTE_KEY_A              (1 << 2)
#define REMOTE_KEY_D              (1 << 3)
#define REMOTE_KEY_SHIFT          (1 << 4)
#define REMOTE_KEY_CTRL           (1 << 5)
#define REMOTE_KEY_Q              (1 << 6)
#define REMOTE_KEY_E              (1 << 7)
#define REMOTE_KEY_R              (1 << 8)
#define REMOTE_KEY_F              (1 << 9)
#define REMOTE_KEY_G              (1 << 10)
#define REMOTE_KEY_Z              (1 << 11)
#define REMOTE_KEY_X              (1 << 12)
#define REMOTE_KEY_C              (1 << 13)
#define REMOTE_KEY_V              (1 << 14)
#define REMOTE_KEY_B              (1 << 15)

#define REMOTE_KEY_PRESSED(k)     (remote_data && (remote_data->key & (k)))

#define REMOTE_MOUSE_LEFT_PRESSED()    (remote_data && remote_data->mouse_left)
#define REMOTE_MOUSE_RIGHT_PRESSED()   (remote_data && remote_data->mouse_right)
#define REMOTE_MOUSE_MIDDLE_PRESSED()  (remote_data && remote_data->mouse_middle)

const remote_frame_t* Remote_Init(UART_HandleTypeDef* huart);

#elif defined(REMOTE_DEVICE_DT7)

#include "dt7.h"

#define REMOTE_FRAME_LEN REMOTE_DT7_FRAME_LEN

#define remote_data_flag remote_dt7::data_flag
#define remote_data       remote_dt7::data

#define REMOTE_RC_CH_MID          REMOTE_DT7_CH_MID
#define REMOTE_RC_CH_MIN          REMOTE_DT7_CH_MIN
#define REMOTE_RC_CH_MAX          REMOTE_DT7_CH_MAX

#define REMOTE_RC_RH()            (remote_data->rocker_right_x)
#define REMOTE_RC_RV()            (remote_data->rocker_right_y)
#define REMOTE_RC_LV()            (remote_data->rocker_left_y)
#define REMOTE_RC_LH()            (remote_data->rocker_left_x)
#define REMOTE_RC_WHEEL()         (remote_data->dial)

#define REMOTE_RC_SW_UP           REMOTE_DT7_SW_UP
#define REMOTE_RC_SW_MID          REMOTE_DT7_SW_MID
#define REMOTE_RC_SW_DOWN         REMOTE_DT7_SW_DOWN

#define REMOTE_RC_SW_LEFT()       (remote_data->switch_left)
#define REMOTE_RC_SW_RIGHT()      (remote_data->switch_right)

const dt7_rc_t* Remote_Init(UART_HandleTypeDef* huart);

#elif defined(REMOTE_DEVICE_HT10A)

#include "ht10a.h"

#define REMOTE_FRAME_LEN REMOTE_HT10A_FRAME_LEN

#define remote_data_flag remote_ht10a::data_flag
#define remote_data       remote_ht10a::data

#define REMOTE_RC_CH_MID          REMOTE_HT10A_CH_MID

#define REMOTE_RC_RH()            (remote_data->rocker_right_x)
#define REMOTE_RC_RV()            (remote_data->rocker_right_y)
#define REMOTE_RC_LV()            (remote_data->rocker_left_y)
#define REMOTE_RC_LH()            (remote_data->rocker_left_x)

#define REMOTE_RC_SW_UP           REMOTE_HT10A_SW_UP
#define REMOTE_RC_SW_MID          REMOTE_HT10A_SW_MID
#define REMOTE_RC_SW_DOWN         REMOTE_HT10A_SW_DOWN

#define REMOTE_RC_SW_LEFT()            (remote_data->switch_left)
#define REMOTE_RC_SW_RIGHT()           (remote_data->switch_right)
#define REMOTE_RC_SW_CENTER_LEFT()     (remote_data->switch_center_left)
#define REMOTE_RC_SW_CENTER_RIGHT()    (remote_data->switch_center_right)

#define REMOTE_RC_CH(n)           (remote_data->channels[(n)])

const ht10a_rc_t* Remote_Init(UART_HandleTypeDef* huart);

#endif

uint8_t Remote_Online();

/* Force-restart the underlying UART reception (link self-heal). */
void Remote_Recover();
