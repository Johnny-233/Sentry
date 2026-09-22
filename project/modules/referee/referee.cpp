/**
 * @file    referee.cpp
 * @brief   裁判系统模块实现（C → C++）
 * @note    原 C 版逻辑不变；字节缓冲强转结构体指针统一改 memcpy（§9），
 *          去 malloc、去 NULL 检查。
 */
#include "referee.h"
#include "serial.h"
#include "daemon.h"
#include "crc_rm.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*============================================
 * 内部变量
 ============================================*/

static referee_info_t referee_info;
static Serial referee_serial;
static Daemon referee_daemon;
static uint8_t referee_tx_buf[128];
static uint8_t referee_seq = 0;

/*============================================
 * 帧解析
 ============================================*/

static void Referee_ParseFrame(uint8_t *buf, uint16_t len)
{
    uint16_t offset = 0;

    // 至少一个完整帧（帧头5 + cmd_id2 + 帧尾2 = 9 字节）才进入解析
    while (offset + REFEREE_FRAME_HEADER_LEN + REFEREE_CMD_ID_LEN + REFEREE_FRAME_TAIL_LEN <= len)
    {
        xFrameHeader header;
        memcpy(&header, buf + offset, sizeof(xFrameHeader));

        // 不是帧头，逐字节找下一个 SOF
        if (header.SOF != REFEREE_SOF)
        {
            offset++;
            continue;
        }

        // 校验 CRC8（帧头），不过可能是假 SOF，逐字节重找
        uint8_t crc8 = CRC8_Calculate(buf + offset, REFEREE_FRAME_HEADER_LEN - 1);
        if (crc8 != header.CRC8)
        {
            offset++;
            continue;
        }

        // 获取数据长度
        uint16_t data_len = header.data_length;

        // 剩余长度不足这一帧（帧头5 + cmd_id2 + data + 帧尾2），等下一包
        if (data_len > len - offset - (REFEREE_FRAME_HEADER_LEN + REFEREE_CMD_ID_LEN + REFEREE_FRAME_TAIL_LEN))
            break;

        uint16_t frame_len = REFEREE_FRAME_HEADER_LEN + REFEREE_CMD_ID_LEN + data_len + REFEREE_FRAME_TAIL_LEN;

        // 校验 CRC16（整帧），不过跳过这一帧
        uint16_t expected_crc16 = CRC16_Calculate(buf + offset, frame_len - REFEREE_FRAME_TAIL_LEN);
        uint16_t actual_crc16;
        memcpy(&actual_crc16, buf + offset + frame_len - REFEREE_FRAME_TAIL_LEN, sizeof(uint16_t));
        if (expected_crc16 != actual_crc16)
        {
            offset += frame_len;
            continue;
        }

        // cmd_id 和数据指针
        uint16_t cmd_id;
        memcpy(&cmd_id, buf + offset + REFEREE_FRAME_HEADER_LEN, sizeof(uint16_t));
        uint8_t *data = buf + offset + REFEREE_FRAME_HEADER_LEN + REFEREE_CMD_ID_LEN;

        // 路由到对应处理函数
        switch (cmd_id) {
            case ID_game_state:
                memcpy(&referee_info.game_state, data, sizeof(game_state_t));
                break;
            case ID_game_result:
                memcpy(&referee_info.game_result, data, sizeof(game_result_t));
                break;
            case ID_game_robot_HP:
                memcpy(&referee_info.game_robot_HP, data, sizeof(game_robot_HP_t));
                break;
            case ID_event_data:
                memcpy(&referee_info.event_data, data, sizeof(event_data_t));
                break;
            case ID_referee_warning:
                memcpy(&referee_info.referee_warning, data, sizeof(referee_warning_t));
                break;
            case ID_dart_info:
                memcpy(&referee_info.dart_info, data, sizeof(dart_info_t));
                break;
            case ID_robot_status:
                memcpy(&referee_info.robot_status, data, sizeof(robot_status_t));
                referee_info.id.robot_id = referee_info.robot_status.robot_id;
                referee_info.id.robot_color = referee_info.robot_status.robot_id > 7 ? 1 : 0;
                referee_info.id.client_id = 0x0100 + referee_info.robot_status.robot_id;
                break;
            case ID_power_heat:
                memcpy(&referee_info.power_heat, data, sizeof(power_heat_data_t));
                break;
            case ID_robot_pos:
                memcpy(&referee_info.robot_pos, data, sizeof(robot_pos_t));
                break;
            case ID_buff_musk:
                memcpy(&referee_info.buff_musk, data, sizeof(buff_musk_t));
                break;
            case ID_robot_hurt:
                memcpy(&referee_info.robot_hurt, data, sizeof(robot_hurt_t));
                break;
            case ID_shoot_data:
                memcpy(&referee_info.shoot_data, data, sizeof(shoot_data_t));
                break;
            case ID_projectile_allowance:
                memcpy(&referee_info.projectile_allowance, data, sizeof(projectile_allowance_t));
                break;
            default:
                break;
        }

        // 喂狗
        referee_daemon.reset();

        // 前进到下一帧
        offset += frame_len;
    }
}

/*============================================
 * 串口回调
 ============================================*/

static void Referee_SerialCallback(uint16_t len)
{
    Referee_ParseFrame(referee_serial.recv_buf_, len);
}

/*============================================
 * 离线回调
 ============================================*/

static void Referee_OfflineCallback(void *device)
{
    (void)device;
    referee_info.online = 0;
}

/*============================================
 * 初始化
 ============================================*/

referee_info_t *Referee_Init(UART_HandleTypeDef *huart)
{
    memset(&referee_info, 0, sizeof(referee_info_t));

    // 初始化 Daemon (100ms 超时)
    Daemon::Config daemon_config = {
        .tim_config = { .htim = &htim5 },
        .cycle = 100,
        .daemon_callback = Referee_OfflineCallback,
        .device = nullptr,
    };
    referee_daemon.init(daemon_config);

    // 初始化 Serial
    Serial::Config serial_config = {
        .usart_handle = huart,
        .htim = &htim5,
        .rx_callback = Referee_SerialCallback,
    };
    referee_serial.init(serial_config);

    referee_info.init_flag = 1;
    return &referee_info;
}

/*============================================
 * 数据访问
 ============================================*/

referee_info_t *Referee_GetData()
{
    return &referee_info;
}

uint8_t Referee_Online()
{
    return referee_daemon.online_;
}

/*============================================
 * 发送
 ============================================*/

void Referee_SendFrame(uint16_t cmd_id, uint8_t *data, uint16_t len)
{
    uint16_t frame_len = REFEREE_FRAME_HEADER_LEN + REFEREE_CMD_ID_LEN + len + REFEREE_FRAME_TAIL_LEN;
    uint8_t *buf = referee_tx_buf;

    // 帧头
    xFrameHeader header;
    header.SOF = REFEREE_SOF;
    header.data_length = len;
    header.seq = referee_seq++;
    header.CRC8 = CRC8_Calculate(reinterpret_cast<const uint8_t *>(&header), REFEREE_FRAME_HEADER_LEN - 1);
    memcpy(buf, &header, sizeof(xFrameHeader));

    // cmd_id
    memcpy(buf + REFEREE_FRAME_HEADER_LEN, &cmd_id, 2);

    // data
    if (data && len > 0)
        memcpy(buf + REFEREE_FRAME_HEADER_LEN + REFEREE_CMD_ID_LEN, data, len);

    // CRC16
    uint16_t crc16 = CRC16_Calculate(buf, frame_len - 2);
    memcpy(buf + frame_len - 2, &crc16, 2);

    // 发送
    referee_serial.send(buf, frame_len);
}

void Referee_SendInteractive(uint16_t sub_cmd_id, uint16_t receiver_id, uint8_t *data, uint16_t len)
{
    uint8_t buf[120];

    // 交互数据头
    ext_student_interactive_header_data_t header;
    header.data_cmd_id = sub_cmd_id;
    header.sender_ID = referee_info.id.robot_id;
    header.receiver_ID = receiver_id;
    memcpy(buf, &header, sizeof(header));
    uint16_t offset = Interactive_Header_Len;

    // 数据
    if (data && len > 0) {
        memcpy(buf + offset, data, len);
        offset += len;
    }

    Referee_SendFrame(ID_student_interactive, buf, offset);
}

/*============================================
 * UI 绘图
 ============================================*/

// 图形数据结构 (15 字节, 必须 1 字节对齐, 否则 figure_name[3] 后补 1 字节变 16)
#pragma pack(1)
typedef struct {
    uint8_t  figure_name[3];
    uint32_t operate_type : 3;
    uint32_t figure_type  : 3;
    uint32_t layer        : 4;
    uint32_t color        : 4;
    uint32_t details_a    : 9;
    uint32_t details_b    : 9;
    uint32_t width        : 10;
    uint32_t start_x      : 11;
    uint32_t start_y      : 11;
    uint32_t details_c    : 10;
    uint32_t details_d    : 11;
    uint32_t details_e    : 11;
} interaction_figure_t;
#pragma pack()

void Referee_UIDelete(uint8_t operate, uint8_t layer)
{
    uint8_t data[2];
    data[0] = operate;
    data[1] = layer;
    Referee_SendInteractive(UI_Delete, referee_info.id.client_id, data, 2);
}

void Referee_UIDraw(uint8_t *graph_name, uint8_t operate, uint8_t type,
                    uint8_t layer, uint8_t color, ...)
{
    interaction_figure_t figure;
    memset(&figure, 0, sizeof(figure));

    memcpy(figure.figure_name, graph_name, 3);
    figure.operate_type = operate;
    figure.figure_type = type;
    figure.layer = layer;
    figure.color = color;

    va_list args;
    va_start(args, color);

    switch (type) {
        case UI_Graph_Line:
            figure.width = va_arg(args, uint32_t);
            figure.start_x = va_arg(args, uint32_t);
            figure.start_y = va_arg(args, uint32_t);
            figure.details_d = va_arg(args, uint32_t);  // end_x
            figure.details_e = va_arg(args, uint32_t);  // end_y
            break;
        case UI_Graph_Rectangle:
            figure.width = va_arg(args, uint32_t);
            figure.start_x = va_arg(args, uint32_t);
            figure.start_y = va_arg(args, uint32_t);
            figure.details_d = va_arg(args, uint32_t);  // end_x
            figure.details_e = va_arg(args, uint32_t);  // end_y
            break;
        case UI_Graph_Circle:
            figure.width = va_arg(args, uint32_t);
            figure.start_x = va_arg(args, uint32_t);
            figure.start_y = va_arg(args, uint32_t);
            figure.details_c = va_arg(args, uint32_t);  // radius
            break;
        case UI_Graph_Float:
            figure.details_a = va_arg(args, uint32_t);  // font_size
            figure.width = va_arg(args, uint32_t);
            figure.start_x = va_arg(args, uint32_t);
            figure.start_y = va_arg(args, uint32_t);
            figure.details_d = va_arg(args, uint32_t);  // value * 1000
            break;
        case UI_Graph_Int:
            figure.details_a = va_arg(args, uint32_t);  // font_size
            figure.width = va_arg(args, uint32_t);
            figure.start_x = va_arg(args, uint32_t);
            figure.start_y = va_arg(args, uint32_t);
            figure.details_d = va_arg(args, uint32_t);  // value
            break;
        default:
            break;
    }

    va_end(args);

    Referee_SendInteractive(UI_Draw1, referee_info.id.client_id, reinterpret_cast<uint8_t *>(&figure), sizeof(figure));
}

void Referee_UIRefresh(int cnt, ...)
{
    // 批量刷新多个图形
    uint8_t buf[120];
    uint16_t offset = 0;

    va_list args;
    va_start(args, cnt);

    for (int i = 0; i < cnt; i++) {
        interaction_figure_t *figure = va_arg(args, interaction_figure_t *);
        memcpy(buf + offset, figure, sizeof(interaction_figure_t));
        offset += sizeof(interaction_figure_t);
    }

    va_end(args);

    // 根据数量选择子内容 ID
    uint16_t sub_cmd_id;
    switch (cnt) {
        case 1: sub_cmd_id = UI_Draw1; break;
        case 2: sub_cmd_id = UI_Draw2; break;
        case 5: sub_cmd_id = UI_Draw5; break;
        case 7: sub_cmd_id = UI_Draw7; break;
        default: sub_cmd_id = UI_Draw1; break;
    }

    Referee_SendInteractive(sub_cmd_id, referee_info.id.client_id, buf, offset);
}

void Referee_UIChar(uint8_t *graph_name, uint8_t operate, uint8_t layer,
                    uint8_t color, uint8_t size, uint8_t width,
                    uint16_t x, uint16_t y, const char *fmt, ...)
{
    uint8_t buf[45];
    uint16_t offset = 0;

    // 图形配置 (15 字节)
    interaction_figure_t figure;
    memcpy(figure.figure_name, graph_name, 3);
    figure.operate_type = operate;
    figure.figure_type = UI_Graph_Char;
    figure.layer = layer;
    figure.color = color;
    figure.details_a = size;
    figure.width = width;
    figure.start_x = x;
    figure.start_y = y;
    memcpy(buf, &figure, sizeof(figure));
    offset += sizeof(interaction_figure_t);

    // 字符数据 (30 字节)
    char str[30];
    va_list args;
    va_start(args, fmt);
    vsnprintf(str, 30, fmt, args);
    va_end(args);

    uint16_t str_len = strlen(str);
    if (str_len > 30) str_len = 30;
    memcpy(buf + offset, str, str_len);
    offset += str_len;

    Referee_SendInteractive(UI_DrawChar, referee_info.id.client_id, buf, offset);
}

void Referee_UICharRefresh(uint8_t *data)
{
    // 字符刷新直接发送
    Referee_SendInteractive(UI_DrawChar, referee_info.id.client_id, data, 45);
}
