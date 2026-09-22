/**
 * @file    referee.h
 * @brief   裁判系统模块（C → C++：无实例，自由函数置于全局命名空间）
 * @note    从 C 版 referee 迁移，逻辑不变。referee_protocol.h（协议/结构定义头）保持原样。
 *          去掉 extern "C"、改用 #pragma once（对齐 crc.h）。
 */
#pragma once

#include "referee_protocol.h"
#include "bsp_usart.h"
#include <stdint.h>

#pragma pack(1)

/* 机器人 ID 信息 */
typedef struct {
    uint8_t  robot_color;       // 0=红, 1=蓝
    uint16_t robot_id;          // 本机器人 ID
    uint16_t client_id;         // 选手端客户端 ID
    uint16_t receiver_id;       // 机器人间通信接收者 ID
} referee_id_t;

/* 裁判系统数据 */
typedef struct {
    referee_id_t id;

    // 接收数据
    game_state_t          game_state;           // 0x0001
    game_result_t         game_result;          // 0x0002
    game_robot_HP_t       game_robot_HP;        // 0x0003
    event_data_t          event_data;           // 0x0101
    referee_warning_t     referee_warning;      // 0x0104
    dart_info_t           dart_info;            // 0x0105
    robot_status_t        robot_status;         // 0x0201
    power_heat_data_t     power_heat;           // 0x0202
    robot_pos_t           robot_pos;            // 0x0203
    buff_musk_t           buff_musk;            // 0x0204
    robot_hurt_t          robot_hurt;           // 0x0206
    shoot_data_t          shoot_data;           // 0x0207
    projectile_allowance_t projectile_allowance; // 0x0208

    // 状态标志
    uint8_t init_flag;
    uint8_t online;
} referee_info_t;

#pragma pack()

/*============================================
 * API
 ============================================*/

/**
 * @brief 初始化裁判系统
 * @param huart 串口句柄 (C板用USART6)
 * @return referee_info_t* 返回裁判数据指针
 */
referee_info_t *Referee_Init(UART_HandleTypeDef *huart);

/**
 * @brief 获取裁判系统数据
 * @return referee_info_t* 数据指针
 */
referee_info_t *Referee_GetData();

/**
 * @brief 检查裁判系统是否在线
 * @return 1=在线, 0=离线
 */
uint8_t Referee_Online();

/**
 * @brief 发送数据帧
 * @param cmd_id 命令码
 * @param data 数据指针
 * @param len 数据长度
 */
void Referee_SendFrame(uint16_t cmd_id, uint8_t *data, uint16_t len);

/**
 * @brief 发送交互数据 (0x0301)
 * @param sub_cmd_id 子内容 ID
 * @param receiver_id 接收者 ID
 * @param data 数据指针
 * @param len 数据长度
 */
void Referee_SendInteractive(uint16_t sub_cmd_id, uint16_t receiver_id, uint8_t *data, uint16_t len);

/*============================================
 * UI 绘图 API
 ============================================*/

/**
 * @brief 删除图层
 * @param operate 0=空操作, 1=删除图层, 2=删除所有
 * @param layer 图层数 (0-9)
 */
void Referee_UIDelete(uint8_t operate, uint8_t layer);

/**
 * @brief 绘制图形
 * @param graph_name 图形名 (3字节)
 * @param operate 操作 (增加/修改/删除)
 * @param type 图形类型 (直线/矩形/圆等)
 * @param layer 图层
 * @param color 颜色
 * @param ... 图形参数
 */
void Referee_UIDraw(uint8_t *graph_name, uint8_t operate, uint8_t type,
                    uint8_t layer, uint8_t color, ...);

/**
 * @brief 刷新图形到选手端
 * @param cnt 图形数量
 */
void Referee_UIRefresh(int cnt, ...);

/**
 * @brief 绘制字符
 * @param graph_name 图形名
 * @param operate 操作
 * @param layer 图层
 * @param color 颜色
 * @param size 字体大小
 * @param width 线宽
 * @param x, y 坐标
 * @param fmt 字符串格式
 */
void Referee_UIChar(uint8_t *graph_name, uint8_t operate, uint8_t layer,
                    uint8_t color, uint8_t size, uint8_t width,
                    uint16_t x, uint16_t y, const char *fmt, ...);

/**
 * @brief 刷新字符到选手端
 * @param data 字符数据
 */
void Referee_UICharRefresh(uint8_t *data);
