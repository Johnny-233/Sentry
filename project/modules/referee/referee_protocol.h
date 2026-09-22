#ifndef REFEREE_PROTOCOL_H
#define REFEREE_PROTOCOL_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif


/*============================================
 * 帧格式定义
 * SOF(1) + data_length(2) + seq(1) + CRC8(1) + cmd_id(2) + data(n) + CRC16(2)
 ============================================*/

#define REFEREE_SOF         0xA5
#define REFEREE_FRAME_HEADER_LEN   5
#define REFEREE_CMD_ID_LEN         2
#define REFEREE_FRAME_TAIL_LEN     2

#pragma pack(1)

/* 帧头 */
typedef struct {
    uint8_t  SOF;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  CRC8;
} xFrameHeader;

/* 交互数据头 */
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

#pragma pack()

/*============================================
 * 命令码 ID
 ============================================*/

typedef enum {
    ID_game_state             = 0x0001,  // 比赛状态 1Hz
    ID_game_result            = 0x0002,  // 比赛结果
    ID_game_robot_HP          = 0x0003,  // 全队血量 3Hz
    ID_event_data             = 0x0101,  // 场地事件 1Hz
    ID_referee_warning        = 0x0104,  // 裁判警告
    ID_dart_info              = 0x0105,  // 飞镖数据 1Hz
    ID_robot_status           = 0x0201,  // 本机器人状态 10Hz
    ID_power_heat             = 0x0202,  // 功率热量 10Hz
    ID_robot_pos              = 0x0203,  // 机器人位置 1Hz
    ID_buff_musk              = 0x0204,  // 增益数据 3Hz
    ID_robot_hurt             = 0x0206,  // 伤害状态
    ID_shoot_data             = 0x0207,  // 射击数据
    ID_projectile_allowance   = 0x0208,  // 允许发弹量 10Hz
    ID_student_interactive    = 0x0301,  // 交互数据
} CmdID_e;

/* 交互子内容 ID */
typedef enum {
    UI_Delete         = 0x0100,
    UI_Draw1          = 0x0101,
    UI_Draw2          = 0x0102,
    UI_Draw5          = 0x0103,
    UI_Draw7          = 0x0104,
    UI_DrawChar       = 0x0110,
    Robot_Communicate = 0x0200,  // 机器人间通信 0x0200~0x02FF
} InteractiveID_e;

/*============================================
 * 数据段长度
 ============================================*/

typedef enum {
    LEN_game_state           = 11,   // 0x0001
    LEN_game_result          = 1,    // 0x0002
    LEN_game_robot_HP        = 20,   // 0x0003
    LEN_event_data           = 4,    // 0x0101
    LEN_referee_warning      = 3,    // 0x0104
    LEN_dart_info            = 3,    // 0x0105
    LEN_robot_status         = 17,   // 0x0201
    LEN_power_heat           = 14,   // 0x0202
    LEN_robot_pos            = 12,   // 0x0203
    LEN_buff_musk            = 8,    // 0x0204
    LEN_robot_hurt           = 1,    // 0x0206
    LEN_shoot_data           = 7,    // 0x0207
    LEN_projectile_allowance = 8,    // 0x0208
    LEN_student_interactive  = 118,  // 0x0301
} DataLen_e;

/* 交互数据子内容长度 */
typedef enum {
    UI_Delete_Len      = 2,
    UI_Draw1_Len       = 15,
    UI_Draw2_Len       = 30,
    UI_Draw5_Len       = 75,
    UI_Draw7_Len       = 105,
    UI_DrawChar_Len    = 45,
    Interactive_Header_Len = 6,
} InteractiveLen_e;

/*============================================
 * 数据结构定义 (2026 协议)
 * 注意：裁判系统数据段是紧密排列的，必须 1 字节对齐，
 * 否则 uint16/float/uint64 前的填充会让 sizeof 大于 data_length，
 * memcpy 时越界读。
 ============================================*/

#pragma pack(1)

/* 0x0001 比赛状态 */
typedef struct {
    uint8_t  game_type : 4;
    uint8_t  game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_state_t;

/* 0x0002 比赛结果 */
typedef struct {
    uint8_t winner;
} game_result_t;

/* 0x0003 全队血量 */
typedef struct {
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    int16_t  damage_difference;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
    uint16_t enemy_outpost_HP;
    uint16_t enemy_base_HP;
} game_robot_HP_t;

/* 0x0101 场地事件 */
typedef struct {
    uint32_t event_data;
} event_data_t;

/* 0x0104 裁判警告 */
typedef struct {
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

/* 0x0105 飞镖数据 */
typedef struct {
    uint8_t  dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;

/* 0x0201 本机器人状态 */
typedef struct {
    uint8_t  robot_id;
    uint8_t  robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    float    bullet_speed_limit;
    uint8_t  power_management_gimbal_output  : 1;
    uint8_t  power_management_chassis_output : 1;
    uint8_t  power_management_shooter_output : 1;
} robot_status_t;

/* 0x0202 功率热量 */
typedef struct {
    uint16_t reserved1;
    uint16_t reserved2;
    float    reserved3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

/* 0x0203 机器人位置 */
typedef struct {
    float x;
    float y;
    float angle;
} robot_pos_t;

/* 0x0204 增益数据 */
typedef struct {
    uint8_t  recovery_rate;
    uint16_t cooling_rate;
    uint8_t  defence_buff;
    uint8_t  vulnerability_buff;
    uint16_t attack_buff;
    uint8_t  remaining_energy;
} buff_musk_t;

/* 0x0206 伤害状态 */
typedef struct {
    uint8_t armor_id          : 4;
    uint8_t HP_deduction_reason : 4;
} robot_hurt_t;

/* 0x0207 射击数据 */
typedef struct {
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float   initial_speed;
} shoot_data_t;

/* 0x0208 允许发弹量 */
typedef struct {
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} projectile_allowance_t;

#pragma pack()

/* 机器人 ID */
typedef enum {
    Robot_Red_Hero     = 1,
    Robot_Red_Engineer = 2,
    Robot_Red_Standard1 = 3,
    Robot_Red_Standard2 = 4,
    Robot_Red_Standard3 = 5,
    Robot_Red_Aerial   = 6,
    Robot_Red_Sentry   = 7,
    Robot_Red_Radar    = 9,
    Robot_Blue_Hero     = 101,
    Robot_Blue_Engineer = 102,
    Robot_Blue_Standard1 = 103,
    Robot_Blue_Standard2 = 104,
    Robot_Blue_Standard3 = 105,
    Robot_Blue_Aerial   = 106,
    Robot_Blue_Sentry   = 107,
    Robot_Blue_Radar    = 109,
} RobotID_e;

/* UI 颜色 */
typedef enum {
    UI_Color_Main = 0,
    UI_Color_Yellow = 1,
    UI_Color_Green = 2,
    UI_Color_Orange = 3,
    UI_Color_Purplish_red = 4,
    UI_Color_Pink = 5,
    UI_Color_Cyan = 6,
    UI_Color_Black = 7,
    UI_Color_White = 8,
} UI_Color_e;

/* UI 图形操作 */
typedef enum {
    UI_Graph_NoOperate = 0,
    UI_Graph_Add = 1,
    UI_Graph_Change = 2,
    UI_Graph_Delete = 3,
} UI_Graph_Operate_e;

/* UI 图形类型 */
typedef enum {
    UI_Graph_Line = 0,
    UI_Graph_Rectangle = 1,
    UI_Graph_Circle = 2,
    UI_Graph_Ellipse = 3,
    UI_Graph_Arc = 4,
    UI_Graph_Float = 5,
    UI_Graph_Int = 6,
    UI_Graph_Char = 7,
} UI_Graph_Type_e;

/* UI 删除操作 */
typedef enum {
    UI_Delete_NoOperate = 0,
    UI_Delete_Layer = 1,
    UI_Delete_All = 2,
} UI_Delete_Operate_e;


#ifdef __cplusplus
}
#endif

#endif // REFEREE_PROTOCOL_H
