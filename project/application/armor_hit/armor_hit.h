/**
 * @file    armor_hit.h
 * @brief   受击检测：判断哪块装甲板被打 + 算出对应角度（底盘系 / 世界系）
 */
#pragma once

#include <stdint.h>

typedef struct
{
    uint8_t armor_id;       // 最近一次受击的装甲板 ID（0 = 开机以来无受击）
    uint8_t reason;         // HP_deduction_reason
    uint8_t hit_cnt;        // 累计受击次数（8 位回绕）
    uint8_t seq;            // 事件序号，每检测到一次新受击 +1（小电脑用它去重）
    float angle_chassis;    // 受击方向, 相对底盘正前方 [度], 逆时针为正
    float angle_world;      // 叠加当前 yaw 后的场地方位 [度], 0~360
    uint8_t cnt[16];        // 各装甲板 ID 的累计受击次数（标定用：敲哪块板哪个涨）
} ArmorHit_Info_s;

extern ArmorHit_Info_s g_armor_hit;

/** 受击检测 + 角度换算 + 填进发给小电脑的帧。周期调用（200Hz 随 RobotCMDTask 即可）。 */
void ArmorHitUpdate(void);
