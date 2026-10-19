/**
 * @file    armor_hit.h
 * @brief   受击检测：判断哪块装甲板被打 + 算出对应角度（底盘系 / 世界系）
 */
#pragma once

#include <stdint.h>

typedef struct
{
    uint8_t armor_id;       // 最近一次弹丸命中的装甲 ID（0~15；开机未命中时为 0 且 hit_cnt=0）
    uint8_t reason;         // 最近一帧 0x0206 的 HP_deduction_reason（含非弹丸）
    uint8_t hit_cnt;        // 累计弹丸受击次数（8 位回绕）
    uint8_t seq;            // 弹丸受击事件序号（小电脑用它判断新事件）
    float angle_chassis;    // 受击方向, 相对底盘正前方 [度], 逆时针为正
    float angle_world;      // 底盘航向叠加后的方位 [度], 0~360
    uint8_t cnt[16];        // 各装甲板 ID 的累计弹丸受击次数（标定用）
} ArmorHit_Info_s;

extern ArmorHit_Info_s g_armor_hit;

/** 受击检测 + 角度换算 + 填进发给小电脑的帧。周期调用（200Hz 随 RobotCMDTask 即可）。 */
void ArmorHitUpdate(void);
