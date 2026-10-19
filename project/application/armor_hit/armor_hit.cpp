/**
 * @file    armor_hit.cpp
 * @brief   受击检测 + 角度换算（见 armor_hit.h）
 * @note    数据源：裁判 0x0206。新事件以 robot_hurt_seq 边沿为准（勿轮询残留字段）。
 */
#include "armor_hit.h"

#include "bsp_dwt.h"
#include "referee.h"
#include "robot_cmd.h" /* chassis_cmd_recv.offset_angle */
#include "robot_def.h"
#include "master_process.h"

/* 装甲板 ID → 相对底盘正前方的角度[度，逆时针为正]。
 * 协议 armor_id 常从 0 起; 下表按 0前/1左/2后/3右 占位。
 * TODO(标定): 用真实弹丸命中逐块确认 ID↔方位（手拍会报撞击, 不能用来标定）。 */
static const float ARMOR_ANGLE_DEG[16] = {
    0.0f, 90.0f, 180.0f, 270.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/* 扣血原因: 0=弹丸命中装甲（才计入受击方位） */
static const uint8_t HURT_REASON_PROJECTILE = 0u;

ArmorHit_Info_s g_armor_hit = {};

extern AHRS g_ahrs;

void ArmorHitUpdate(void)
{
    referee_info_t *ref = Referee_GetData();

    if (ref != nullptr)
    {
        static uint16_t last_hurt_seq = 0;
        static uint8_t last_id = 0xFFu;
        static uint32_t last_ms = 0;

        const uint16_t hurt_seq = ref->robot_hurt_seq;
        if (hurt_seq != last_hurt_seq)
        {
            last_hurt_seq = hurt_seq;

            const uint8_t id = (uint8_t)(ref->robot_hurt.armor_id & 0x0Fu);
            const uint8_t reason = (uint8_t)(ref->robot_hurt.HP_deduction_reason & 0x0Fu);
            const uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();

            g_armor_hit.reason = reason;

            /* 只把「弹丸命中」记为受击方位事件; 撞击/超限/离线不进角度与计数 */
            if (reason == HURT_REASON_PROJECTILE)
            {
                /* 同一板短时连发的重复 0x0206 合并（边沿已保证是新包, 这里防裁判连推） */
                if (id != last_id || (now_ms - last_ms) >= 80u)
                {
                    last_id = id;
                    last_ms = now_ms;

                    g_armor_hit.armor_id = id;
                    g_armor_hit.hit_cnt++;
                    g_armor_hit.seq++;
                    g_armor_hit.cnt[id]++;
                    g_armor_hit.angle_chassis = ARMOR_ANGLE_DEG[id];

                    /* 世界系: 装甲在底盘上 → 底盘航向 ≈ 云台 yaw - offset_angle */
                    float chassis_yaw =
                        g_ahrs.output_.euler[2] - chassis_cmd_recv.offset_angle;
                    float world = g_armor_hit.angle_chassis + chassis_yaw;
                    while (world < 0.0f)
                        world += 360.0f;
                    while (world >= 360.0f)
                        world -= 360.0f;
                    g_armor_hit.angle_world = world;
                }
            }
        }
    }

    Minipc_Send_s *tx = Minipc_GetSendData();
    if (tx != nullptr)
    {
        tx->Vision.armor_id = g_armor_hit.armor_id;
        tx->Vision.armor_reason = g_armor_hit.reason;
        tx->Vision.armor_hit_cnt = g_armor_hit.hit_cnt;
        tx->Vision.armor_hit_seq = g_armor_hit.seq;
        tx->Vision.armor_angle_chassis = g_armor_hit.angle_chassis;
        tx->Vision.armor_angle_world = g_armor_hit.angle_world;
    }
}
