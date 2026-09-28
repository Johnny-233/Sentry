/**
 * @file    armor_hit.cpp
 * @brief   受击检测 + 角度换算（见 armor_hit.h）
 * @note    数据源：裁判系统 0x0206 robot_hurt（referee 模块已解析，含 armor_id / 扣血原因）。
 */
#include "armor_hit.h"

#include "bsp_dwt.h"
#include "referee.h"
#include "robot_def.h"
#include "master_process.h"

/* 装甲板 ID → 相对底盘正前方的角度[度，逆时针为正]。
 * TODO(标定): 依次敲四块装甲板，看 g_armor_hit.cnt[] 哪个涨 → 得到 ID 与物理位置的对应关系，
 *             再按机械朝向填这张表（下标 0 不用；暂按 前=1, 左=2, 后=3, 右=4 占位）。 */
static const float ARMOR_ANGLE_DEG[16] = {
    0.0f, 0.0f, 90.0f, 180.0f, 270.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

ArmorHit_Info_s g_armor_hit = {};

/* 云台 IMU 的全局实例（定义在 application/cmd/robot_cmd.cpp） */
extern AHRS g_ahrs;

void ArmorHitUpdate(void)
{
    referee_info_t *ref = Referee_GetData();

    if (ref != nullptr)
    {
        static uint8_t last_id = 0;
        static uint32_t last_ms = 0;

        const uint8_t id = (uint8_t)(ref->robot_hurt.armor_id & 0x0Fu);
        const uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();

        /* 新事件判据：ID 非 0 且（换了装甲板 或 距上次已超过 200ms）
           —— 裁判会把同一受击状态连续推几帧，不这样滤会重复计数。 */
        if (id != 0u && (id != last_id || (now_ms - last_ms) >= 200u))
        {
            last_id = id;
            last_ms = now_ms;

            g_armor_hit.armor_id = id;
            g_armor_hit.reason = (uint8_t)(ref->robot_hurt.HP_deduction_reason & 0x0Fu);
            g_armor_hit.hit_cnt++;
            g_armor_hit.seq++;
            g_armor_hit.cnt[id]++;
            g_armor_hit.angle_chassis = ARMOR_ANGLE_DEG[id];

            /* 世界系：底盘系角度 + 当前 yaw（IMU 绝对角，度），归一到 0~360 */
            float world = g_armor_hit.angle_chassis + g_ahrs.output_.euler[2];
            while (world < 0.0f)
                world += 360.0f;
            while (world >= 360.0f)
                world -= 360.0f;
            g_armor_hit.angle_world = world;
        }
    }

    /* 填进发给小电脑的帧（模块持有唯一一份发送缓冲） */
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
