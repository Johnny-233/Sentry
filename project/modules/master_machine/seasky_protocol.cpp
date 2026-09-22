/**
 * @file    seasky_protocol.cpp
 * @brief   视觉/小电脑 seasky 协议的打/解包（由 Modules/master_machine/seasky_protocol.c 移植）
 * @note    C → C++，逻辑与字节偏移逐行照搬；只把日志从旧 LOGERROR/LOGWARNING 换成对方的 LOG_ERR/LOG_INFO。
 *
 * CRC 说明（重要）：
 *   本模块用的是**本仓原本的 CRC 实现**，即同目录下原样搬过来的 `crc8.c` / `crc16.c`：
 *     - crc_8 : SHT75 表(多项式 0x31, MSB-first), 初值 CRC_START_8 = 0x00
 *     - crc_16: 反射表, 多项式 CRC_POLY_16 = 0xA001(Modbus), 初值 CRC_START_16 = 0xFFFF
 *   对方的 `crc_rm.h` 是 CRC-8(初值 0xFF) + CRC-16/CCITT(反射 0x8408)，**两者结果不同，不能混用**，
 *   所以这里没有用 crc_rm。
 */
#include "master_process.h"
#include "seasky_protocol.h"

extern "C" {
#include "crc8.h"
#include "crc16.h"
}

#include <string.h>
#include "bsp_log.h"

/* 获取CRC8校验码 */
uint8_t Get_CRC8_Check(uint8_t *pchMessage, uint16_t dwLength)
{
    return crc_8(pchMessage, dwLength);
}

/* 检验CRC8数据段 */
static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = crc_8(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/* 获取CRC16校验码 */
uint16_t Get_CRC16_Check(uint8_t *pchMessage, uint32_t dwLength)
{
    return crc_16(pchMessage, (uint16_t)dwLength);
}

/* 检验CRC16数据段 */
static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = crc_16(pchMessage, (uint16_t)(dwLength - 2));
    return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
}

/* 检验数据帧头 */
static uint8_t protocol_heade_Check(protocol_rm_struct *pro, uint8_t *rx_buf)
{
    if (rx_buf[0] == PROTOCOL_CMD_ID)
    {
        pro->header.sof = rx_buf[0];
        // pro->header.data_length = (rx_buf[2] << 8) | rx_buf[1];
        // pro->header.crc_check = rx_buf[3];
        // pro->cmd_id = (rx_buf[5] << 8) | rx_buf[4];
        return 1;
    }
    return 0;
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void get_protocol_send_Vision_data(uint16_t send_id,       // 信号id
                                   uint16_t flags_register, // 16位寄存器
                                   Minipc_Send_s *tx_data,  // 待发送的float数据
                                   uint8_t float_length,    // float的数据长度
                                   uint8_t *tx_buf,         // 待发送的数据帧
                                   uint16_t *tx_buf_len)    // 待发送的数据帧长度
{
    static uint16_t crc16;
    static uint16_t data_len;

    (void)send_id;
    (void)flags_register;
    (void)float_length;

    // 有效载荷长度: 1(header) + 38(payload) = 39字节
    data_len = 1 + sizeof(tx_data->Vision) - 1;

    /*帧头部分*/
    tx_buf[0] = SEND_VISION_ID;
    /*数据段: 跳过结构体中的header字段, 从detect_color开始复制 */
    /*对应Python格式: <BBfffffHHHHHBBf (38字节) */
    memcpy(&tx_buf[1], &tx_data->Vision.detect_color, sizeof(tx_data->Vision) - 1);

    /*整包校验: CRC16覆盖 header + payload */
    crc16 = crc_16(&tx_buf[0], data_len);
    tx_buf[data_len] = crc16 & 0xff;
    tx_buf[data_len + 1] = (crc16 >> 8) & 0xff;

    *tx_buf_len = data_len + 2; // header + payload + CRC16
}

/*
    此函数用于处理接收数据，
    返回数据内容的id
*/
/* 解析一帧至少需要的字节数: header(1) + 6个4字节字段(24) = 25, 加上帧尾才完整 */
#define VISION_RX_MIN_LEN 25u

/* 【重要】严格校验开关 —— **已按要求打开(=1)**
 *   1 = 启用 长度 + CRC16 校验, 失败丢帧。这样"任何以 0x5A 开头的字节流"就不能再决定云台指向与
 *       开火许可了(CODE_REVIEW 里记的那条安全隐患)。
 *   0 = 旧行为: 只判首字节, 收到就采信。
 * ⚠️ 打开的前提是小电脑(PC)端发的是"整帧 CRC16、小端、放在最后 2 字节", 与本文件
 *    get_protocol_send_Vision_data() 的发帧格式一致。**原代码注释里记着"开启校验后现场收不到
 *    小电脑数据"**, 所以第一次联调务必看 RTT:
 *      "[vision] CRC16 mismatch ... (len=...)" → PC 端的 CRC 算法/字节序/覆盖长度与我们不一致
 *      "[vision] short frame len=... (<25)"    → 收到的是半包/噪声
 *      "[vision] CRC16 diag ..."               → 首次失败时打的诊断(只打一次), 含实际长度与两个 CRC
 *    若确实收不到数据: 优先让 PC 端对齐(整帧小端 CRC16), 实在不行再把这里改回 0 先跑通。
 * 说明: CRC 覆盖"本次实际收到的整帧"(len 字节) —— 因为小电脑真实帧长可能是 27 也可能更长,
 *       写死 27 会误判。 */
#define VISION_RX_STRICT_CHECK 1

void get_protocol_info_vision(uint8_t *rx_buf,
                              uint16_t len,
                              uint16_t *flags_register,
                              Minipc_Recv_s *recv_data)
{
    /* 严格校验开启时使用的失败计数(默认关闭, 保留声明以免切换宏时编译不过) */
    static uint32_t crc_fail_cnt = 0;
    static uint32_t len_fail_cnt = 0;

    if (rx_buf == NULL || recv_data == NULL)
        return;

    /* 与原实现一致: 首字节不是帧头就不处理 */
    if (rx_buf[0] != PROTOCOL_CMD_ID)
        return;

#if VISION_RX_STRICT_CHECK
    /* 长度校验: 短包/半包时缓冲区里是上一包的残留, 不能解析 */
    if (len < VISION_RX_MIN_LEN)
    {
        if ((++len_fail_cnt % 100u) == 0u)
            LOG_ERR(LOG_MOD_COMM, "vision", "[vision] short frame len=%u (<%u), dropped %u\r\n",
                    (unsigned)len, (unsigned)VISION_RX_MIN_LEN, (unsigned)len_fail_cnt);
        return;
    }
    /* CRC16: 覆盖本次收到的整帧, 校验值在最后 2 字节(小端) */
    if (!CRC16_Check_Sum(rx_buf, len))
    {
        /* 首次失败打一条诊断(只打一次): 实际长度 + 帧尾两个字节 + 我们算出来的 CRC,
           用来判断 PC 端到底是 CRC 算法不对、字节序反了、还是长度/覆盖范围不一致。 */
        static uint8_t crc_diag_logged = 0;
        if (!crc_diag_logged)
        {
            const uint16_t calc_crc = crc_16(rx_buf, (uint16_t)(len - 2));
            LOG_INFO(LOG_MOD_COMM, "vision",
                     "[vision] CRC16 diag(first fail): len=%u, tail[%u]=0x%02X tail[%u]=0x%02X, "
                     "calc_over_len-2=0x%04X (小端期望: 低字节在前)\r\n",
                     (unsigned)len, (unsigned)(len - 2), (unsigned)rx_buf[len - 2],
                     (unsigned)(len - 1), (unsigned)rx_buf[len - 1], (unsigned)calc_crc);
            crc_diag_logged = 1;
        }
        if ((++crc_fail_cnt % 100u) == 0u)
            LOG_ERR(LOG_MOD_COMM, "vision", "[vision] CRC16 mismatch on %u frames (len=%u), check the PC-side protocol\r\n",
                    (unsigned)crc_fail_cnt, (unsigned)len);
        return;
    }
#else
    /* 校验关闭时的诊断: 只打印第一帧, 报告实际长度与 CRC 是否匹配, 便于定位 PC 端协议 */
    static uint8_t diag_logged = 0;
    if (!diag_logged)
    {
        uint16_t recv_crc = (uint16_t)((rx_buf[25] << 8) | rx_buf[26]);
        uint16_t calc_crc = Get_CRC16_Check(rx_buf, (uint16_t)(len >= 2u ? len - 2u : 0u));
        LOG_INFO(LOG_MOD_COMM, "vision", "[vision] diag: len=%u, crc_in_frame=0x%04X, crc_calc_over_len-2=0x%04X (%s)\r\n",
                 (unsigned)len, recv_crc, calc_crc, (recv_crc == calc_crc) ? "CRC MATCH" : "CRC MISMATCH");
        diag_logged = 1;
    }
#endif

    /* 与原实现一致: 直接解析到 recv_data(字段偏移未改) */
    *flags_register = (rx_buf[7] << 8) | rx_buf[6];
    recv_data->Vision.header = rx_buf[0];
    memcpy(&recv_data->Vision.linear_velocity_x, &rx_buf[1], sizeof(float));
    memcpy(&recv_data->Vision.linear_velocity_y, &rx_buf[5], sizeof(float));
    memcpy(&recv_data->Vision.gimbal_mode, &rx_buf[9], sizeof(int32_t));
    memcpy(&recv_data->Vision.yaw, &rx_buf[13], sizeof(float));
    memcpy(&recv_data->Vision.pitch, &rx_buf[17], sizeof(float));
    memcpy(&recv_data->Vision.can_fire, &rx_buf[21], sizeof(int32_t));
    recv_data->Vision.checksum = (rx_buf[25] << 8) | rx_buf[26];
}
