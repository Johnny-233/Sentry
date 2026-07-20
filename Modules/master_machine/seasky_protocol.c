#include "master_process.h"
#include "seasky_protocol.h"
#include "crc8.h"
#include "crc16.h"
#include "memory.h"

static Minipc_Recv_s minipc_recv_data;
/*获取CRC8校验码*/
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength)
{
    return crc_8(pchMessage,dwLength);
}
/*检验CRC8数据段*/
static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = crc_8(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*获取CRC16校验码*/
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength)
{
    return crc_16(pchMessage,dwLength);
}

/*检验CRC16数据段*/
static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = crc_16(pchMessage, dwLength - 2);
    return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
}

/*检验数据帧头*/
static uint8_t protocol_heade_Check(protocol_rm_struct *pro, uint8_t *rx_buf)
{
    if (rx_buf[0] == PROTOCOL_CMD_ID)
    {
        pro->header.sof = rx_buf[0]; 
        //pro->header.data_length = (rx_buf[2] << 8) | rx_buf[1];
        //pro->header.crc_check = rx_buf[3];
        //pro->cmd_id = (rx_buf[5] << 8) | rx_buf[4];
        return 1;
    }
    return 0;
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void get_protocol_send_Vision_data(uint16_t send_id,        // 信号id
                            uint16_t flags_register, // 16位寄存器
                            Minipc_Send_s *tx_data,          // 待发送的float数据
                            uint8_t float_length,    // float的数据长度
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len)    // 待发送的数据帧长度
{
    static uint16_t crc16;
    static uint16_t data_len;

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
void get_protocol_info_vision(uint8_t *rx_buf, 
                           uint16_t *flags_register, 
                        Minipc_Recv_s *recv_data)
{
    static protocol_rm_struct pro;
    static uint16_t date_length;

    if (protocol_heade_Check(&pro, rx_buf)==1) 
    {
        date_length = OFFSET_BYTE + pro.header.data_length;
        //if (CRC16_Check_Sum(rx_buf, date_length)) {
            *flags_register = (rx_buf[7] << 8) | rx_buf[6];

            // 将接收到的数据复制到Minipc_Recv_s结构体中
            recv_data->Vision.header = rx_buf[0];
            memcpy(&recv_data->Vision.linear_velocity_x, &rx_buf[1], sizeof(float));
            memcpy(&recv_data->Vision.linear_velocity_y, &rx_buf[5], sizeof(float));
            memcpy(&recv_data->Vision.gimbal_mode, &rx_buf[9], sizeof(int32_t));
            memcpy(&recv_data->Vision.yaw, &rx_buf[13], sizeof(float));
            memcpy(&recv_data->Vision.pitch, &rx_buf[17], sizeof(float));
            memcpy(&recv_data->Vision.can_fire, &rx_buf[21], sizeof(int32_t));
            recv_data->Vision.checksum = (rx_buf[25] << 8) | rx_buf[26];
    }
}
