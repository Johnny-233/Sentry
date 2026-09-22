#ifndef __SEASKY_PROTOCOL_H
#define __SEASKY_PROTOCOL_H

/**
 * @file    seasky_protocol.h
 * @brief   视觉/小电脑 seasky 协议（由 Modules/master_machine/seasky_protocol.h/.c 移植）
 * @note    协议逐字节照搬；只把 bsp 从旧 USARTInstance 换成对方的 Serial/Daemon。
 *          CRC 用**本仓原本的实现**（见 seasky_protocol.cpp 顶部说明）：
 *            crc_8  : SHT75 表, 多项式 0x31(反射), 初值 0x00
 *            crc_16 : 反射, 多项式 0xA001(Modbus), 初值 0xFFFF
 *          —— 与对方 crc_rm.h 的 CRC8(初值 0xFF)/CRC16(CCITT 0x8408) **不是一回事**，不能混用。
 */

#include <stdio.h>
#include <stdint.h>

#define PROTOCOL_CMD_ID 0x5A
#define SEND_VISION_ID 0xA5

#define OFFSET_BYTE 8 // 出数据段外，其他部分所占字节数

typedef struct
{
	struct
	{
		uint8_t sof;
		uint16_t data_length;
		uint8_t crc_check; // 帧头CRC校验
	} header;			   // 数据帧头
	uint16_t cmd_id;	   // 数据ID
	uint16_t frame_tail;   // 帧尾CRC校验
} protocol_rm_struct;

#endif // !__SEASKY_PROTOCOL_H
