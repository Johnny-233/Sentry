/**
 * @file    master_process.h
 * @brief   视觉/小电脑通信协议（由 Modules/master_machine/master_process.h 移植）
 * @note    结构体布局、帧头、缓冲区长度全部与原实现一致（逐字节兼容原有的视觉端）。
 *          bsp 换成对方的 Serial + Daemon；姿态从 application 的 AHRS 传入（不再直接读 QEKF_INS）。
 */
#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"
#include "ahrs.h"

#define Minipc_Recv_sIZE 36u // 新协议27字节,留余量
#define Minipc_Send_sIZE 64u // 53(整帧: 1 header + 50 payload + 2 CRC) + 余量

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	struct
	{
		uint8_t header;			  // 帧头，固定为0x5A
		float linear_velocity_x;  // 目标 x 轴线速度
		float linear_velocity_y;  // 目标 y 轴线速度
		int32_t gimbal_mode;	  // 小陀螺模式 (0=关, 非0=开)
		float yaw;				  // 需要云台转动的相对 yaw 角
		float pitch;			  // 需要云台转动的相对 pitch 角
		int32_t can_fire;		  // 开火信号 (0=无目标, 非0=可开火)
		uint16_t checksum;		  // CRC16 校验
	} Vision;

} __attribute__((packed)) Minipc_Recv_s;

typedef enum
{
	COLOR_BLUE = 1,
	COLOR_RED = 0,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2,
} Vision_Work_Mode_e;

typedef struct
{
	struct
	{
		uint8_t header;			   // 帧头，固定为0xA5 (由send函数写入tx_buf[0])
		uint8_t detect_color;	   // B: 敌方颜色
		uint8_t reserved;		   // B: 保留字节, 置0
		float roll;				   // f: roll角
		float pitch;			   // f: pitch角
		float yaw;				   // f: yaw角
		float vx;				   // f: x轴线速度 (未得到置0)
		float vy;				   // f: y轴线速度 (未得到置0)
		uint16_t self_sentry_hp;   // H: 己方哨兵血量
		uint16_t self_hero_hp;	   // H: 己方英雄血量
		uint16_t self_infantry_hp; // H: 己方步兵血量
		uint16_t remain_time;	   // H: 剩余时间
		uint16_t remain_bullet;	   // H: 剩余子弹数
		uint8_t match_progress;	   // B: 比赛进度
		uint8_t occupation;		   // B: 占领状态
		float bullet_speed;		   // f: 子弹速度
		/* ---- 受击检测（见 application/armor_hit），追加在末尾以保证前面偏移不变 ---- */
		uint8_t armor_id;			   // B: 最近一次受击的装甲板 ID (0=无)
		uint8_t armor_reason;		   // B: HP_deduction_reason (0=弹丸,1=撞击,4=超射速…)
		uint8_t armor_hit_cnt;		   // B: 累计受击次数
		uint8_t armor_hit_seq;		   // B: 事件序号(每次新受击+1, 供小电脑去重)
		float armor_angle_chassis;	   // f: 受击方向, 相对底盘正前方[度], 逆时针为正
		float armor_angle_world;	   // f: 叠加 yaw 的场地方位[度], 0~360
	} Vision;

} __attribute__((packed)) Minipc_Send_s;

#pragma pack()

/**
 * @brief 初始化和视觉(小电脑)的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 * @param ahrs   姿态来源(原来直接读 QEKF_INS; 现在由应用层把全局 AHRS 实例传进来)
 */
Minipc_Recv_s *Minipc_Init(UART_HandleTypeDef *handle, AHRS *ahrs);

/** 视觉帧计数(每次成功解帧 +1), 供调试采样判断新帧 */
extern volatile uint32_t g_vision_frame_cnt;

/** 取接收数据(初始化时返回的同一份) */
Minipc_Recv_s *Minipc_GetData(void);

/** 取发送数据缓冲(模块持有唯一那一份): 应用层只填自己负责的字段(如敌我颜色),
 *  姿态字段由 SendMinipcData() 每次发送前统一填; 1kHz 与 200Hz 两条路径共用它。 */
Minipc_Send_s *Minipc_GetSendData(void);

/** 视觉在线(daemon 判定) */
uint8_t Minipc_Online(void);

/** 强制重开一次串口接收(链路自愈, 见 Robot 的 DAEMON 任务) */
void Minipc_Recover(void);

/** 发送视觉数据(传 NULL 时用模块内部那份静态副本, 兼容 1kHz INS 路径)
 *  说明: 模块只持有**唯一一份**发送数据(Minipc_GetSendData()), 所以 1kHz(INS 任务)
 *        与 200Hz(RobotCMDTask) 两条路径发出去的内容是一致的, 都会带敌我颜色。 */
void SendMinipcData(Minipc_Send_s *send_data);

/** 设置发送帧里的敌方颜色 */
void VisionSetFlag(uint8_t color);

/** 把姿态(roll/pitch/yaw)填进模块内部那份发送数据 */
void VisionSetAltitude(void);

/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_Vision_data(uint16_t send_id,		// 信号id
								   uint16_t flags_register, // 16位寄存器
								   Minipc_Send_s *tx_data,	// 待发送的float数据
								   uint8_t float_length,	// float的数据长度
								   uint8_t *tx_buf,			// 待发送的数据帧
								   uint16_t *tx_buf_len);	// 待发送的数据帧长度

/* 解析视觉(小电脑)回传帧
 * @param len 本次实际接收字节数: 必须 >= 一帧长度, 否则直接丢弃(避免解析到上一包残留)
 */
void get_protocol_info_vision(uint8_t *rx_buf,
							  uint16_t len,
							  uint16_t *flags_register,
							  Minipc_Recv_s *recv_data);

#endif // !MASTER_PROCESS_H
