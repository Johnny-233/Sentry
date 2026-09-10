#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
/* 一帧固定开销: 帧头5 + 命令码2 + 帧尾CRC16 2 (对应 LEN_HEADER/LEN_CMDID/LEN_TAIL) */
#define REFEREE_FRAME_MIN_LEN (LEN_HEADER + LEN_CMDID + LEN_TAIL)
/* 官方协议单帧数据段最大字节数, 用于给帧内 DataLength 做上限兜底(不能拿未校验的长度去算偏移) */
#define REFEREE_DATA_MAX_LEN 113u
/* 单次接收内最多连续解析的帧数, 防止异常数据把中断拖长 */
#define REFEREE_MAX_FRAMES 4u

static void JudgeReadData(uint8_t *buff, uint16_t len)
{
	if (buff == NULL) // 空数据包，则不作任何处理
		return;
	if (len > RE_RX_BUFFER_SIZE) // 防御: 长度不应超过缓冲区
		len = RE_RX_BUFFER_SIZE;

	// 逐帧解析(原来是递归 + 未校验长度跳转): 只有 CRC 全通过且长度合法才向后推进
	for (uint32_t offset = 0, frame = 0; frame < REFEREE_MAX_FRAMES; ++frame)
	{
		// 本次实际接收长度内剩余空间不足一帧
		if (offset + REFEREE_FRAME_MIN_LEN > len)
			return;
		// 判断帧头数据(0)是否为0xA5
		if (buff[offset + SOF] != REFEREE_SOF)
			return;

		// 帧内数据长度(小端16位), 先做范围校验再用于任何偏移计算
		uint16_t data_len = (uint16_t)buff[offset + DATA_LENGTH] |
							((uint16_t)buff[offset + DATA_LENGTH + 1] << 8);
		if (data_len > REFEREE_DATA_MAX_LEN)
		{
			LOGERROR("[ref] illegal DataLength [%u], stop parsing", (unsigned)data_len);
			return;
		}

		// 统计一帧数据长度(byte),用于CRC16校验
		uint16_t judge_length = (uint16_t)(data_len + REFEREE_FRAME_MIN_LEN);
		if (offset + judge_length > len)
			return;

		// 帧头CRC8校验 + 帧尾CRC16校验, 通过后才解析
		if (Verify_CRC8_Check_Sum(&buff[offset], LEN_HEADER) != TRUE)
			return;
		if (Verify_CRC16_Check_Sum(&buff[offset], judge_length) != TRUE)
			return;

		// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
		memcpy(&referee_info.FrameHeader, &buff[offset], LEN_HEADER);
		// 2个8位拼成16位int
		referee_info.CmdID = (buff[offset + CMD_ID_Offset + 1] << 8 | buff[offset + CMD_ID_Offset]);
		// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
		// 第8个字节开始才是数据 data=7
		switch (referee_info.CmdID)
		{
		case ID_game_state: // 0x0001
			memcpy(&referee_info.GameState, &buff[offset + DATA_Offset], LEN_game_state);
			break;
		case ID_game_result: // 0x0002
			memcpy(&referee_info.GameResult, &buff[offset + DATA_Offset], LEN_game_result);
			break;
		case ID_game_robot_survivors: // 0x0003
			memcpy(&referee_info.GameRobotHP, &buff[offset + DATA_Offset], LEN_game_robot_HP);
			break;
		case ID_event_data: // 0x0101
			memcpy(&referee_info.EventData, &buff[offset + DATA_Offset], LEN_event_data);
			break;
		case ID_referee_warning://0x0104
			memcpy(&referee_info.RefereeWarning, &buff[offset + DATA_Offset], LEN_referee_warning); // 原来错用了 0x0101 的 LEN_event_data(4B), 目标结构体只有 3B
			break;
		case ID_game_robot_state: // 0x0201
			memcpy(&referee_info.GameRobotState, &buff[offset + DATA_Offset], LEN_game_robot_state);
			break;
		case ID_power_heat_data: // 0x0202
			memcpy(&referee_info.PowerHeatData, &buff[offset + DATA_Offset], LEN_power_heat_data);
			break;
		case ID_game_robot_pos: // 0x0203
			memcpy(&referee_info.GameRobotPos, &buff[offset + DATA_Offset], LEN_game_robot_pos);
			break;
		case ID_buff_musk: // 0x0204
			memcpy(&referee_info.BuffMusk, &buff[offset + DATA_Offset], LEN_buff_musk);
			break;

		case ID_robot_hurt: // 0x0206
			memcpy(&referee_info.RobotHurt, &buff[offset + DATA_Offset], LEN_robot_hurt);
			break;
		case ID_shoot_data: // 0x0207
			memcpy(&referee_info.ShootData, &buff[offset + DATA_Offset], LEN_shoot_data);
			break;
		case ID_projectile_allowance: // 0x0208
			memcpy(&referee_info.ProjectileAllowance, &buff[offset + DATA_Offset], LEN_projectile_allowance);
			break;
		case ID_student_interactive: // 0x0301   syhtodo接收代码未测试
			memcpy(&referee_info.ReceiveData, &buff[offset + DATA_Offset], LEN_receive_data);
			break;
		}

		// 推进到下一帧(原来是递归调用): 同一接收缓冲里若还粘连着下一帧, 由 for 循环继续解析
		offset += judge_length;
	}
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback(USARTInstance *instance, uint16_t len)
{
	DaemonReload(referee_daemon);
	JudgeReadData(instance->recv_buff, len); // 传入本次实际接收长度, 解析器据此做边界校验
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
	LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(115);
}
