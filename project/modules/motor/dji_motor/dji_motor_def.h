/**
 * @file DJIMotorDef.h
 * @author 乐
 * @brief 电机定义头文件
        用于定义电机的一些参数，例如ID、编码器线数等。用于电机ID映射，数值设置等。
 */

#ifndef DJI_MOTOR_DEF_H
#define DJI_MOTOR_DEF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define GM6020_VOLTAGE_CONTROL 1  //GM6020采用电压控制的开关

typedef enum
{
    DJIM_DIRECTION_NORMAL = 0,
    DJIM_DIRECTION_REVERT,
} DJIMotor_Direction;

typedef enum
{
    DJIMotor_3508 = 0,
    DJIMotor_2006,
    DJIMotor_6020,

    DJIMotor_Type_COUNT//电机型号总数
} DJIMotor_Type;

static const uint16_t tx_group_table[DJIMotor_Type_COUNT][2]=
{
    {1,0},//3508
    {1,0},//2006
    #if GM6020_VOLTAGE_CONTROL
    {0,2},//6020
    #else
    {3,4},//6020
    #endif
};//电机发送组号映射表：电机类型 电机ID → 控制组号

static const uint16_t group_tx_id[5]=
{
    0x1ff,  // 6020 的1-4号电机发送ID（电压控制） 3508和2006 的5-8号电机发送ID
    0x200,  // 3508和2006 的1-4号电机发送ID
    0x2ff,  // 6020 的5-7号电机发送ID（电压控制）
    0x1fe,  // 6020 的1-4号电机发送ID（电流控制）
    0x2fe,  // 6020 的5-7号电机发送ID（电流控制）
};//电机控制ID映射表：控制组号 → CAN发送ID

static const uint16_t rx_id_table[DJIMotor_Type_COUNT]=
{
    0x200,//3508
    0x200,//2006
    0x204,//6020
};//电机反馈ID表,实际接收ID=表中值+电机ID

static const uint8_t djim_current_range[DJIMotor_Type_COUNT] =
{
    20, //3508
    10, //2006
    #if GM6020_VOLTAGE_CONTROL
    1,  //6020           此处应乘上供电电压，以获取实际电压
    #else
    3  //6020
    #endif
};//电机电流范围,单位A,用于计算实际电流

static const uint16_t djim_current_cnt_range[DJIMotor_Type_COUNT] =
{
    16384, //3508
    10000, //2006
    #if GM6020_VOLTAGE_CONTROL
    25000,  //6020           此处表示电压的计数范围
    #else
    16384,  //6020
    #endif
};//电机电流的位数,用于计算实际电流

#define DJIM_TX_GROUP(type, id) tx_group_table[(type)][(uint8_t)(((id)-1)/4)] //电机发送组号
#define DJIM_TX_ID(group) group_tx_id[group]                                  //电机发送ID
#define DJIM_RX_ID(type, id) (rx_id_table[(type)]+id)  //电机反馈ID

#define DJIM_ENCODER_LINES 8192 //电机编码器的线数
#define DJIM_RPM_TO_DEGREE_PER_SEC 6.0f //转速rpm转换为角速度度/s的系数

#define DJIM_CURRENT(motor_type, raw_current) ((float)(raw_current) * (float)(djim_current_range[(motor_type)]) / (float)(djim_current_cnt_range[(motor_type)])) //电流计算公式 A
#define DJIM_VELOCITY(raw_rpm) ((float)(raw_rpm) * DJIM_RPM_TO_DEGREE_PER_SEC) //速度计算公                   度/s
#define DJIM_ANGLE(raw_ecd) ((float)(raw_ecd) * 360.0f / (float)(DJIM_ENCODER_LINES)) //角度计算公式        度

#define DJIM_CURRENT_TO_CNT(motor_type, current) (int16_t)((float)(current) * (float)(djim_current_cnt_range[(motor_type)]) / (float)(djim_current_range[(motor_type)])) //电流转换为发送数据的计数值

#define DJIM_REDUCTION_RATIO_M3508  19.0f
#define DJIM_REDUCTION_RATIO_M2006  36.0f


#ifdef __cplusplus
}
#endif

#endif //DJI_MOTOR_DEF_H