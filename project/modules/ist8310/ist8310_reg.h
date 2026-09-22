#ifndef IST8310_REG_H
#define IST8310_REG_H

#ifdef __cplusplus
extern "C" {
#endif


/*---------- IST8310 寄存器地址 ----------*/

#define IST8310_REG_WHO_AM_I    0x00
#define IST8310_REG_DATA_OUT    0x03  // XOUT_L 起始地址, 连续读取 6 字节
#define IST8310_REG_CNTL1       0x0A
#define IST8310_REG_CNTL2       0x0B
#define IST8310_REG_STR         0x0C
#define IST8310_REG_TEMP        0x1C
#define IST8310_REG_CNTL3       0x0D

/*---------- 配置值 ----------*/

#define IST8310_WHO_AM_I_VALUE  0x10
#define IST8310_I2C_ADDR        0x0E  // 7-bit 地址

// CNTL1: 输出数据率
#define IST8310_ODR_SINGLE      0x00  // 单次测量
#define IST8310_ODR_10HZ        0x01
#define IST8310_ODR_20HZ        0x02
#define IST8310_ODR_50HZ        0x03
#define IST8310_ODR_100HZ       0x04
#define IST8310_ODR_200HZ       0x05

// CNTL2: 中断使能
#define IST8310_INT_ENABLE      0x08  // 使能中断, 低电平有效
#define IST8310_INT_DISABLE     0x00

// CNTL3: 软复位
#define IST8310_SOFT_RESET      0x01

/*---------- 灵敏度 ----------*/

#define IST8310_MAG_SEN         0.3f  // µT/LSB


#ifdef __cplusplus
}
#endif

#endif
