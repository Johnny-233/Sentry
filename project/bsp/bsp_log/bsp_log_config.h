#ifndef BSP_LOG_CONFIG_H
#define BSP_LOG_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_LOG_BUF_SIZE  160

#define BSP_LOG_DEFAULT_LEVEL  1

#define BSP_LOG_DEFAULT_MODULES  0xFFFF

#ifndef FAULT_LIST
  #define FAULT_LIST \
      X(0,  MOTOR_OFFLINE, "motor offline")   \
      X(1,  IMU_NODATA,    "IMU no data")     \
      X(2,  UART_TIMEOUT,  "UART timeout")    \
      X(3,  SPI_ERROR,     "SPI error")       \
      X(4,  CAN_TX_FAIL,   "CAN TX fail")     \
      X(5,  CAN_RX_FAIL,   "CAN RX fail")     \
      X(6,  HEAP_LOW,      "heap low")        \
      X(7,  STACK_OVERFLOW, "stack overflow")
#endif

#ifndef LOG_MODULE_LIST
  #define LOG_MODULE_LIST \
      X(0, SYS,   "SYS ")  \
      X(1, INS,   "INS ")  \
      X(2, MOTOR, "MOTR")  \
      X(3, GIMB,  "GIMB")  \
      X(4, CHAS,  "CHAS")  \
      X(5, SHOT,  "SHOT")  \
      X(6, COMM,  "COMM")  \
      X(7, SM,    "SM  ")
#endif

#ifdef __cplusplus
}
#endif

#endif
