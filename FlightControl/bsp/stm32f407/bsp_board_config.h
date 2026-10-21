#ifndef BSP_BOARD_IF_H
#define BSP_BOARD_IF_H
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "i2c.h"
// 电机定时器
#define Left_Front_TIM (&htim3)
#define Right_Behind_TIM (&htim3)
#define Left_Behind_TIM (&htim2)
#define Right_Front_TIM (&htim2)
// 定时器通道
#define Left_Front_CHANNEL (TIM_CHANNEL_2)
#define Right_Behind_CHANNEL (TIM_CHANNEL_3)
#define Left_Behind_CHANNEL (TIM_CHANNEL_1)
#define Right_Front_CHANNEL (TIM_CHANNEL_2)
// 串口
#define BSP_USART (&huart1)
#define UART_TX_TIMEOUT_MS 20U
#define MSG_SIZE 128
#define UART_SEND_OK 1
// SPI
#define BSP_SPI (&hspi1)
#define BSP_SPI_TIMEOUT_MS 10U

// 其他
#define MAX_SPEED 2000
#define MIN_SPEED 1000

#endif // BSP_BOARD_IF_H