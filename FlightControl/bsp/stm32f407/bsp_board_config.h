#ifndef BSP_BOARD_IF_H
#define BSP_BOARD_IF_H
#include "tim.h"

#define Left_Front_TIM (&htim3)
#define Right_Behind_TIM (&htim3)
#define Left_Behind_TIM (&htim2)
#define Right_Front_TIM (&htim2)

#define Left_Front_CHANNEL (TIM_CHANNEL_2)
#define Right_Behind_CHANNEL (TIM_CHANNEL_3)
#define Left_Behind_CHANNEL (TIM_CHANNEL_1)
#define Right_Front_CHANNEL (TIM_CHANNEL_2)
#endif // BSP_BOARD_IF_H