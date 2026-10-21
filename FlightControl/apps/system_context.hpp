#ifndef SYSTEM_CONTEXT_H
#define SYSTEM_CONTEXT_H

#include "MPU6050.hpp"
#include "Motor_Config.hpp"
#include "NRF24L01.hpp"
#include "PIDset.hpp"
#include "bsp_board_config.h"
#include "bsp_i2c.hpp"
#include "Com_Filter.hpp"

namespace SystemContext {
inline NRF24L01 nrf;
inline Filter filter;
inline MPU6050 mpu(BSP_I2C_Write, BSP_I2C_Read);
inline Motor Left_Front(Left_Front_TIM, Left_Front_CHANNEL);
inline Motor Right_Behind(Right_Behind_TIM, Right_Behind_CHANNEL);
inline Motor Left_Behind(Left_Behind_TIM, Left_Behind_CHANNEL);
inline Motor Right_Front(Right_Front_TIM, Right_Front_CHANNEL);
// PID还没调
inline PID<float> Pitch_Angle(1.0f, 0.05f, 0.05f);
inline PID<float> Roll_Angle(1.0f, 0.05f, 0.05f);
inline PID<float> Yaw_Angle(1.0f, 0, 0.05f);
inline PID<float> Pitch_Gyro(1.0f, 0.05f, 0.01f);
inline PID<float> Roll_Gyro(1.0f, 0.05f, 0.01f);
inline PID<float> Yaw_Gyro(1.0f, 0.05f, 0.01f);
} // namespace SystemContext

#endif // SYSTEM_CONTEXT_H