#ifndef SHARED_DATA_H
#define SHARED_DATA_H
extern "C" {
#include "Com_Config.h"
#include "FreeRTOS.h"
#include "NRF24L01.hpp"
#include "semphr.h"

}
/*
 *结构体与状态枚举
 */
typedef enum {
  MOTOR_STATE_UNREMOTE = 0,
  MOTOR_STATE_IDLE,
  MOTOR_STATE_ARMED, // 你们说我给无人机加制导系统会不会被抓进去
  MOTOR_STATE_ACTIVE,
  MOTOR_STATE_CALI,
  MOTOR_STATE_SLOW,
} MotorState;
typedef struct {
  float roll, pitch, yaw; // MPU_Data_Real 的副本
  float gx, gy, gz;       // 三轴角速度的副本
} Attitude_Snapshot;
namespace SharedData {
inline MotorState motor_state = MOTOR_STATE_IDLE;
inline float distance = 0.0f;
// 数据杂项
inline uint8_t Buf[NRF24L01_Buf_Len] = {0};
inline uint8_t Send_Buf[NRF24L01_Buf_Len] = {'c', '8', 's', 'u', 'c', 'o'};
inline uint8_t Telecontrol = 0; // 遥控器接收状态
// 六向加速度
inline float ax, ay, az;
inline float gx, gy, gz;

// 串口调试
inline char msg[128];
inline Remote_data Remote_Control_Data = {500, 500, 500, 0,
                                          0,   0,   0}; // 遥控器接收数据
inline MPU_Data MPU_Data_Real = {0, 0, 0};
inline MPU_Data MPU_Data_Last = {0, 0, 0};
inline const float pitch_target_scale = 0.1f;     // 0..1000 -> -10..10 deg
inline const float roll_target_scale = 0.1f;      // 0..1000 -> -10..10 deg
inline const float yaw_rate_target_scale = 0.12f; // 0..1200 -> -60..60 deg
inline TickType_t last_remote_rx_tick = 0;
inline Attitude_Snapshot attitude = {0, 0, 0, 0, 0, 0};
inline SemaphoreHandle_t mutex = nullptr;
inline QueueHandle_t sem_handle;
inline void mutex_init() { mutex = xSemaphoreCreateMutex(); }
} // namespace SharedData

#endif // SHARED_DATA_H