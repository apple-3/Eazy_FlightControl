#include "shared_data.hpp"
#include "system_context.hpp"

extern "C" void MPU_Task(void *any) {
  TickType_t last_wake = xTaskGetTickCount();
  while (1) {
    SystemContext::mpu.Read_Accel_Calibrated(SharedData::ax, SharedData::ay,
                                             SharedData::az);
    SystemContext::mpu.Read_Accel_Calibrated(SharedData::ax, SharedData::ay,
                                             SharedData::az);
    SystemContext::mpu.Read_Gyro_Calibrated(SharedData::gx, SharedData::gy,
                                            SharedData::gz);
    SharedData::MPU_Data_Real.roll =
        atan2f(SharedData::ay, SharedData::az) * 180.0f / 3.141592f;
    SharedData::MPU_Data_Real.pitch =
        atan2f(-SharedData::ax, sqrtf(SharedData::ay * SharedData::ay +
                                      SharedData::az * SharedData::az)) *
        180.0f / 3.141592f;

    float dt = (xTaskGetTickCount() - last_wake) * portTICK_PERIOD_MS / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;

    SharedData::MPU_Data_Real.yaw += SharedData::gz * dt;

    if (SharedData::MPU_Data_Real.yaw > 180.0f)
      SharedData::MPU_Data_Real.yaw -= 360.0f;
    if (SharedData::MPU_Data_Real.yaw < -180.0f)
      SharedData::MPU_Data_Real.yaw += 360.0f;

    SharedData::MPU_Data_Real.pitch =
        SystemContext::filter.First_order_Lowpass_Filter(
            SharedData::MPU_Data_Real.pitch, SharedData::MPU_Data_Last.pitch);

    SharedData::MPU_Data_Real.roll =
        SystemContext::filter.First_order_Lowpass_Filter(
            SharedData::MPU_Data_Real.roll, SharedData::MPU_Data_Last.roll);

    SharedData::MPU_Data_Real.yaw =
        SystemContext::filter.First_order_Lowpass_Filter(
            SharedData::MPU_Data_Real.yaw, SharedData::MPU_Data_Last.yaw);

    SharedData::MPU_Data_Last.pitch = SharedData::MPU_Data_Real.pitch;
    SharedData::MPU_Data_Last.roll = SharedData::MPU_Data_Real.roll;
    SharedData::MPU_Data_Last.yaw = SharedData::MPU_Data_Real.yaw;

    // 整块发布给Motor_Task避免它读到只更新了一半的姿态
    taskENTER_CRITICAL();
    SharedData::attitude.roll = SharedData::MPU_Data_Real.roll;
    SharedData::attitude.pitch = SharedData::MPU_Data_Real.pitch;
    SharedData::attitude.yaw = SharedData::MPU_Data_Real.yaw;
    SharedData::Send_Buf[14] =
        (uint8_t)((uint16_t)SharedData::attitude.roll & 0xFF);
    SharedData::Send_Buf[15] =
        ((uint8_t)(uint16_t)SharedData::attitude.roll >> 8 & 0xFF);
    SharedData::Send_Buf[16] =
        (uint8_t)((uint16_t)SharedData::attitude.pitch & 0xFF);
    SharedData::Send_Buf[17] =
        ((uint8_t)(uint16_t)SharedData::attitude.pitch >> 8 & 0xFF);
    SharedData::Send_Buf[18] =
        (uint8_t)((uint16_t)SharedData::attitude.yaw & 0xFF);
    SharedData::Send_Buf[19] =
        ((uint8_t)(uint16_t)SharedData::attitude.yaw >> 8 & 0xFF);
    SharedData::Send_Buf[21] = 1;
    SharedData::attitude.gx = SharedData::gx;
    SharedData::attitude.gy = SharedData::gy;
    SharedData::attitude.gz = SharedData::gz;
    taskEXIT_CRITICAL();

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(6));
  }
}