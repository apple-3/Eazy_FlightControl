#include "bsp_uart.hpp"
#include "shared_data.hpp"
#include "system_context.hpp"
extern "C" {
#include "FreeRTOS.h"
#include "bsp_board_config.h"
#include "task.h"
}

extern "C" void Motor_Task(void *any) {
  TickType_t last_wake = xTaskGetTickCount();
  while (1) {
    // 1. 安全地拿数据
    Attitude_Snapshot att;
    Remote_data r;
    xSemaphoreTake(SharedData::mutex, portMAX_DELAY);
    att = SharedData::attitude;
    r = SharedData::Remote_Control_Data;
    xSemaphoreGive(SharedData::mutex);
    float throttle = (float)r.thr;
    float base_speed = 1000.0f + throttle;
    float pitch_target =
        ((float)r.pit - 500.0f) * SharedData::pitch_target_scale;
    float roll_target = ((float)r.rol - 500.0f) * SharedData::roll_target_scale;
    float yaw_target =
        ((float)r.yaw - 500.0f) * SharedData::yaw_rate_target_scale;

    TickType_t now = xTaskGetTickCount();
    float dt =
        (now - SharedData::last_remote_rx_tick) * portTICK_PERIOD_MS / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    SharedData::last_remote_rx_tick = now;

    /*
    电机控制逻辑：
      NRF从机接收指令
      怠速：这里选百分之零是因为我害怕
      校准：2000为临时数据，后续准备在电机类加上校准逻辑，此处为临时校准
      激活：激活后通过计算PID串级角速度环按电机对应位置输出PWM信号
      控制器失联：通过内部的姿态计算选择对应的电机控制，比如：降落、大幅度矫正、或者如果有视觉就选没人的地方迫降吧（，当然还有可能还没飞就失联了
    */
    switch (SharedData::motor_state) {
    case MOTOR_STATE_IDLE: {
      SystemContext::Left_Front.setSpeed(1000);
      SystemContext::Left_Behind.setSpeed(1000);
      SystemContext::Right_Behind.setSpeed(1000);
      SystemContext::Right_Front.setSpeed(1000);
      // 每个状态都上报，避免状态切换时串口流出现空档
      Uart_Send_Motors(SharedData::msg, 1000, 1000, 1000, 1000);
      SharedData::Send_Buf[20] = MOTOR_STATE_IDLE;
      break;
    }
    case MOTOR_STATE_CALI: {
      SystemContext::Left_Front.setSpeed(2000);
      SystemContext::Right_Front.setSpeed(2000);
      SystemContext::Left_Behind.setSpeed(2000);
      SystemContext::Right_Behind.setSpeed(2000);
      Uart_Send_Motors(SharedData::msg, 2000, 2000, 2000, 2000);
      break;
    }
    case MOTOR_STATE_ACTIVE: {
      SystemContext::Pitch_Angle.Set_Setpoint(pitch_target);
      SystemContext::Roll_Angle.Set_Setpoint(roll_target);
      SystemContext::Yaw_Angle.Set_Setpoint(yaw_target);
      float pitch_output = SystemContext::Pitch_Angle.Update_Chain(
          SystemContext::Pitch_Gyro, att.pitch, att.gy, dt);
      float roll_output = SystemContext::Roll_Angle.Update_Chain(
          SystemContext::Roll_Gyro, att.roll, att.gx, dt);
      float yaw_output = SystemContext::Yaw_Angle.Update_Chain(
          SystemContext::Yaw_Gyro, att.yaw, att.gz, dt);

      int16_t speed_LB = clamp_speed(
          (int16_t)(base_speed - pitch_output + roll_output - yaw_output));
      int16_t speed_RB = clamp_speed(
          (int16_t)(base_speed - pitch_output - roll_output + yaw_output));
      int16_t speed_LF = clamp_speed(
          (int16_t)(base_speed + pitch_output + roll_output + yaw_output));
      int16_t speed_RF = clamp_speed(
          (int16_t)(base_speed + pitch_output - roll_output - yaw_output));

      SystemContext::Left_Front.setSpeed(speed_LF);
      SystemContext::Right_Front.setSpeed(speed_RF);
      SystemContext::Left_Behind.setSpeed(speed_LB);
      SystemContext::Right_Behind.setSpeed(speed_RB);
      SharedData::Send_Buf[6] = (uint8_t)(speed_LF & 0xFF);
      SharedData::Send_Buf[7] = (uint8_t)((speed_LF >> 8) & 0xFF);
      SharedData::Send_Buf[8] = (uint8_t)(speed_RF & 0xFF);
      SharedData::Send_Buf[9] = (uint8_t)((speed_RF >> 8) & 0xFF);
      SharedData::Send_Buf[10] = (uint8_t)(speed_LB & 0xFF);
      SharedData::Send_Buf[11] = (uint8_t)((speed_LB >> 8) & 0xFF);
      SharedData::Send_Buf[12] = (uint8_t)(speed_RB & 0xFF);
      SharedData::Send_Buf[13] = (uint8_t)((speed_RB >> 8) & 0xFF);
      SharedData::Send_Buf[20] =
          MOTOR_STATE_ACTIVE; // 通信正常且电机接收信号正常
      Uart_Send_Motors(SharedData::msg, speed_LF, speed_RF, speed_LB, speed_RB);
      break;
    }
    case MOTOR_STATE_UNREMOTE: {
      // 这里是蜂鸣器的逻辑，并且最好能计算当前姿态然后选择模式，大概（
      // 比如说单纯平飞失联，慢慢降落或者返程（通过GPS）
      // 再者就是大角度坠机 =。=那就要看看怎么让电机骚操作救回来了
      // 这里先默认暂停电机
      SystemContext::Left_Front.setSpeed(1000);
      SystemContext::Left_Behind.setSpeed(1000);
      SystemContext::Right_Behind.setSpeed(1000);
      SystemContext::Right_Front.setSpeed(1000);
      Uart_Send_Motors(SharedData::msg, 1000, 1000, 1000, 1000);
      SharedData::Send_Buf[20] = MOTOR_STATE_UNREMOTE;
      break;
    case MOTOR_STATE_SLOW: // 降落
      break;
    case MOTOR_STATE_ARMED: // 忠诚！！！
      break;
    }
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
  }
}