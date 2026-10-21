#include "shared_data.hpp"
#include "system_context.hpp"
#include "ultrasonic.h"
#define REMOTE_LINK_TIMEOUT_MS 500U
extern "C" void Status_Task(void *any) {
  // 启动时先把"上次收到数据"的时间推到超时窗口之外，
  // 即初始化时判定为失联，收到第一帧有效数据后才算连上
  SharedData::last_remote_rx_tick =
      xTaskGetTickCount() - pdMS_TO_TICKS(REMOTE_LINK_TIMEOUT_MS);
  while (1) {
    TickType_t now = xTaskGetTickCount();
    SharedData::Telecontrol =
        SystemContext::nrf.NRF24L01_receive(SharedData::Buf); // 获取遥控器数据
    if (SharedData::Telecontrol == NRF24L01_RX_OK) {
      SharedData::last_remote_rx_tick = now;
      Com_NRF_Access(&SharedData::Remote_Control_Data, SharedData::Buf);
      xSemaphoreGive(SharedData::sem_handle);
    }
    SharedData::distance = Ultrasonic_Getdistance(); // HCSR-04获取距离信息
    // 链路判定：超时窗口内收到过数据就算链路正常，
    // 这样遥控发包间隔大于轮询周期时不会把状态反复打回 UNREMOTE
    uint8_t link_ok = (now - SharedData::last_remote_rx_tick) <
                      pdMS_TO_TICKS(REMOTE_LINK_TIMEOUT_MS);

    // 状态机逻辑
    switch (SharedData::motor_state) {
    case MOTOR_STATE_IDLE: {
      if (link_ok == 0) {
        SharedData::motor_state = MOTOR_STATE_UNREMOTE; // 真失联
      } else if (SharedData::Remote_Control_Data.shortdown != 0) {
        SharedData::motor_state = MOTOR_STATE_ACTIVE; // 已解锁
      } else if (SharedData::Remote_Control_Data.calibrate == 1) {
        SharedData::motor_state = MOTOR_STATE_CALI;
      }
      break;
    }
    case MOTOR_STATE_ACTIVE: {
      if (link_ok == 0 || SharedData::Remote_Control_Data.shortdown == 0) {
        SharedData::motor_state = MOTOR_STATE_UNREMOTE;
      }
      break;
    }
    case MOTOR_STATE_CALI: {
      if (link_ok == 0) {
        SharedData::motor_state = MOTOR_STATE_UNREMOTE;
      }
      break;
    }
    case MOTOR_STATE_UNREMOTE: {
      if (link_ok != 0 && SharedData::Remote_Control_Data.shortdown != 0) {
        SharedData::motor_state = MOTOR_STATE_IDLE;
      }
      break;
    }
    case MOTOR_STATE_SLOW: {
      break;
    }
    case MOTOR_STATE_ARMED: {
      break;
    }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}