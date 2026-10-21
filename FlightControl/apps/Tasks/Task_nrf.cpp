#include "shared_data.hpp"
#include "system_context.hpp"
extern "C" void NRF_Send_Task(void *any) {
  BaseType_t res = 0;
  while (1) {
    res = xSemaphoreTake(SharedData::sem_handle, portMAX_DELAY);
    if (res == pdPASS) {
      SystemContext::nrf.NRF24L01_send(SharedData::Send_Buf);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}