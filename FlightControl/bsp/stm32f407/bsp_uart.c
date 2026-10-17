#include "bsp_board_config.h"
#include "bsp_if.h"

uint8_t Uart_Send_Motors(char *msg, int16_t lf, int16_t rf, int16_t lb,
                      int16_t rb) {
  int n =
      snprintf(msg, MSG_SIZE, ":%d,%d,%d,%d\n", (int)lf, (int)rf, (int)lb, (int)rb);
  if (n > 0) {
    HAL_UART_Transmit(BSP_USART, (uint8_t *)msg, (uint16_t)n,
                      UART_TX_TIMEOUT_MS);
    return UART_SEND_OK;
  }
  return 0;
}