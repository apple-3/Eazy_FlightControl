#include "bsp_board_config.h"
#include "bsp_if.h"

uint8_t BSP_SPI_SwapByte(uint8_t Byte) {
  uint8_t receive = 0;
  if (HAL_SPI_TransmitReceive(BSP_SPI, &Byte, &receive, 1,
                              BSP_SPI_TIMEOUT_MS) != HAL_OK) {
    return 0;
  }

  return receive;
}