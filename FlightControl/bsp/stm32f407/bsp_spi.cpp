#include "bsp_spi.hpp"
#include "bsp_board_config.h"
#include "stm32_hal_legacy.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"

uint8_t BSP_SPI_SwapByte(uint8_t Byte) {
  uint8_t receive = 0;
  if (HAL_SPI_TransmitReceive(BSP_SPI, &Byte, &receive, 1,
                              BSP_SPI_TIMEOUT_MS) != HAL_OK) {
    return 0;
  }

  return receive;
}