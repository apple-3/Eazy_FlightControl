#ifndef BSP_IF_H
#define BSP_IF_H
#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32_hal_legacy.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "main.h"

#define NRF24L01_CSN_SET HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_SET)
#define NRF24L01_CSN_RESET HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET)
#define NRF24L01_CE_SET HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET)
#define NRF24L01_CE_RESET HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET)

#ifdef __cplusplus
extern "C" {
#endif

void BSP_I2C_Bus_Recover(void);
int BSP_I2C_Write(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
int BSP_I2C_Read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // BSP_IF_H