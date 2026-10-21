#ifndef BSP_IF_H
#define BSP_IF_H
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "stm32_hal_legacy.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
// I2C
void BSP_I2C_Bus_Recover(void);
int BSP_I2C_Write(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
int BSP_I2C_Read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
// SPI
uint8_t BSP_SPI_SwapByte(uint8_t Byte);
// GPIO

// UART
uint8_t Uart_Send_Motors(char *msg, int16_t lf, int16_t rf, int16_t lb,
                         int16_t rb);

// TIM

#ifdef __cplusplus
}
#endif

#endif // BSP_IF_H