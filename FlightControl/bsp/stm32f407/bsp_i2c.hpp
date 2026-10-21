#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_I2C_Bus_Recover(void);
int BSP_I2C_Write(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
int BSP_I2C_Read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // BSP_I2C_H