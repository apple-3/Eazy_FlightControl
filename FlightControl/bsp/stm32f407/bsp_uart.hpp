#ifndef BSP_UART_H
#define BSP_UART_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t Uart_Send_Motors(char *msg, int16_t lf, int16_t rf, int16_t lb,
                         int16_t rb);

#ifdef __cplusplus
}
#endif

#endif // BSP_UART_H