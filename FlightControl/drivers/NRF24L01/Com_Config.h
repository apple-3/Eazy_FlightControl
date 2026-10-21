#ifndef COM_CONFIG_H
#define COM_CONFIG_H

#include <stdint.h>
#include "bsp_board_config.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct Joy_data {
  uint16_t pit;
  uint16_t rol;
  uint16_t yaw;
  uint16_t thr;
  uint8_t shortdown;
  uint8_t fix_high;
  uint8_t calibrate;
} Remote_data;
typedef struct Measure_Data {
  float roll, pitch, yaw;
} MPU_Data;
void Com_Limit(float *inner, float Max, float Min);
void Com_NRF_Access(Remote_data *data, uint8_t *Buf);
int16_t clamp_speed(int16_t v);

#ifdef __cplusplus
}
#endif

#endif // COM_CONFIG_H