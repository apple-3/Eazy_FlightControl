#include "Com_Config.h"

void Com_Limit(float *inner, float Max, float Min) {
  if (*inner > Max) {
    *inner = Max;
  }
  if (*inner < Min) {
    *inner = Min;
  }
}
void Com_NRF_Access(Remote_data *data,uint8_t *Buf) {
  data->pit = (uint16_t)(Buf[6] | (Buf[7] << 8));
  data->rol = (uint16_t)(Buf[8] | (Buf[9] << 8));
  data->thr = (uint16_t)(Buf[10] | (Buf[11] << 8));
  data->yaw = (uint16_t)(Buf[12] | (Buf[13] << 8));
  data->shortdown = Buf[14];
  data->fix_high = Buf[15];
  data->calibrate = Buf[16];
}
// PWM限幅
int16_t clamp_speed(int16_t v) {
  if (v <= MIN_SPEED)
    return MIN_SPEED;
  if (v >= MAX_SPEED)
    return MAX_SPEED;
  return v;
}