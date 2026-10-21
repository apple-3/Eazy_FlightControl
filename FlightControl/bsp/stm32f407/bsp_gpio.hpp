#ifndef GPIOPIN_H
#define GPIOPIN_H
#include "stm32_hal_legacy.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"

#include <cstdint>
namespace BSP {
class GPIOPin {
public:
  constexpr GPIOPin(GPIO_TypeDef *port, uint16_t pin)
      : _port(port), _pin(pin) {}
  inline void high() const { HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET); }
  inline void low() const { HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET); }
  inline bool read() const { return HAL_GPIO_ReadPin(_port, _pin); }
  inline void toggle() const { HAL_GPIO_TogglePin(_port, _pin); }

private:
  GPIO_TypeDef *const _port;
  const uint16_t _pin;
};
namespace PINs {
// 尚未添加的LED闪烁与蜂鸣器
//  extern const GPIOPin LED();
//  extern const GPIOPin Bee();
extern const GPIOPin NRFce;
extern const GPIOPin NRFcsn;
extern const GPIOPin I2C1Scl;
extern const GPIOPin I2C1Sda;
} // namespace PINs
} // namespace BSP

#endif // GPIOPIN_H