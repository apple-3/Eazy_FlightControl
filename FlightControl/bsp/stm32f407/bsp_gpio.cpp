#include "bsp_gpio.hpp"
#include "main.h"
namespace BSP::PINs {
// const GPIOPin LED();
// const GPIOPin Bee();
const GPIOPin NRFce(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin);
const GPIOPin NRFcsn(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin);
const GPIOPin I2C1Scl(GPIOB, GPIO_PIN_6);
const GPIOPin I2C1Sda(GPIOB, GPIO_PIN_7);
} // namespace BSP::PINs