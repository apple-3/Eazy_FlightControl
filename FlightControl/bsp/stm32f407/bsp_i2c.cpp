#include "bsp_i2c.hpp"
#include "bsp_board_config.h"
#include "bsp_gpio.hpp"
#include "stm32_hal_legacy.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"

// 恢复限流：设备本来就不在线时，不要每次读写失败都去复位总线
#define IMU_I2C_RECOVER_MIN_INTERVAL_MS 100U

// 累计恢复次数，调试时可以直接看这个值
volatile uint32_t IMU_I2C_Recover_Count = 0;
static uint32_t imu_i2c_last_recover_tick = 0;

// I2C 总线卡死恢复。
// 现象：HAL_I2C_Mem_Read/Write 开头的"等 BUSY 标志复位"超时
//       （HAL 内部固定 I2C_TIMEOUT_BUSY_FLAG = 25ms），返回 HAL_BUSY，
//       且不会自己恢复，之后每次访问都同样失败。
// 原因通常是传输中途被打断（在 I2C 传输里下断点/复位），
// 或者从机(MPU6050)抓着 SDA 不放，MCU 误判为总线忙。
void BSP_I2C_Bus_Recover(void) {
  uint32_t now = HAL_GetTick();
  if ((now - imu_i2c_last_recover_tick) < IMU_I2C_RECOVER_MIN_INTERVAL_MS) {
    return;
  }
  imu_i2c_last_recover_tick = now;

  GPIO_InitTypeDef gpio = {0};

  // 1) 复位 I2C 外设，清掉锁死的 BUSY 状态
  HAL_I2C_DeInit(&hi2c1);

  // 2) 在 SCL 上打 9 个时钟，把卡在字节中间的从机顶出去
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);
  BSP::PINs::I2C1Scl.high();
  BSP::PINs::I2C1Sda.high();

  for (int i = 0; i < 9; i++) {
    BSP::PINs::I2C1Scl.low();
    for (volatile int d = 0; d < 200; d++) {
      __NOP();
    }
    BSP::PINs::I2C1Scl.high();
    for (volatile int d = 0; d < 200; d++) {
      __NOP();
    }
  }

  // 3) 补一个 STOP 条件：SCL 为高时 SDA 由低跳高
  BSP::PINs::I2C1Sda.low();
  for (volatile int d = 0; d < 200; d++) {
    __NOP();
  }
  BSP::PINs::I2C1Scl.high();
  for (volatile int d = 0; d < 200; d++) {
    __NOP();
  }
  BSP::PINs::I2C1Sda.high();

  // 4) 重新初始化外设，并把 PB6/PB7 配回复用功能
  MX_I2C1_Init();
  IMU_I2C_Recover_Count++;
}

// 读写统一封装：失败就尝试恢复总线，避免 BUSY 卡死后永久失效
int BSP_I2C_Write(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
  int ret = HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, data,
                              len, HAL_MAX_DELAY);
  if (ret != HAL_OK) {
    BSP_I2C_Bus_Recover();
  }
  return ret;
}

int BSP_I2C_Read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
  int ret = HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, data, len,
                             HAL_MAX_DELAY);
  if (ret != HAL_OK) {
    BSP_I2C_Bus_Recover();
  }
  return ret;
}