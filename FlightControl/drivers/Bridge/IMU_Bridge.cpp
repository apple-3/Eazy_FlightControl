#include "IMU_Bridge.h"
#include "MPU6050.hpp"
#include "bsp_if.h"

mpu6050_write_reg_t MPU6050_Write_Reg = BSP_I2C_Write;
mpu6050_read_reg_t MPU6050_Read_Reg = BSP_I2C_Read;
MPU6050 mpu(MPU6050_Write_Reg, MPU6050_Read_Reg);
void MPU_Init() { mpu.init(); }
void MPU_Zero_Offset() { mpu.Zero_Offset(); }
void MPU_Read_Accel_Float(float *ax, float *ay, float *az) {
  if (ax && ay && az) {
    mpu.MPU6050_Read_Accel_Float(*ax, *ay, *az);
  }
}
void MPU_Read_Gyro_Float(float *gx, float *gy, float *gz) {
  if (gx && gy && gz) {
    mpu.MPU6050_Read_Gyro_Float(*gx, *gy, *gz);
  }
}
void MPU_Zero_Offset_Accel(float *ax, float *ay, float *az) {
  mpu.Read_Accel_Calibrated(*ax, *ay, *az);
}
void MPU_Zero_Offset_Gyro(float *gx, float *gy, float *gz) {
  mpu.Read_Gyro_Calibrated(*gx, *gy, *gz);
}
