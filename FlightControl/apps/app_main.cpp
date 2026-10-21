#include "freertos_start.h"
#include "Com_Filter.hpp"
#include "MPU6050.hpp"
#include "Motor_Config.hpp"
#include "NRF24L01.hpp"
#include "PIDset.hpp"
#include "bsp_gpio.hpp"
#include "bsp_i2c.hpp"
#include "bsp_spi.hpp"
#include "bsp_tim.hpp"
#include "bsp_uart.hpp"
#include "shared_data.hpp"
#include "system_context.hpp"
extern "C" {
#include "Com_Config.h"
#include "FreeRTOS.h"
#include "bsp_board_config.h"
#include "main.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "task.h"
#include "ultrasonic.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
}
// 电机任务
#define Motor_Task_stack_size 512
#define Motor_Task_prioritize 2
TaskHandle_t Motor_Task_handle;
// 配置任务
#define Status_Task_stack_size 256
#define Status_Task_prioritize 3
TaskHandle_t Status_Task_handle;
// 陀螺仪任务
#define MPU_Task_stack_size 512
#define MPU_Task_prioritize 2
TaskHandle_t MPU_Task_handle;
#define NRF_Send_stack_size 128
#define NRF_Send_Task_prioritize 1
TaskHandle_t NRF_Send_handle;
QueueHandle_t sem_handle;

// 遥控链路超时(ms)：NRF_Receive 只是轮询NRF的 STATUS 寄存器，
// 取不到数据只说明"这一轮刚好没收到"，不代表失联。
// 只有超过这个时间都没收到有效数据，才判定链路断开


extern "C" void Motor_Task(void *any);
extern "C" void MPU_Task(void *any);
extern "C" void Status_Task(void *any);
extern "C" void NRF_Send_Task(void *any);

// 栈溢出钩子：命中说明有任务爆栈
// 溢出的任务句柄/名字分别是
// g_overflow_task/g_overflow_task_name，断在这就能看出是谁
TaskHandle_t g_overflow_task = 0;
volatile char *g_overflow_task_name = 0;
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  g_overflow_task = xTask;
  g_overflow_task_name = pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}

void Start_Rtos(void) {
  // 设备初始化
  SystemContext::mpu.init();
  SystemContext::Left_Front.init();
  SystemContext::Right_Front.init();
  SystemContext::Left_Behind.init();
  SystemContext::Right_Front.init();
  SystemContext::nrf.NRF24L01_Init();
  SystemContext::mpu.Zero_Offset();
  // 初始化01值信号量
  vSemaphoreCreateBinary(sem_handle);
  // 创建任务
  App_Rtos_Creat();
  // 启动任务调度进入循环
  vTaskStartScheduler();
}
void App_Rtos_Creat(void) {
  // 电机任务
  xTaskCreate(Motor_Task, "Motor_Task", Motor_Task_stack_size, NULL,
              Motor_Task_prioritize, &Motor_Task_handle);
  // 状态机与NRF接收任务
  xTaskCreate(Status_Task, "Status_Task", Status_Task_stack_size, NULL,
              Status_Task_prioritize, &Status_Task_handle);
  // 陀螺仪任务
  xTaskCreate(MPU_Task, "MPU_Task", MPU_Task_stack_size, NULL,
              MPU_Task_prioritize, &MPU_Task_handle);
  // NRF发送任务
  xTaskCreate(NRF_Send_Task, "NRF_Send_Task", NRF_Send_stack_size, NULL,
              NRF_Send_Task_prioritize, &NRF_Send_handle);
}


