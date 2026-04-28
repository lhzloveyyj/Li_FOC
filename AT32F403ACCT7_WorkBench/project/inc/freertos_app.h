/* add user code begin Header */
/**
  ******************************************************************************
  * File Name          : freertos_app.h
  * Description        : Code for FreeRTOS applications
  *
  * 本文件声明了三个任务：
  *   - comm_task：    通信 + 初始化
  *   - control_task： 速度/位置控制
  *   - Monitor_task：温度监控
  ******************************************************************************
  */
/* add user code end Header */

#ifndef FREERTOS_APP_H
#define FREERTOS_APP_H

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "led.h"
/* add user code end private includes */

/* exported types ------------------------------------------------------------*/
/* add user code begin exported types */
/* add user code end exported types */

/* exported constants --------------------------------------------------------*/
/* add user code begin exported constants */
/* add user code end exported constants */

/* exported macro ------------------------------------------------------------*/
/* add user code begin exported macro */
/* add user code end exported macro */

/* task handler 声明 */
extern TaskHandle_t comm_task_handle;
extern TaskHandle_t control_task_handle;
extern TaskHandle_t Monitor_task_handle;

/* task stack & TCB（静态分配） */
extern StackType_t comm_task_stack[256];
extern StackType_t control_task_stack[256];
extern StackType_t Monitor_task_stack[128];
extern StaticTask_t comm_task_buffer;
extern StaticTask_t control_task_buffer;
extern StaticTask_t Monitor_task_buffer;

/* task function 声明 */
void comm_task_func(void *pvParameters);
void control_task_func(void *pvParameters);
void Monitor_task_func(void *pvParameters);

/* binary semaphore 声明 */
extern SemaphoreHandle_t usart3_dma_tx_sem_handle;

/* FreeRTOS 初始化 API */
void freertos_task_create(void);
void freertos_semaphore_create(void);
void wk_freertos_init(void);

/* 获取运行指示 LED 对象 */
led_device_t *freertos_get_run_led(void);

#endif /* FREERTOS_APP_H */
