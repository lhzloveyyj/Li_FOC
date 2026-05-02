/* add user code begin Header */
/**
  ******************************************************************************
  * File Name          : freertos_app.c
  * Description        : Code for FreeRTOS applications
  *
  * FreeRTOS 任务说明：
  *   - comm_task：   通信处理任务，处理上位机命令和参数初始化
  *   - control_task：控制任务，执行速度/位置控制运算
  *   - Monitor_task：监控任务（仅用于 mostemp 温度上传）
  ******************************************************************************
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "freertos_app.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include <stdio.h>
#include "usart3.h"
#include "protocol.h"
#include "led.h"
#include "mt6701.h"
#include "flash_ops.h"
#include "FOC.h"
#include "foc_config.h"
#include "filter.h"
#include "current_control.h"
#include "mostemp.h"
#include "speed_control.h"
#include "position_control.h"
#include "smo_observer.h"
/* add user code end private includes */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */
static led_device_t g_ledRun;
uint16_t adcvbus = 0;
float dcVbus = 0.0f;
/* add user code end private variables */

/* task handle */
TaskHandle_t comm_task_handle;
TaskHandle_t control_task_handle;
TaskHandle_t Monitor_task_handle;
/* variables for task TCB and stack (static allocation) */
StackType_t comm_task_stack[256];
StackType_t control_task_stack[256];
StackType_t Monitor_task_stack[128];
StaticTask_t comm_task_buffer;
StaticTask_t control_task_buffer;
StaticTask_t Monitor_task_buffer;

/* binary semaphore handle */
SemaphoreHandle_t usart3_dma_tx_sem_handle;

/* Idle & Timer task static allocation for FreeRTOS */
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];
static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];
static StaticTask_t idle_task_tcb;
static StaticTask_t timer_task_tcb;

extern void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);
extern void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
    StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &idle_task_tcb;
    *ppxIdleTaskStackBuffer = &idle_task_stack[0];
    *pulIdleTaskStackSize   = (uint32_t)configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
    StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &timer_task_tcb;
    *ppxTimerTaskStackBuffer = &timer_task_stack[0];
    *pulTimerTaskStackSize   = (uint32_t)configTIMER_TASK_STACK_DEPTH;
}

/* =================== FreeRTOS 初始化接口 =================== */

/**
  * @brief  Create all FreeRTOS tasks.
  */
void freertos_task_create(void)
{
    comm_task_handle = xTaskCreateStatic(comm_task_func, "comm_task", 256, NULL, 0,
                                         comm_task_stack, &comm_task_buffer);
    control_task_handle = xTaskCreateStatic(control_task_func, "control_task", 256, NULL, 0,
                                            control_task_stack, &control_task_buffer);
    Monitor_task_handle = xTaskCreateStatic(Monitor_task_func, "Monitor_task", 128, NULL, 0,
                                            Monitor_task_stack, &Monitor_task_buffer);
}

/**
  * @brief  Create all FreeRTOS semaphores.
  */
void freertos_semaphore_create(void)
{
    usart3_dma_tx_sem_handle = xSemaphoreCreateBinary();
}

/**
  * @brief  FreeRTOS init entry: create tasks/semaphores and start scheduler.
  */
void wk_freertos_init(void)
{
    taskENTER_CRITICAL();
    freertos_semaphore_create();
    freertos_task_create();
    taskEXIT_CRITICAL();
    vTaskStartScheduler();
}

led_device_t *freertos_get_run_led(void)
{
    return &g_ledRun;
}

/******************************************************************************
 * 任务函数：comm_task_func
 *
 * 功能描述：通信任务（系统初始化 + 命令处理）。
 *           在进入循环前完成一次性的初始化工作：
 *             - 从 Flash 加载电机参数
 *             - ADC 偏置校准
 *             - 电流/速度滤波器初始化
 *             - 设置 PID 参数默认值
 *             - SMO 观测器初始化
 *             - LED 初始化
 *           进入循环后轮询处理上位机命令。
 ******************************************************************************/
void comm_task_func(void *pvParameters)
{
    (void)pvParameters;
  /* add user code begin comm_task_func 2 */
    vTaskDelay(100);

    /* ---- 加载 Flash 参数 ---- */
    foc_params_load(&g_readback);
    g_pMotor->pole_pairs = g_readback.pole_pairs;
    g_pMotor->dir        = g_readback.dir;
    g_pMotor->zeroOffset = g_readback.elec_offset;
    g_pMotor->speedDir   = g_readback.speeddir;
    g_pMotor->rs         = g_readback.rs;
    g_pMotor->lq         = g_readback.lq;
    g_pMotor->ld         = g_readback.ld;

    /* ---- ADC 偏置 ---- */
    getAdoffset();

    /* ---- 滤波器初始化 ---- */
    LPF_Init(PM1_LPF);
    LPF_Speed_Init(PM1_LPF_Speed);

    /* ---- PID 目标清零 ---- */
    SetCurrentPIDTar(g_pMotor, 0.0f, 0.0f);
    SetSpeedPIDTar(g_pMotor, 0.0f);
    SetPositionPIDTar(g_pMotor, 0.0f);

    /* ---- PID 参数 ---- */
    SetCurrentPIDParams(g_pMotor, 0.0005f, 0.5f, 0.0f, 12.0f);
    SetSpeedPIDParams(g_pMotor, 0.002f, 0.1f, 0.0f, 10.0f);
    SetPositionPIDParams(g_pMotor, 1.0f, 0.0f, 0.0001f, 400.0f);

    /* ---- SMO 初始化（使用 PLL 版本） ---- */
    const SmoObserverConfig smoConfig = {
        .rs = g_pMotor->rs,
        .ls = (g_pMotor->lq + g_pMotor->ld) * 0.5f,
        .ts = FOC_SMO_TS,
        .k_slide = FOC_SMO_K_SLIDE,
        .e_lpf_alpha = FOC_SMO_E_LPF_ALPHA,
        .pll = {
            .kp = FOC_SMO_PLL_KP,
            .ki = FOC_SMO_PLL_KI,
            .ts = FOC_SMO_TS,
        },
    };
    SMO_Init(&g_smoObserver, &smoConfig);

    /* ---- LED ---- */
    led_init(&g_ledRun, "LED1", GPIOB, GPIO_PINS_4);
  /* add user code end comm_task_func 2 */

  while (1) {
      Comm_CommandHandler();
      vTaskDelay(5);
  }
}

/******************************************************************************
 * 任务函数：control_task_func
 *
 * 功能描述：控制任务。
 *           周期 2ms 执行一次：
 *             - 速度计算（差分法 + 低通滤波）
 *             - 速度 PI 控制
 *             - 位置计算（每 5 个周期一次）
 *             - 位置 PD 控制（每 5 个周期一次）
 *
 * 注意：电流环的控制在 ADC 中断中实时执行，不受此任务周期限制。
 ******************************************************************************/
void control_task_func(void *pvParameters)
{
    (void)pvParameters;
  /* add user code begin control_task_func 0 */
    /* 设置 TIM1 通道 4 占空比（用于 PWM 输出 brake 或附加输出） */
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, (uint32_t)(FOC_ALL_DUTY * 0.99f));
  /* add user code end control_task_func 0 */

  /* add user code begin control_task_func 2 */
    int num = 0;
  /* add user code end control_task_func 2 */

  while (1) {
      CalculateSpeed(g_pMotor, 0.002f, PM1_LPF_Speed);

      if (g_pMotor->ctrolmode == FOC_SPEED_LOOP
          || g_pMotor->ctrolmode == FOC_POSITION_LOOP) {
          SpeedPIControl(g_pMotor);
      }

      /* 位置环以较低频率运行（每 5 个周期 = 10ms 执行一次） */
      if (g_pMotor->ctrolmode == FOC_POSITION_LOOP) {
          num++;
          if (num == 5) {
              CalculatePosition(g_pMotor);
              PositionPDControl(g_pMotor);
              num = 0;
          }
      }
      vTaskDelay(2);
  }
}

/******************************************************************************
 * 任务函数：Monitor_task_func
 *
 * 功能描述：监控任务。
 *           每 500ms 上报一次 MOS 管温度。
 ******************************************************************************/
void Monitor_task_func(void *pvParameters)
{
    (void)pvParameters;
  /* add user code begin Monitor_task_func 2 */
    float sendata[1] = {0.0f};
  /* add user code end Monitor_task_func 2 */

  while (1) {
      if (mostemp_Enabled == 1) {
          sendata[0] = GetMosTemp();
          USART3_SendPacket(CMD_MOSTEMP, &sendata[0], 1);
      }
      vTaskDelay(500);
  }
}
