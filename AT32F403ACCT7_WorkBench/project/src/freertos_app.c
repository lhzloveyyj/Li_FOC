/* add user code begin Header */
/**
  ******************************************************************************
  * File Name          : freertos_app.c
  * Description        : Code for freertos applications
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
#include "foc.h"
#include "foc_config.h"
#include "filter.h"
#include "current_control.h"
#include "mostemp.h"
#include "speed_control.h"
#include "position_control.h"
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */
/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */
static led_device_t g_ledRun;
uint16_t adcMostemp = 0;
float dcVbus = 0.0f;

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/* task handler */
TaskHandle_t comm_task_handle;
TaskHandle_t control_task_handle;
TaskHandle_t Monitor_task_handle;
/* variables for task tcb and stack */
StackType_t comm_task_stack[256];
StackType_t control_task_stack[256];
StackType_t Monitor_task_stack[128];
StaticTask_t comm_task_buffer;
StaticTask_t control_task_buffer;
StaticTask_t Monitor_task_buffer;

/* binary semaphore handler */
SemaphoreHandle_t usart3_dma_tx_sem_handle;

/* Idle task control block and stack */
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];
static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];

static StaticTask_t idle_task_tcb;
static StaticTask_t timer_task_tcb;

/* External Idle and Timer task static memory allocation functions */
extern void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer, StackType_t ** ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
extern void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer, StackType_t ** ppxTimerTaskStackBuffer, uint32_t * pulTimerTaskStackSize );

/*
  vApplicationGetIdleTaskMemory gets called when configSUPPORT_STATIC_ALLOCATION
  equals to 1 and is required for static memory allocation support.
*/
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer, StackType_t ** ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &idle_task_tcb;
  *ppxIdleTaskStackBuffer = &idle_task_stack[0];
  *pulIdleTaskStackSize = (uint32_t)configMINIMAL_STACK_SIZE;
}
/*
  vApplicationGetTimerTaskMemory gets called when configSUPPORT_STATIC_ALLOCATION
  equals to 1 and is required for static memory allocation support.
*/
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer, StackType_t ** ppxTimerTaskStackBuffer, uint32_t * pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &timer_task_tcb;
  *ppxTimerTaskStackBuffer = &timer_task_stack[0];
  *pulTimerTaskStackSize = (uint32_t)configTIMER_TASK_STACK_DEPTH;
}

/* add user code begin 1 */

/* add user code end 1 */

/**
  * @brief  initializes all task.
  * @param  none
  * @retval none
  */
void freertos_task_create(void)
{
  /* create the comm_task task by static */
  comm_task_handle = xTaskCreateStatic(comm_task_func,
                                       "comm_task",
                                       256,
                                       NULL,
                                       0,
                                       comm_task_stack,
                                       &comm_task_buffer);

  /* create the control_task task by static */
  control_task_handle = xTaskCreateStatic(control_task_func,
                                       "control_task",
                                       256,
                                       NULL,
                                       0,
                                       control_task_stack,
                                       &control_task_buffer);

  /* create the Monitor_task task by static */
  Monitor_task_handle = xTaskCreateStatic(Monitor_task_func,
                                       "Monitor_task",
                                       128,
                                       NULL,
                                       0,
                                       Monitor_task_stack,
                                       &Monitor_task_buffer);
}

/**
  * @brief  initializes all semaphore.
  * @param  none
  * @retval none
  */
void freertos_semaphore_create(void)
{
  /* Create the usart3_dma_tx_sem */
  usart3_dma_tx_sem_handle = xSemaphoreCreateBinary();
}

/**
  * @brief  freertos init and begin run.
  * @param  none
  * @retval none
  */
void wk_freertos_init(void)
{
  /* add user code begin freertos_init 0 */

  /* add user code end freertos_init 0 */

  /* enter critical */
  taskENTER_CRITICAL();

  freertos_semaphore_create();
  freertos_task_create();
	
  /* add user code begin freertos_init 1 */

  /* add user code end freertos_init 1 */

  /* exit critical */
  taskEXIT_CRITICAL();

  /* start scheduler */
  vTaskStartScheduler();
}

/**
  * @brief comm_task function.
  * @param  none
  * @retval none
  */
void comm_task_func(void *pvParameters)
{
  /* add user code begin comm_task_func 0 */
    
    
  /* add user code end comm_task_func 0 */

  /* add user code begin comm_task_func 2 */  
    //加载参数
    vTaskDelay(100);
    
    
    foc_params_load(&g_readback);
    g_pMotor->pole_pairs = g_readback.pole_pairs;
    g_pMotor->dir        = g_readback.dir;
    g_pMotor->zeroOffset = g_readback.elec_offset;
    
    
    getAdoffset();
    
    LPF_Init(PM1_LPF);
    LPF_Speed_Init(PM1_LPF_Speed);
    
    //ID , IQ
    SetCurrentPIDTar(g_pMotor, 0.0f, 0.0f);
    SetSpeedPIDTar(g_pMotor, 0.0f);
    SetPositionPIDTar(g_pMotor, 0.0f);
    
    //KP, KI, KD, OUTMAX
    SetCurrentPIDParams(g_pMotor, 0.0005f, 0.5f, 0.0f, 12.0f);
    SetSpeedPIDParams(g_pMotor, 0.002f, 0.1f, 0.0f, 10.0f);
    SetPositionPIDParams(g_pMotor, 1.0f, 0.0f, 0.0001f, 400.0f);
    
    led_init(&g_ledRun, "LED1", GPIOB, GPIO_PINS_4);
    
  /* add user code end comm_task_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin comm_task_func 1 */
    Comm_CommandHandler();
    vTaskDelay(5);
  /* add user code end comm_task_func 1 */
  }
}


/**
  * @brief control_task function.
  * @param  none
  * @retval none
  */
void control_task_func(void *pvParameters)
{
  /* add user code begin control_task_func 0 */
   tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, FOC_ALL_DUTY * 0.99f);
    
  /* add user code end control_task_func 0 */

  /* add user code begin control_task_func 2 */
    int num = 0;
  /* add user code end control_task_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin control_task_func 1 */
        CalculateSpeed(g_pMotor, 0.002f, PM1_LPF_Speed);
        SpeedPIControl(g_pMotor);
      
        num ++;
        if(num == 5){
            CalculatePosition(g_pMotor);
            PositionPDControl(g_pMotor);
            num = 0;
        }
        vTaskDelay(2);
  /* add user code end control_task_func 1 */
  }
}


/**
  * @brief Monitor_task function.
  * @param  none
  * @retval none
  */
void Monitor_task_func(void *pvParameters)
{
  /* add user code begin Monitor_task_func 0 */
    
  /* add user code end Monitor_task_func 0 */

  /* add user code begin Monitor_task_func 2 */
    float sendata[2] = {0.0f};
  /* add user code end Monitor_task_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin Monitor_task_func 1 */
      if(mostemp_Enabled == 1){
          sendata[0] = GetMosTemp();
          sendata[1] = getVbus();
        USART3_SendPacket(CMD_MOSTEMP, &sendata[0], 2); 
      }
      
     vTaskDelay(500);

  /* add user code end Monitor_task_func 1 */
  }
}


/* add user code begin 2 */


/* add user code end 2 */

