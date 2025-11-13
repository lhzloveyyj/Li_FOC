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

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/* task handler */
TaskHandle_t comm_task_handle;
/* variables for task tcb and stack */
StackType_t comm_task_stack[256];
StaticTask_t comm_task_buffer;

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
  /* enter critical */
  taskENTER_CRITICAL();

  freertos_semaphore_create();
  freertos_task_create();
	
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
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
    usart_interrupt_enable(USART3, USART_RDBF_INT, TRUE);
  /* add user code end comm_task_func 0 */

  /* add user code begin comm_task_func 2 */
    float data = 1.2f;
  /* add user code end comm_task_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin comm_task_func 1 */
    switch(g_commCmd)
    {
        case CMD_CONNECT_MOTOR:
            USART3_SendPacket(CMD_CONNECT_MOTOR, &data, 1);
            g_commCmd = CMD_NONE;   
            break;
        default:
            break;
    }
    
    vTaskDelay(1);
  /* add user code end comm_task_func 1 */
  }
}


/* add user code begin 2 */

/* add user code end 2 */

