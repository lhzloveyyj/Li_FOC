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
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */
#define ADC_VERF    3.3f
#define DCVBUS_R1   20
#define DCVBUS_R2   1
/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */
static led_device_t g_ledRun;
static float angle = 0.0f;
volatile uint16_t adcVbus = 0;
float dcVbus = 0.0f;

static float g_zeroOffset = 0.0f;
static float g_correctedElecAngle = 0.0f;

volatile uint8_t uabcEnabled = 0;
volatile uint8_t adcEnabled  = 0;
volatile uint8_t tabcEnabled = 0;
volatile uint8_t IabcEnabled = 0;
volatile uint8_t UAlpha_BetaEnabled = 0;
volatile uint8_t IAlpha_BetaEnabled = 0;
volatile uint8_t IQ_ID_Enabled = 0;
/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */
static float getVbus(void);
/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/* task handler */
TaskHandle_t comm_task_handle;
TaskHandle_t control_task_handle;
/* variables for task tcb and stack */
StackType_t comm_task_stack[256];
StackType_t control_task_stack[256];
StaticTask_t comm_task_buffer;
StaticTask_t control_task_buffer;

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
    tmr_interrupt_enable(TMR2, TMR_OVF_INT, TRUE);
    
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
    usart_interrupt_enable(USART3, USART_RDBF_INT, TRUE);
  /* add user code end comm_task_func 0 */

  /* add user code begin comm_task_func 2 */
    //加载参数
    foc_params_load(&g_readback);
    g_pMotor->pole_pairs = g_readback.pole_pairs;
    g_pMotor->dir        = g_readback.dir;
    g_pMotor->zeroOffset = g_readback.elec_offset;
    
    getAdoffset();
    
    float data[3] = {0.0f};
    led_init(&g_ledRun, "LED1", GPIOB, GPIO_PINS_4);
    
    uint8_t anglePrintingEnabled = 0;
  /* add user code end comm_task_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin comm_task_func 1 */
    if((g_commCmd != CMD_NONE) && (0 == led_get(&g_ledRun))){
        led_set(&g_ledRun, 1);
        vTaskDelay(50);
        led_set(&g_ledRun, 0);
    }
    
    angle = g_pMotor->mechanicalAngle;
    
    //数据回传上位机
    switch(g_commCmd)
    {
        case CMD_CONNECT_MOTOR:
            foc_params_load(&g_readback);
            g_pMotor->pole_pairs = g_readback.pole_pairs;
            g_pMotor->dir        = g_readback.dir;
            g_pMotor->zeroOffset = g_readback.elec_offset;
            data[0]   = (float)g_pMotor->pole_pairs;
            data[1]   = (float)g_pMotor->dir;
            data[2]   = g_pMotor->zeroOffset;
            USART3_SendPacket(CMD_CONNECT_MOTOR, &data[0], 3);
            g_commCmd = CMD_NONE;  
            break;
        
        case CMD_MECHANICALANGLE:
            anglePrintingEnabled = 1;
            g_commCmd = CMD_NONE;  
            break;
        
        case CMD_MECHANICALANGLE_CLOSE:
            anglePrintingEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETPAIRS:
            g_params.pole_pairs  = (int)g_cmdData;
            foc_params_save(&g_params);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETDIR:
            g_params.dir  = (int)g_cmdData;
            foc_params_save(&g_params);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_ZEROCALIBRATIO:
            led_set(&g_ledRun, 1);
            AngleInitZeroOffset(&g_zeroOffset, &g_correctedElecAngle);
            data[0] = g_zeroOffset;
            data[1] = g_correctedElecAngle;
            
            g_params.elec_offset  = g_zeroOffset;
            foc_params_save(&g_params);
            
            USART3_SendPacket(CMD_ZEROCALIBRATIO_OVER, &data[0], 2);
            led_set(&g_ledRun, 0);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_UABC:
            uabcEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_UABC_CLOSE:
            uabcEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETUQ:
            g_pMotor->uq = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_ADC:
            adcEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_ADC_CLOSE:
            adcEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_DCVBUS:
            data[0] = getVbus();
            USART3_SendPacket(CMD_DCVBUS, &data[0], 1);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_TABC:
            tabcEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_TABC_CLOSE:
            tabcEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IABC:
            IabcEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IABC_CLOSE:
            IabcEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_UALPHA_BETA:
            UAlpha_BetaEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_UALPHA_BETA_CLOSE:
            UAlpha_BetaEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IALPHA_BETA:
            IAlpha_BetaEnabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IALPHA_BETA_CLOSE:
            IAlpha_BetaEnabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IQ_ID:
            IQ_ID_Enabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_IQ_ID_CLOSE:
            IQ_ID_Enabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        default:
            break;
    }
    
    if(1 == anglePrintingEnabled){
        USART3_SendPacket(CMD_MECHANICALANGLE, &angle, 1); 
    }
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
    adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, FOC_ALL_DUTY * 0.98f);
  /* add user code end control_task_func 0 */

  /* add user code begin control_task_func 2 */

  /* add user code end control_task_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin control_task_func 1 */
      //printf("%f,%lf,%f\r\n",g_pMotor->Ia, g_pMotor->Ib, g_pMotor->Ic);
      //printf("%f,%lf\r\n",g_pMotor->iAlpha, g_pMotor->iBeta);
      //printf("%f,%lf\r\n",g_pMotor->id, g_pMotor->iq);
      //printf("%d\r\n", PSVpwm->sector);
    vTaskDelay(10);

  /* add user code end control_task_func 1 */
  }
}


/* add user code begin 2 */
static float getVbus(void)
{
    adc_flag_clear(ADC1, ADC_CCE_FLAG);

    adc_ordinary_software_trigger_enable(ADC1, TRUE);

    while(adc_flag_get(ADC1, ADC_CCE_FLAG) == RESET);

    adc_flag_clear(ADC1, ADC_CCE_FLAG);

    adcVbus = adc_ordinary_conversion_data_get(ADC1);
    
    float vbus = (adcVbus/4096.0f)* ADC_VERF * (DCVBUS_R1 + DCVBUS_R2)/DCVBUS_R2;
    
    return vbus;
}
/* add user code end 2 */

