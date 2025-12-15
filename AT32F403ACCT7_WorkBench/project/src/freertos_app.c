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
uint16_t adcVbus = 0;
uint16_t adcMostemp = 0;
float dcVbus = 0.0f;

static float g_zeroOffset = 0.0f;
static float g_correctedElecAngle = 0.0f;

volatile uint8_t anglePrintingEnabled = 0;
volatile uint8_t uabcEnabled = 0;
volatile uint8_t adcEnabled  = 0;
volatile uint8_t tabcEnabled = 0;
volatile uint8_t IabcEnabled = 0;
volatile uint8_t UAlpha_BetaEnabled = 0;
volatile uint8_t IAlpha_BetaEnabled = 0;
volatile uint8_t IQ_ID_Enabled = 0;
volatile uint8_t mostemp_Enabled = 0;
volatile uint8_t speed_Enabled = 0;
volatile uint8_t speedOut_Enabled = 0;
volatile uint8_t local_Enabled = 0;
volatile uint8_t localOut_Enabled = 0;

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
    
    float data[14] = {0.0f};
    led_init(&g_ledRun, "LED1", GPIOB, GPIO_PINS_4);
    
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
    
    //angle = g_pMotor->mechanicalAngle;
    
    //数据回传上位机
    switch(g_commCmd)
    {
        case CMD_CONNECT_MOTOR:
            foc_params_load(&g_readback);
            g_pMotor->pole_pairs = g_readback.pole_pairs;
            g_pMotor->dir        = g_readback.dir;
            g_pMotor->zeroOffset = g_readback.elec_offset;
            g_pMotor->speedDir   = g_readback.speeddir;
            data[0]   = (float)g_pMotor->pole_pairs;
            data[1]   = (float)g_pMotor->dir;
            data[2]   = g_pMotor->zeroOffset;
            data[3]   = g_pMotor->iqPID.kp;
            data[4]   = g_pMotor->iqPID.ki;
            data[5]   = getVbus();
            data[6]   = g_pMotor->speedDir;
            data[7]   = g_pMotor->speedPID.kp;
            data[8]   = g_pMotor->speedPID.ki;
            data[9]   = g_pMotor->positionPID.kp;
            data[10]  = g_pMotor->positionPID.kd;
            data[11]  = g_pMotor->iqPID.outMax;
            data[12]  = g_pMotor->speedPID.outMax;
            data[13]  = g_pMotor->positionPID.outMax;
            USART3_SendPacket(CMD_CONNECT_MOTOR, &data[0], 14);
          
            mostemp_Enabled = 1;
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
            foc_params_load(&g_params); 
            g_params.pole_pairs  = (int)g_cmdData;
            foc_params_save(&g_params);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETDIR:
            foc_params_load(&g_params); 
            g_params.dir  = (int)g_cmdData;
            foc_params_save(&g_params);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_ZEROCALIBRATIO:
            led_set(&g_ledRun, 1);
            AngleInitZeroOffset(&g_zeroOffset, &g_correctedElecAngle);
            data[0] = g_zeroOffset;
            data[1] = g_correctedElecAngle;
            
            foc_params_load(&g_params); 
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
        
        case CMD_SETIQ:
            g_pMotor->tariq = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETID:
            g_pMotor->tarid = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_OPEN_LOOP:
            g_pMotor->ctrolmode = FOC_OPEN_LOOP;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_CURRENT_LOOP:
            g_pMotor->ctrolmode = FPC_CURRENT_LOOP;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SPEED_LOOP:
            g_pMotor->ctrolmode = FOC_SPEED_LOOP;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_POSITION_LOOP :
            g_pMotor->ctrolmode = FOC_POSITION_LOOP;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETUD :
            g_pMotor->ud = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETIQPIDKP :
            g_pMotor->iqPID.kp = g_cmdData;
            g_pMotor->idPID.kp = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETIQPIDKI :
            g_pMotor->iqPID.ki = g_cmdData;
            g_pMotor->idPID.ki = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SPEED:
            speed_Enabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SPEED_CLODE:
            speed_Enabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETSPEEDDIR:
            foc_params_load(&g_params); 
            g_params.speeddir  = (int)g_cmdData;
            foc_params_save(&g_params);
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SPEEDOUT:
            speedOut_Enabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SPEEDOUT_CLOSE:
            speedOut_Enabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETSPEEDTAR:
            g_pMotor->tar_speed = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETSPEEDPIDKP:
            g_pMotor->speedPID.kp = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETSPEEDPIDKI:
            g_pMotor->speedPID.ki = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETLOCALTAR:
            g_pMotor->tarPosition = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_LOCAL:
            local_Enabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_LOCAL_CLOSE:
            local_Enabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_LOCALOUT:
            localOut_Enabled = 1;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_LOCALOUT_CLOSE:
            localOut_Enabled = 0;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETLOCALPIDKP:
            g_pMotor->positionPID.kp = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETLOCALPIDKD:
            g_pMotor->positionPID.kd = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETIQPIDOUT:
            g_pMotor->iqPID.outMax = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETSPEEDPIDOUT:
            g_pMotor->speedPID.outMax = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        case CMD_SETLOCALPIDOUT:
            g_pMotor->positionPID.outMax = g_cmdData;
            g_commCmd = CMD_NONE; 
            break;
        
        default:
            break;
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
        float temp = GetMosTemp();
          sendata[0] = temp;
        USART3_SendPacket(CMD_MOSTEMP, &sendata[0], 1); 
      }
      
     vTaskDelay(500);

  /* add user code end Monitor_task_func 1 */
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

