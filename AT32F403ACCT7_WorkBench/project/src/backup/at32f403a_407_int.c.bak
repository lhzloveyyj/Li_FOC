/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f403a_407_int.c
  * @brief    main interrupt service routines.
  **************************************************************************
  */
/* add user code end Header */

/* includes ------------------------------------------------------------------*/
#include "at32f403a_407_int.h"
#include "freertos_app.h"


/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "usart3.h"
#include "freertos_app.h"
#include "FOC.h"
#include "foc_config.h"
#include "protocol.h"
#include "mostemp.h"
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */
/** 遥测数据缓存 */
static float focData[32] = {0.0f};
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

/* external variables ---------------------------------------------------------*/
/* add user code begin external variables */

/* add user code end external variables */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
  /* add user code begin NonMaskableInt_IRQ 0 */

  /* add user code end NonMaskableInt_IRQ 0 */

  /* add user code begin NonMaskableInt_IRQ 1 */

  /* add user code end NonMaskableInt_IRQ 1 */
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  /* add user code begin HardFault_IRQ 0 */

  /* add user code end HardFault_IRQ 0 */
  /* go to infinite loop when hard fault exception occurs */
  while (1)
  {
    /* add user code begin W1_HardFault_IRQ 0 */

    /* add user code end W1_HardFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  /* add user code begin MemoryManagement_IRQ 0 */

  /* add user code end MemoryManagement_IRQ 0 */
  /* go to infinite loop when memory manage exception occurs */
  while (1)
  {
    /* add user code begin W1_MemoryManagement_IRQ 0 */

    /* add user code end W1_MemoryManagement_IRQ 0 */
  }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  /* add user code begin BusFault_IRQ 0 */

  /* add user code end BusFault_IRQ 0 */
  /* go to infinite loop when bus fault exception occurs */
  while (1)
  {
    /* add user code begin W1_BusFault_IRQ 0 */

    /* add user code end W1_BusFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  /* add user code begin UsageFault_IRQ 0 */

  /* add user code end UsageFault_IRQ 0 */
  /* go to infinite loop when usage fault exception occurs */
  while (1)
  {
    /* add user code begin W1_UsageFault_IRQ 0 */

    /* add user code end W1_UsageFault_IRQ 0 */
  }
}


/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
  /* add user code begin DebugMonitor_IRQ 0 */

  /* add user code end DebugMonitor_IRQ 0 */
  /* add user code begin DebugMonitor_IRQ 1 */

  /* add user code end DebugMonitor_IRQ 1 */
}

extern void xPortSysTickHandler(void);

/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
void SysTick_Handler(void)
{
  /* add user code begin SysTick_IRQ 0 */

  /* add user code end SysTick_IRQ 0 */

#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */

  /* add user code begin SysTick_IRQ 1 */

  /* add user code end SysTick_IRQ 1 */
}

/**
  * @brief  this function handles DMA1 Channel 1 handler.
  * @param  none
  * @retval none
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* add user code begin DMA1_Channel1_IRQ 0 */
    if(dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
        dma_flag_clear(DMA1_FDT1_FLAG);
        dma_channel_enable(DMA1_CHANNEL1, FALSE);
        usart3_tx_dma_status = 1;
    }
  /* add user code end DMA1_Channel1_IRQ 0 */

  /* add user code begin DMA1_Channel1_IRQ 1 */

  /* add user code end DMA1_Channel1_IRQ 1 */
}

/**
  * @brief  this function handles ADC1 & ADC2 handler.
  * @param  none
  * @retval none
  */
void ADC1_2_IRQHandler(void)
{
  /* add user code begin ADC1_2_IRQ 0 */
    if (adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET) {
        /* 读取三相电流 ADC 值 */
        g_motorAdValues[0] = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_1);
        g_motorAdValues[1] = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_2);
        g_motorAdValues[2] = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_3);
        adcvbus = adc_preempt_conversion_data_get(ADC1, ADC_PREEMPT_CHANNEL_4);

        /* FOC 控制主循环 */
        FocContorl(g_pMotor, PSVpwm);

        adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    }
  /* add user code end ADC1_2_IRQ 0 */

  if(adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET)
  {
    /* add user code begin ADC1_ADC_PCCE_FLAG */
    /* clear flag */
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    /* add user code end ADC1_ADC_PCCE_FLAG */ 
  }

  /* add user code begin ADC1_2_IRQ 1 */

  /* add user code end ADC1_2_IRQ 1 */
}

/**
  * @brief  this function handles TMR2 handler.
  * @param  none
  * @retval none
  */
void TMR2_GLOBAL_IRQHandler(void)
{
  /* add user code begin TMR2_GLOBAL_IRQ 0 */
    uint32_t mask = 0;
    int16_t *v = (int16_t *)(uart3_tx_buffer + 7); /* 跳过 header+cmd+len+mask(4B) */
    int n = 0;

    #define PACK(bit, scale, vals) do { mask |= (1u<<(bit)); vals n++; } while(0)

    /* bit 0: angle x2  scale=1000 */
    if (anglePrintingEnabled) PACK(TELEM_BIT_ANGLE, 1000, {
        *v++ = (int16_t)(g_pMotor->mechanicalAngle * 1000.0f);
        *v++ = (int16_t)(g_pMotor->sensoredCorrectedAngle * 1000.0f); n++;
    });

    /* bit 1: speed  scale=10 */
    if (speed_Enabled) PACK(TELEM_BIT_SPEED, 10, {
        *v++ = (int16_t)(g_pMotor->speed * 10.0f);
    });

    /* bit 2: speedOut  scale=100 */
    if (speedOut_Enabled) PACK(TELEM_BIT_SPEEDOUT, 100, {
        *v++ = (int16_t)(g_pMotor->speedPID.out * 100.0f);
    });

    /* bit 3: Iabc x3  scale=100 */
    if (IabcEnabled) PACK(TELEM_BIT_IABC, 100, {
        *v++ = (int16_t)(g_pMotor->Ia * 100.0f);
        *v++ = (int16_t)(g_pMotor->Ib * 100.0f);
        *v++ = (int16_t)(g_pMotor->Ic * 100.0f); n++; n++;
    });

    /* bit 4: IqId x2  scale=100 */
    if (IQ_ID_Enabled) PACK(TELEM_BIT_IQID, 100, {
        *v++ = (int16_t)(g_pMotor->iq * 100.0f);
        *v++ = (int16_t)(g_pMotor->id * 100.0f); n++;
    });

    /* bit 5: UalphaBeta x2  scale=100 */
    if (UAlpha_BetaEnabled) PACK(TELEM_BIT_UALPHABETA, 100, {
        *v++ = (int16_t)(g_pMotor->uAlpha * 100.0f);
        *v++ = (int16_t)(g_pMotor->uBeta * 100.0f); n++;
    });

    /* bit 6: Uabc x3  scale=100 */
    if (uabcEnabled) PACK(TELEM_BIT_UABC, 100, {
        *v++ = (int16_t)(g_pMotor->ua * 100.0f);
        *v++ = (int16_t)(g_pMotor->ub * 100.0f);
        *v++ = (int16_t)(g_pMotor->uc * 100.0f); n++; n++;
    });

    /* bit 7: ADC x3  scale=1 */
    if (adcEnabled) PACK(TELEM_BIT_ADC, 1, {
        *v++ = (int16_t)(g_motorAdValues[0] - g_ADoffest[0]);
        *v++ = (int16_t)(g_motorAdValues[1] - g_ADoffest[1]);
        *v++ = (int16_t)(g_motorAdValues[2] - g_ADoffest[2]); n++; n++;
    });

    /* bit 8: Tabc x3  scale=10000 */
    if (tabcEnabled) PACK(TELEM_BIT_TABC, 10000, {
        *v++ = (int16_t)(PSVpwm->Ta * 10000.0f);
        *v++ = (int16_t)(PSVpwm->Tb * 10000.0f);
        *v++ = (int16_t)(PSVpwm->Tc * 10000.0f); n++; n++;
    });

    /* bit 9: vbus  scale=100 */
    if (adcvbus_Enabled) PACK(TELEM_BIT_ADCVBUS, 100, {
        *v++ = (int16_t)(adcToVbus(adcvbus) * 100.0f);
    });

    /* bit 10: local  scale=1000 */
    if (local_Enabled) PACK(TELEM_BIT_LOCAL, 1000, {
        *v++ = (int16_t)(g_pMotor->position * 1000.0f);
    });

    /* bit 11: localOut  scale=10 */
    if (localOut_Enabled) PACK(TELEM_BIT_LOCALOUT, 10, {
        *v++ = (int16_t)(g_pMotor->positionPID.out * 10.0f);
    });

    /* bit 12: IalphaBeta x2  scale=100 */
    if (IAlpha_BetaEnabled) PACK(TELEM_BIT_IALPHABETA, 100, {
        *v++ = (int16_t)(g_pMotor->iAlpha * 100.0f);
        *v++ = (int16_t)(g_pMotor->iBeta * 100.0f); n++;
    });

    /* bit 13: elecAngle x2  scale=1000 */
    if (electricalAngle_Enabled) PACK(TELEM_BIT_ELECANGLE, 1000, {
        *v++ = (int16_t)(g_pMotor->sensoredCorrectedAngle * 1000.0f);
        *v++ = (int16_t)(g_pMotor->mechanicalAngle * 1000.0f); n++;
    });

    #undef PACK

    if (mask == 0) {
        tmr_flag_clear(TMR2, TMR_OVF_FLAG);
        return;
    }

    int dataLen = 4 + n * 2;
    uart3_tx_buffer[0] = 0xA5;
    uart3_tx_buffer[1] = 0x5C;
    uart3_tx_buffer[2] = (uint8_t)dataLen;
    uart3_tx_buffer[3] = (uint8_t)(mask);
    uart3_tx_buffer[4] = (uint8_t)(mask >> 8);
    uart3_tx_buffer[5] = (uint8_t)(mask >> 16);
    uart3_tx_buffer[6] = (uint8_t)(mask >> 24);

    int total = 7 + n * 2;
    uint8_t sum = 0;
    for (int i = 0; i < total; i++) sum += uart3_tx_buffer[i];
    uart3_tx_buffer[total] = sum;
    uart3_tx_buffer[total + 1] = 0x49;

    if (usart3_tx_dma_status == 1) {
        usart3_tx_dma_status = 0;
        dma_data_number_set(DMA1_CHANNEL1, total + 2);
        dma_channel_enable(DMA1_CHANNEL1, TRUE);
    }

    tmr_flag_clear(TMR2, TMR_OVF_FLAG);
  /* add user code end TMR2_GLOBAL_IRQ 0 */

  /* add user code begin TMR2_GLOBAL_IRQ 1 */

  /* add user code end TMR2_GLOBAL_IRQ 1 */
}

/**
  * @brief  this function handles USART3 handler.
  * @param  none
  * @retval none
  */
void USART3_IRQHandler(void)
{
  /* add user code begin USART3_IRQ 0 */
    if (usart_interrupt_flag_get(USART3, USART_RDBF_FLAG) != RESET) {
        uint8_t byte = usart_data_receive(USART3);
        USART3_ParseFixedCommand(byte);
        usart_flag_clear(USART3, USART_RDBF_FLAG);
    }
  /* add user code end USART3_IRQ 0 */

  /* add user code begin USART3_IRQ 1 */

  /* add user code end USART3_IRQ 1 */
}

/* add user code begin 1 */

/* add user code end 1 */
