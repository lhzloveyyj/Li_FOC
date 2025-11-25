/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f403a_407_int.c
  * @brief    main interrupt service routines.
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* includes ------------------------------------------------------------------*/
#include "at32f403a_407_int.h"
/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "usart3.h"   
#include "freertos_app.h"
#include "foc.h"  
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
static float focData[3] = {0.0f};
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
    
    if(dma_interrupt_flag_get(DMA1_FDT1_FLAG))
	{
        
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
    if(adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET)
    {
        g_motorAdValues[0] = adc_preempt_conversion_data_get(ADC1, 0);
        g_motorAdValues[1] = adc_preempt_conversion_data_get(ADC1, 1);
        g_motorAdValues[2] = adc_preempt_conversion_data_get(ADC1, 2);
        
        
        FocContorl(g_pMotor, PSVpwm);
        adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    }
    
  /* add user code end ADC1_2_IRQ 0 */
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
    if(uabcEnabled == 1){
        focData[0] =  g_pMotor->ua;
        focData[1] = g_pMotor->ub;
        focData[2] = g_pMotor->uc;
        USART3_SendPacket(CMD_UABC, &focData[0], 3); 
    }
    if(adcEnabled == 1){
        focData[0] = g_motorAdValues[0] - g_ADoffest[0];
        focData[1] = g_motorAdValues[1] - g_ADoffest[1];
        focData[2] = g_motorAdValues[2] - g_ADoffest[2];
        USART3_SendPacket(CMD_ADC, &focData[0], 3); 
    }
    if(tabcEnabled == 1){
        focData[0] = PSVpwm->Ta;
        focData[1] = PSVpwm->Tb;
        focData[2] = PSVpwm->Tc;
        USART3_SendPacket(CMD_TABC, &focData[0], 3); 
    }
    if(IabcEnabled == 1){
        focData[0] = g_pMotor->Ia;
        focData[1] = g_pMotor->Ib;
        focData[2] = g_pMotor->Ic;
        USART3_SendPacket(CMD_IABC, &focData[0], 3); 
    }
    if(UAlpha_BetaEnabled == 1){
        focData[0] = g_pMotor->uAlpha;
        focData[1] = g_pMotor->uBeta;
        USART3_SendPacket(CMD_UALPHA_BETA, &focData[0], 2); 
    }
    if(IAlpha_BetaEnabled == 1){
        focData[0] = g_pMotor->iAlpha;
        focData[1] = g_pMotor->iBeta;
        USART3_SendPacket(CMD_IALPHA_BETA, &focData[0], 2); 
    }
    if(IQ_ID_Enabled == 1){
        focData[0] = g_pMotor->iq;
        focData[1] = g_pMotor->id;
        USART3_SendPacket(CMD_IQ_ID, &focData[0], 2); 
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
    if(usart_interrupt_flag_get(USART3, USART_RDBF_FLAG) != RESET)
    {
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
