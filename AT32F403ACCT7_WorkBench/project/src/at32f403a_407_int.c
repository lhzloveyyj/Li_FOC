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
#include "protocol.h"
#include "mostemp.h"
#include "smo_observer.h"
/* add user code end private includes */

/* add user code begin private macro */
/** 遥测数据缓存 */
static float focData[3] = {0.0f};
/** SMO 遥测轮询槽位（用于多路复用一个 DMA 通道发送） */
static uint8_t smoTelemetrySlot = 0;
/* add user code end private macro */

/* ... 省略 Artery 库默认的中断函数 ... */

/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  this function handles NMI exception.
  */
void NMI_Handler(void)
{
}

/**
  * @brief  this function handles Hard Fault exception.
  */
void HardFault_Handler(void)
{
  /* go to infinite loop when Hard Fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles Memory Manage exception.
  */
void MemManage_Handler(void)
{
  /* go to infinite loop when Memory Manage exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles Bus Fault exception.
  */
void BusFault_Handler(void)
{
  /* go to infinite loop when Bus Fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles Usage Fault exception.
  */
void UsageFault_Handler(void)
{
  /* go to infinite loop when Usage Fault exception occurs */
  while(1)
  {
  }
}

/**
  * @brief  this function handles Debug Monitor exception.
  */
void DebugMon_Handler(void)
{
}

extern void xPortSysTickHandler(void);

/**
  * @brief  this function handles SysTick exception.
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

/* add user code begin 0 */

/* add user code end 0 */

/******************************************************************************
 * 中断服务函数：ADC1_2_IRQHandler
 *
 * 功能描述：ADC1 抢占转换完成中断（PCCE）。
 *           FOC 控制主循环在此中断中触发，每个 ADC 转换完成执行一次。
 *
 * 执行流程：
 *   1. 读取三相电流 + 母线电压的 ADC 值
 *   2. 调用 FocContorl() 执行完整的 FOC 控制算法
 *   3. 清除中断标志
 *
 * 中断频率：由 TIM1 触发 ADC 采样决定，通常为 10kHz~20kHz。
 ******************************************************************************/
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

  if (adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET) {
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
  }
}

/******************************************************************************
 * 中断服务函数：TMR2_GLOBAL_IRQHandler
 *
 * 功能描述：TMR2 定时器溢出中断。
 *           用于定时发送遥测数据到上位机。
 *           根据各使能标志位的状态，选择性发送调试数据。
 *
 * SMO 遥测轮询机制：
 *   SMO 角度/速度/反电势/原始角度/诊断量和实际电角度共用 DMA 通道发送，
 *   通过轮询槽位（smoTelemetrySlot）交替发送，避免冲突。
 ******************************************************************************/
void TMR2_GLOBAL_IRQHandler(void)
{
  /* add user code begin TMR2_GLOBAL_IRQ 0 */
    /* ---- 三相电压 ---- */
    if (uabcEnabled == 1) {
        focData[0] = g_pMotor->ua;
        focData[1] = g_pMotor->ub;
        focData[2] = g_pMotor->uc;
        USART3_SendPacket(CMD_UABC, &focData[0], 3);
    }

    /* ---- ADC 原始值 ---- */
    if (adcEnabled == 1) {
        focData[0] = g_motorAdValues[0] - g_ADoffest[0];
        focData[1] = g_motorAdValues[1] - g_ADoffest[1];
        focData[2] = g_motorAdValues[2] - g_ADoffest[2];
        USART3_SendPacket(CMD_ADC, &focData[0], 3);
    }

    /* ---- SVPWM 占空比 ---- */
    if (tabcEnabled == 1) {
        focData[0] = PSVpwm->Ta;
        focData[1] = PSVpwm->Tb;
        focData[2] = PSVpwm->Tc;
        USART3_SendPacket(CMD_TABC, &focData[0], 3);
    }

    /* ---- 三相电流 ---- */
    if (IabcEnabled == 1) {
        focData[0] = g_pMotor->Ia;
        focData[1] = g_pMotor->Ib;
        focData[2] = g_pMotor->Ic;
        USART3_SendPacket(CMD_IABC, &focData[0], 3);
    }

    /* ---- Uα/Uβ ---- */
    if (UAlpha_BetaEnabled == 1) {
        focData[0] = g_pMotor->uAlpha;
        focData[1] = g_pMotor->uBeta;
        USART3_SendPacket(CMD_UALPHA_BETA, &focData[0], 2);
    }

    /* ---- Iα/Iβ ---- */
    if (IAlpha_BetaEnabled == 1) {
        focData[0] = g_pMotor->iAlpha;
        focData[1] = g_pMotor->iBeta;
        USART3_SendPacket(CMD_IALPHA_BETA, &focData[0], 2);
    }

    /* ---- Id/Iq ---- */
    if (IQ_ID_Enabled == 1) {
        focData[0] = g_pMotor->iq;
        focData[1] = g_pMotor->id;
        USART3_SendPacket(CMD_IQ_ID, &focData[0], 2);
    }

    /* ---- 机械角度 ---- */
    if (anglePrintingEnabled == 1) {
        focData[0] = g_pMotor->mechanicalAngle;
        USART3_SendPacket(CMD_MECHANICALANGLE, &focData[0], 1);
    }

    /* ---- 速度 ---- */
    if (speed_Enabled == 1) {
        focData[0] = g_pMotor->speed;
        USART3_SendPacket(CMD_SPEED, &focData[0], 1);
    }

    /* ---- 速度环输出 ---- */
    if (speedOut_Enabled == 1) {
        focData[0] = g_pMotor->speedPID.out;
        USART3_SendPacket(CMD_SPEEDOUT, &focData[0], 1);
    }

    /* ---- 位置 ---- */
    if (local_Enabled == 1) {
        focData[0] = g_pMotor->position;
        USART3_SendPacket(CMD_LOCAL, &focData[0], 1);
    }

    /* ---- 位置环输出 ---- */
    if (localOut_Enabled == 1) {
        focData[0] = g_pMotor->positionPID.out;
        USART3_SendPacket(CMD_LOCALOUT, &focData[0], 1);
    }

    /* ---- 母线电压 ---- */
    if (adcvbus_Enabled == 1) {
        focData[0] = adcToVbus(adcvbus);
        USART3_SendPacket(CMD_ADCVBUS, &focData[0], 1);
    }

    /* ---- SMO + 电角度轮询发送 ----
     * SMO 角度/速度/反电势/原始角度/诊断量和编码器电角度共用串口带宽。
     * 通过轮询槽位交替发送，确保多路打开时公平分配。
     */
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t slot = (uint8_t)((smoTelemetrySlot + i) % 6U);

        if ((slot == 0U) && (electricalAngle_Enabled == 1)) {
            /* 编码器电角度：有感真实参考，用来和 SMO/PLL 两个角度对比 */
            focData[0] = g_pMotor->correctedAngle;
            USART3_SendPacket(CMD_ELECTRICALANGLE, &focData[0], 1);
            smoTelemetrySlot = 1U;
            break;
        }
        if ((slot == 1U) && (smoAngle_Enabled == 1)) {
            /* PLL 角度：SMO 反电势经 PLL 锁相后的最终角度 */
            focData[0] = g_smoObserver.angle;
            USART3_SendPacket(CMD_SMO_ANGLE, &focData[0], 1);
            smoTelemetrySlot = 2U;
            break;
        }
        if ((slot == 2U) && (smoSpeed_Enabled == 1)) {
            focData[0] = g_smoObserver.speed;
            USART3_SendPacket(CMD_SMO_SPEED, &focData[0], 1);
            smoTelemetrySlot = 3U;
            break;
        }
        if ((slot == 3U) && (smoBackEmf_Enabled == 1)) {
            /* 反电势波形：先看 eAlpha/eBeta 是否接近正交正弦，再判断角度 */
            focData[0] = g_smoObserver.eAlpha;
            focData[1] = g_smoObserver.eBeta;
            USART3_SendPacket(CMD_SMO_BACKEMF, &focData[0], 2);
            smoTelemetrySlot = 4U;
            break;
        }
        if ((slot == 4U) && (smoRawAngle_Enabled == 1)) {
            /* SMO 角度：直接 atan2(-eAlpha, eBeta)，不经过 PLL */
            focData[0] = g_smoObserver.rawAngle;
            USART3_SendPacket(CMD_SMO_RAW_ANGLE, &focData[0], 1);
            smoTelemetrySlot = 5U;
            break;
        }
        if ((slot == 5U) && (smoDiag_Enabled == 1)) {
            /* 诊断量：pllError 看锁相误差，eMag 看反电势幅值是否足够 */
            focData[0] = g_smoObserver.pllError;
            focData[1] = g_smoObserver.eMag;
            USART3_SendPacket(CMD_SMO_DIAG, &focData[0], 2);
            smoTelemetrySlot = 0U;
            break;
        }
    }

    tmr_flag_clear(TMR2, TMR_OVF_FLAG);
  /* add user code end TMR2_GLOBAL_IRQ 0 */
}

/******************************************************************************
 * 中断服务函数：USART3_IRQHandler
 *
 * 功能描述：USART3 接收中断。
 *           逐字节接收上位机命令帧，交由 USART3_ParseFixedCommand 解析。
 ******************************************************************************/
void USART3_IRQHandler(void)
{
  /* add user code begin USART3_IRQ 0 */
    if (usart_interrupt_flag_get(USART3, USART_RDBF_FLAG) != RESET) {
        uint8_t byte = usart_data_receive(USART3);
        USART3_ParseFixedCommand(byte);
        usart_flag_clear(USART3, USART_RDBF_FLAG);
    }
  /* add user code end USART3_IRQ 0 */
}

/**
  * @brief  this function handles DMA1_Channel1 handler.
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
}
