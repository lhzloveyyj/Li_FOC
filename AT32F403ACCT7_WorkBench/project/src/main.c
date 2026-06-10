/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    主程序入口
  *
  * 系统启动流程：
  *   1. 配置系统时钟
  *   2. 初始化各外设（ADC、DMA、USART、SPI、定时器、CAN）
  *   3. 使能相关中断
  *   4. 启动 FreeRTOS 调度器
  *
  * 启动后，控制逻辑主要在以下路径中执行：
  *   - ADC1 中断 → FocContorl()（电流环，10kHz~20kHz）
  *   - TMR2 中断 → 遥测数据发送
  *   - comm_task  → 上位机命令处理
  *   - control_task → 速度/位置控制
  *   - Monitor_task → 温度监控
  **************************************************************************
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "at32f403a_407_wk_config.h"
#include "wk_adc.h"
#include "wk_can.h"
#include "wk_debug.h"
#include "wk_spi.h"
#include "wk_tmr.h"
#include "wk_usart.h"
#include "wk_dma.h"
#include "wk_gpio.h"
#include "freertos_app.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
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

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */

  /* add user code end 1 */

  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /* init debug function. */
  wk_debug_config();

  /* nvic config. */
  wk_nvic_config();

  /* init gpio function. */
  wk_gpio_config();

  /* init adc2 function. */
  wk_adc2_init();

  /* init adc1 function. */
  wk_adc1_init();

  /* init dma1 channel1 */
  wk_dma1_channel1_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR 
     and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL1, 
                        (uint32_t)&USART3->dt, 
                        DMA1_CHANNEL1_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL1_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  /* init usart1 function. */
  wk_usart1_init();

  /* init usart3 function. */
  wk_usart3_init();

  /* init spi1 function. */
  wk_spi1_init();

  /* init tmr1 function. */
  wk_tmr1_init();

  /* init tmr2 function. */
  wk_tmr2_init();

  /* init can1 function. */
  wk_can1_init();

  /* add user code begin 2 */
  adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
  tmr_interrupt_enable(TMR2, TMR_OVF_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  usart_interrupt_enable(USART3, USART_RDBF_INT, TRUE);
  /* add user code end 2 */

  /* init freertos function. */
  wk_freertos_init();

  while(1)
  {
    /* add user code begin 3 */

    /* add user code end 3 */
  }
}
