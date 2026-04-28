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

/**
  * @brief  主函数
  * @param  none
  * @retval none
  *
  * 执行流程：
  *   1. 系统时钟配置
  *   2. 外设时钟配置
  *   3. 调试串口初始化
  *   4. NVIC 中断优先级配置
  *   5. GPIO 初始化
  *   6. ADC1/ADC2 初始化
  *   7. DMA1 通道 1 初始化（USART3 TX）
  *   8. USART1/USART3 初始化
  *   9. SPI1 初始化（MT6701 编码器）
  *   10. TMR1/TMR2 初始化（PWM + 定时触发）
  *   11. CAN1 初始化
  *   12. 使能相关中断
  *   13. 启动 FreeRTOS
  */
int main(void)
{
  /* system clock config */
  wk_system_clock_config();
  /* config periph clock */
  wk_periph_clock_config();
  /* init debug function */
  wk_debug_config();
  /* nvic config */
  wk_nvic_config();
  /* init gpio function */
  wk_gpio_config();
  /* init adc2 function */
  wk_adc2_init();
  /* init adc1 function */
  wk_adc1_init();
  /* init dma1 channel1 */
  wk_dma1_channel1_init();
  /* config dma channel transfer parameter */
  wk_dma_channel_config(DMA1_CHANNEL1,
                        (uint32_t)&USART3->dt,
                        DMA1_CHANNEL1_MEMORY_BASE_ADDR,
                        DMA1_CHANNEL1_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  /* init usart1 function (debug) */
  wk_usart1_init();
  /* init usart3 function (communication with host) */
  wk_usart3_init();
  /* init spi1 function (MT6701 encoder) */
  wk_spi1_init();
  /* init tmr1 function (PWM generation) */
  wk_tmr1_init();
  /* init tmr2 function (ADC trigger & telemetry timer) */
  wk_tmr2_init();
  /* init can1 function */
  wk_can1_init();

  /* ---- 使能中断 ---- */
  adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);     // ADC 转换完成中断
  tmr_interrupt_enable(TMR2, TMR_OVF_INT, TRUE);       // TMR2 溢出中断
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);  // DMA 发送完成中断
  usart_interrupt_enable(USART3, USART_RDBF_INT, TRUE);    // USART3 接收中断

  /* ---- 启动 FreeRTOS ---- */
  wk_freertos_init();

  /* FreeRTOS 启动后不会执行到这里 */
  while (1) {
  }
}
