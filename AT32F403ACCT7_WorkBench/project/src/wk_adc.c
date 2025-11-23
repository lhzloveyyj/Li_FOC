/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_adc.c
  * @brief    work bench config program
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

/* Includes ------------------------------------------------------------------*/
#include "wk_adc.h"

/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  init adc1 function.
  * @param  none
  * @retval none
  */
void wk_adc1_init(void)
{
  /* add user code begin adc1_init 0 */

  /* add user code end adc1_init 0 */

  gpio_init_type gpio_init_struct;
  adc_base_config_type adc_base_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin adc1_init 1 */

  /* add user code end adc1_init 1 */

  /*gpio--------------------------------------------------------------------*/ 
  /* configure the IN0 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = GPIO_PINS_0;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the IN1 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = GPIO_PINS_1;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the IN2 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = GPIO_PINS_2;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the IN4 pin */
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = GPIO_PINS_4;
  gpio_init(GPIOA, &gpio_init_struct);

  adc_reset(ADC1);
  crm_adc_clock_div_set(CRM_ADC_DIV_6);

  /*adc_common_settings-------------------------------------------------------------*/ 
  adc_combine_mode_select(ADC_INDEPENDENT_MODE);

  /*adc_settings--------------------------------------------------------------------*/ 
  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 1;
  adc_base_config(ADC1, &adc_base_struct);

  /* adc_preempt_conversionmode--------------------------------------------- */
  adc_preempt_channel_length_set(ADC1, 4);

  adc_preempt_channel_set(ADC1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_7_5);
  adc_preempt_offset_value_set(ADC1, ADC_PREEMPT_CHANNEL_1, 0x0);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_1, 2, ADC_SAMPLETIME_7_5);
  adc_preempt_offset_value_set(ADC1, ADC_PREEMPT_CHANNEL_2, 0x0);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_2, 3, ADC_SAMPLETIME_7_5);
  adc_preempt_offset_value_set(ADC1, ADC_PREEMPT_CHANNEL_3, 0x0);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_4, 4, ADC_SAMPLETIME_1_5);
  adc_preempt_offset_value_set(ADC1, ADC_PREEMPT_CHANNEL_4, 0x0);
  adc_preempt_conversion_trigger_set(ADC1, ADC12_PREEMPT_TRIG_TMR1CH4, TRUE);
  /**
   * Users need to configure ADC1 interrupt functions according to the actual application.
   * 1. Call the below function to enable the corresponding ADC1 interrupt.
   *     --adc_interrupt_enable(...)
   * 2. Add the user's interrupt handler code into the below function in the at32f403a_407_int.c file.
   *     --void ADC1_2_IRQHandler(void)
   */

  /* preempt interrupt------------------------------------------------------ */
  //adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);

  /* add user code begin adc1_init 2 */

  /* add user code end adc1_init 2 */
  
  adc_enable(ADC1, TRUE);
  
  /* adc calibration-------------------------------------------------------- */
  adc_calibration_init(ADC1);
  while(adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while(adc_calibration_status_get(ADC1));

  /* add user code begin adc1_init 3 */

  /* add user code end adc1_init 3 */
}

/* add user code begin 1 */

/* add user code end 1 */
