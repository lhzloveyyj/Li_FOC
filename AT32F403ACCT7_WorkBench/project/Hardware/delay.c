#include "delay.h"

#define STEP_DELAY_MS   50

/* Systick 定时参数 */
static __IO uint32_t fac_us;
static __IO uint32_t fac_ms;

/******************************************************************************
 * 函数名称：delay_init
 * 功能描述：初始化延时函数。
 *           配置 SysTick 时钟源为 AHB 时钟不分频，
 *           计算微秒和毫秒延时的计数值。
 ******************************************************************************/
void delay_init(void)
{
    systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);
    fac_us = system_core_clock / (1000000U);
    fac_ms = fac_us * (1000U);
}

/******************************************************************************
 * 函数名称：delay_us
 * 功能描述：微秒级延时（阻塞）。
 * 输入参数：nus - 延时微秒数
 ******************************************************************************/
void delay_us(uint32_t nus)
{
    uint32_t temp = 0;
    SysTick->LOAD = (uint32_t)(nus * fac_us);
    SysTick->VAL = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16)));

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL = 0x00;
}

/******************************************************************************
 * 函数名称：delay_ms
 * 功能描述：毫秒级延时（阻塞，分段处理避免 SysTick 计数值溢出）。
 * 输入参数：nms - 延时毫秒数
 ******************************************************************************/
void delay_ms(uint16_t nms)
{
    uint32_t temp = 0;
    while (nms) {
        if (nms > STEP_DELAY_MS) {
            SysTick->LOAD = (uint32_t)(STEP_DELAY_MS * fac_ms);
            nms -= STEP_DELAY_MS;
        } else {
            SysTick->LOAD = (uint32_t)(nms * fac_ms);
            nms = 0;
        }
        SysTick->VAL = 0x00;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        do {
            temp = SysTick->CTRL;
        } while ((temp & 0x01) && !(temp & (1 << 16)));

        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->VAL = 0x00;
    }
}

/******************************************************************************
 * 函数名称：delay_sec
 * 功能描述：秒级延时（阻塞，调用 delay_ms 实现）。
 * 输入参数：sec - 延时秒数
 ******************************************************************************/
void delay_sec(uint16_t sec)
{
    for (uint16_t index = 0; index < sec; index++) {
        delay_ms(500);
        delay_ms(500);
    }
}
