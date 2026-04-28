#ifndef __delay_H
#define __delay_H

#include "at32f403a_407.h"
#include "at32f403a_407_misc.h"

/**
 * @brief 延时函数模块
 * 基于 SysTick 实现微秒/毫秒/秒级阻塞延时。
 */

void delay_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void delay_sec(uint16_t sec);

#endif
