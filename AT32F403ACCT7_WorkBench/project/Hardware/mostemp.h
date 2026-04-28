#ifndef __MOSTEMP_H
#define __MOSTEMP_H

#include "at32f403a_407.h"

/**
 * @brief MOS 温度检测 & 母线电压检测模块
 *
 * 温度：使用 NTC 热敏电阻 + ADC2 采样，B 值法计算温度
 * 电压：使用电阻分压 + ADC1 采样，计算母线电压
 */

/** 获取 MOS 管温度（单位：°C） */
float GetMosTemp(void);

/** 获取母线电压（单位：V） */
float getVbus(void);

/** ADC 值转母线电压（不触发采样） */
float adcToVbus(uint16_t adc);

#endif
