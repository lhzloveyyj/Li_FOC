#ifndef __MY_MATH_H
#define __MY_MATH_H

#include "at32f403a_407.h"  // AT32F403A/407 头文件

/**
 * @brief 限幅函数
 * @param input    待限制的值
 * @param minValue 最小值
 * @param maxValue 最大值
 * @return 限幅后的值（范围 [minValue, maxValue]）
 */
float LimitValue(float input, float minValue, float maxValue);

#endif
