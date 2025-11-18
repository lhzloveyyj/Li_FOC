#include "my_math.h"

/**
 * @brief     限制输入值在指定范围内
 * @param     input      待限制的值
 * @param     minValue   最小值
 * @param     maxValue   最大值
 * @return    限幅后的值
 */
float LimitValue(float input, float minValue, float maxValue)
{
    if (input >= maxValue) {
        return maxValue;
    } else if (input <= minValue) {
        return minValue;
    } else {
        return input;
    }
}
