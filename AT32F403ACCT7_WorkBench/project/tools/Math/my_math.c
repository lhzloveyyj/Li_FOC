#include "my_math.h"

/******************************************************************************
 * 函数名称：LimitValue
 * 功能描述：通用限幅函数。
 *           如果输入值在范围内则原样返回，否则返回边界值。
 * 输入参数：input    - 待限制的值
 *           minValue - 下限
 *           maxValue - 上限
 * 返回值：限幅后的值，范围 [minValue, maxValue]
 ******************************************************************************/
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
