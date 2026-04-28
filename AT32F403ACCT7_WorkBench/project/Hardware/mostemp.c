#include "mostemp.h"
#include <math.h>

/* =================== NTC 温度检测参数 =================== */
#define ADC_MAX    4095.0f     // 12-bit ADC 最大值
#define VREF       3.3f        // ADC 参考电压（单位：V）
#define R_FIXED    10000.0f    // 分压电阻（单位：Ω）
#define R0         10000.0f    // NTC 在 25°C 时的标称电阻（单位：Ω）
#define T0_K       298.15f     // 25°C 对应的开尔文温度
#define BETA       3950.0f     // NTC B 值（需参考具体元件 datasheet）

/******************************************************************************
 * 函数名称：GetTempAdc
 * 功能描述：触发 ADC2 采样温度传感器并读取原始值。
 * 返回值：ADC 原始值（0~4095）
 ******************************************************************************/
int GetTempAdc(void)
{
    adc_flag_clear(ADC2, ADC_CCE_FLAG);
    adc_ordinary_software_trigger_enable(ADC2, TRUE);
    while (adc_flag_get(ADC2, ADC_CCE_FLAG) == RESET);
    adc_flag_clear(ADC2, ADC_CCE_FLAG);

    return adc_ordinary_conversion_data_get(ADC2);
}

/******************************************************************************
 * 函数名称：ntc_temp_c
 * 功能描述：根据 NTC 分压 ADC 值计算温度。
 *
 * 计算过程：
 *   1. Vout = Vref * ADC / ADC_MAX
 *   2. R_ntc = R_fixed * (Vref / Vout - 1)
 *   3. 使用 B 值公式：1/T = 1/T0 + ln(R_ntc/R0) / B
 *   4. T(°C) = T(K) - 273.15
 *
 * 输入参数：adc_val - ADC 原始值（0~4095）
 * 返回值：温度（单位：°C），异常情况返回 -273.15
 ******************************************************************************/
float ntc_temp_c(int adc_val)
{
    if (adc_val <= 0) return -273.15f;
    if (adc_val >= ADC_MAX) return -273.15f;

    float vout = VREF * ((double)adc_val / ADC_MAX);
    if (vout <= 0.0f || vout >= VREF) return -273.15f;

    float r_ntc = R_FIXED * (VREF / vout - 1.0f);
    float invT = 1.0f / T0_K + (1.0f / BETA) * logf(r_ntc / R0);
    float tK = 1.0f / invT;
    return tK - 273.15f;
}

/******************************************************************************
 * 函数名称：GetMosTemp
 * 功能描述：获取 MOS 管温度（简化接口）。
 * 返回值：温度（单位：°C）
 ******************************************************************************/
float GetMosTemp(void)
{
    return ntc_temp_c(GetTempAdc());
}

/* =================== 母线电压检测参数 =================== */
#define ADC_VERF    3.3f
#define DCVBUS_R1   20          // 分压上拉电阻（单位：kΩ）
#define DCVBUS_R2   1           // 分压下拉电阻（单位：kΩ）

/******************************************************************************
 * 函数名称：getVbus
 * 功能描述：通过 ADC1 采集母线电压。
 *
 *   母线电压 = ADC / 4096 * 3.3V * (R1 + R2) / R2
 *
 * 返回值：母线电压（单位：V）
 ******************************************************************************/
float getVbus(void)
{
    adc_flag_clear(ADC1, ADC_CCE_FLAG);
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
    while (adc_flag_get(ADC1, ADC_CCE_FLAG) == RESET);
    adc_flag_clear(ADC1, ADC_CCE_FLAG);

    uint16_t adcVbus = adc_ordinary_conversion_data_get(ADC1);

    float vbus = (adcVbus / 4096.0f) * ADC_VERF
                 * (DCVBUS_R1 + DCVBUS_R2) / DCVBUS_R2;
    return vbus;
}

/******************************************************************************
 * 函数名称：adcToVbus
 * 功能描述：将 ADC 原始值转换为母线电压（不触发采样，直接用已有值）。
 * 输入参数：adc - ADC 原始值
 * 返回值：母线电压（单位：V）
 ******************************************************************************/
float adcToVbus(uint16_t adc)
{
    return (adc / 4096.0f) * ADC_VERF * (DCVBUS_R1 + DCVBUS_R2) / DCVBUS_R2;
}
