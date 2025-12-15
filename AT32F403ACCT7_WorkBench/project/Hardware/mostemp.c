#include "mostemp.h"
#include <math.h>

#define ADC_MAX 4095.0f     // 12-bit ADC
#define VREF    3.3f
#define R_FIXED 10000.0f    // 10k
#define R0      10000.0f    // NTC 的 R@25C
#define T0_K    298.15f     // 25°C in Kelvin
#define BETA    3950.0f     // 选择一个近似值，最好用元件 datasheet 的 B 值

int GetTempAdc(void)
{
    adc_flag_clear(ADC2, ADC_CCE_FLAG);

    adc_ordinary_software_trigger_enable(ADC2, TRUE);

    while(adc_flag_get(ADC2, ADC_CCE_FLAG) == RESET);

    adc_flag_clear(ADC2, ADC_CCE_FLAG);
    

    int adcVbus = adc_ordinary_conversion_data_get(ADC2);
    
    return adcVbus;
}

// adc_val: 0..ADC_MAX
float ntc_temp_c(int adc_val)
{
    if (adc_val <= 0) return -273.15f; // 防护
    if (adc_val >= ADC_MAX) return -273.15f;
    float vout = VREF * ((double)adc_val / ADC_MAX);
    // 防止除零
    if (vout <= 0.0f || vout >= VREF) return -273.15f;

    float r_ntc = R_FIXED * (VREF / vout - 1.0f);

    float invT = 1.0f / T0_K + (1.0f / BETA) * log(r_ntc / R0);
    float tK = 1.0f / invT;
    float tC = tK - 273.15f;
    return tC;
}

float GetMosTemp(void)
{
    return ntc_temp_c(GetTempAdc());
}

