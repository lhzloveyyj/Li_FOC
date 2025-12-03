#include "mostemp.h"
#include <math.h>

#define ADC_MAX 4095.0     // 12-bit ADC
#define VREF    3.3
#define R_FIXED 10000.0    // 10k
#define R0      10000.0    // NTC 的 R@25C
#define T0_K    298.15     // 25°C in Kelvin
#define BETA    3950.0     // 选择一个近似值，最好用元件 datasheet 的 B 值

// adc_val: 0..ADC_MAX
float ntc_temp_c(int adc_val)
{
    if (adc_val <= 0) return -273.15; // 防护
    if (adc_val >= ADC_MAX) return -273.15;
    float vout = VREF * ((double)adc_val / ADC_MAX);
    // 防止除零
    if (vout <= 0.0 || vout >= VREF) return -273.15;

    float r_ntc = R_FIXED * (VREF / vout - 1.0);

    float invT = 1.0 / T0_K + (1.0 / BETA) * log(r_ntc / R0);
    float tK = 1.0 / invT;
    float tC = tK - 273.15;
    return tC;
}

