#ifndef __FOC_CONFIG_H
#define __FOC_CONFIG_H

#include "at32f403a_407.h"  // Device header

/* ---------------- 电压、电流、电机参数 ---------------- */
#define FOC_VOLTAGE_LIMIT       12.0f       // 电压限制值（最大输出幅值）
#define FOC_BUS_VOLTAGE         12.0f       // 母线电压 Udc
#define FOC_POLE_PAIRS          11           // 电机极对数
#define FOC_RS                  0.01f       // 电流采样电阻值（单位：Ω）
#define FOC_CURRENT_GAIN        50.0f       // 电流放大倍数（运放增益）
#define FOC_IQ_MAX              40.0f       // 最大目标 q 轴电流
#define FOC_STRONGDRAG			1.0f

/* ---------------- 常数定义 ---------------- */
#define FOC_PI                  3.14159f
#define FOC_2PI                 6.28318f
#define FOC_3PI_2               4.712388f
#define FOC_SQRT3               1.732f
#define FOC_SQRT3_DIV_2         0.866025f
#define FOC_1_2                0.5f
#define FOC_EPSILON             1e-6f        // 浮点精度误差容忍阈值

/* ---------------- PWM/ADC ---------------- */
#define FOC_ALL_DUTY            5999         // PWM周期（例如 Time2）
#define FOC_ADC_REF_VOLTAGE     3.3f          // ADC参考电压（单位：V）
#define FOC_GAIN                10 
#define FOC_SHUNT_R             0.01f 

/* ---------------- SMO观测器 ----------------
 * 参数先按后台观测给一组保守默认值，后续可根据电机实际 Rs/Ls 和PWM频率标定。
 */
#define FOC_SMO_RS                  0.198f
#define FOC_SMO_LS                  0.000057f
#define FOC_SMO_TS                  0.0001f
#define FOC_SMO_K_SLIDE             8.0f
#define FOC_SMO_E_LPF_ALPHA         0.08f
#define FOC_SMO_SPEED_LPF_ALPHA     0.02f
#define FOC_SMO_CURRENT_ERR_BAND    0.5f

/* ---------------- 调试功能开关 ---------------- */
#define FOC_ENABLE_DEBUG        1             // 调试开关（1=开启，0=关闭）

#endif // _LOS_FOC_CONFIG_H
