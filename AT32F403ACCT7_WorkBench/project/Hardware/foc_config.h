#ifndef __FOC_CONFIG_H
#define __FOC_CONFIG_H

#include "at32f403a_407.h"  // Device header

/* =================== 电压、电流、电机参数 =================== */
#define FOC_VOLTAGE_LIMIT       12.0f       // 电压限制值（最大输出幅值）
#define FOC_BUS_VOLTAGE         12.0f       // 母线电压 Udc
#define FOC_POLE_PAIRS          11          // 电机极对数
#define FOC_RS                  0.01f       // 电流采样电阻值（单位：Ω）
#define FOC_CURRENT_GAIN        50.0f       // 电流放大倍数（运放增益）
#define FOC_IQ_MAX              40.0f       // 最大目标 q 轴电流
#define FOC_STRONGDRAG			1.0f        // 强拖对准电压（Ud，单位 V）

/* =================== 常数定义 =================== */
#define FOC_PI                  3.14159f    // 圆周率 PI
#define FOC_2PI                 6.28318f    // 2 * PI
#define FOC_3PI_2               4.712388f   // 3/2 * PI
#define FOC_SQRT3               1.732f      // 根号 3
#define FOC_SQRT3_DIV_2         0.866025f   // 根号 3 / 2
#define FOC_1_2                 0.5f        // 1/2
#define FOC_EPSILON             1e-6f       // 浮点精度误差容忍阈值

/* =================== PWM/ADC =================== */
#define FOC_ALL_DUTY            5999        // PWM 周期（Timer1/Timer2 通用）
#define FOC_ADC_REF_VOLTAGE     3.3f        // ADC 参考电压（单位：V）
#define FOC_GAIN                10          // 运放放大倍数
#define FOC_SHUNT_R             0.01f       // 电流采样电阻（单位：Ω）

/* =================== SMO 滑模观测器 ===================
 * 参数先按后台观测给一组保守默认值，后续可根据电机实际 Rs/Ls 和 PWM 频率标定。
 * 注意：SMO 运行在 PWM 中断中，因此 Ts = 1 / PWM 频率。
 */
#define FOC_SMO_RS                  0.198f      // 电机相电阻（单位：Ω）
#define FOC_SMO_LS                  0.000057f   // 等效电感（Lq+Ld)/2（单位：H）
#define FOC_SMO_TS                  0.0001f     // SMO 采样周期（单位：s，对应 10kHz）
#define FOC_SMO_K_SLIDE             8.0f        // 滑模增益
#define FOC_SMO_E_LPF_ALPHA         0.08f       // 反电势低通滤波系数
#define FOC_SMO_SPEED_LPF_ALPHA     0.02f       // 速度低通滤波系数
#define FOC_SMO_CURRENT_ERR_BAND    0.5f        // 电流误差饱和带，用于平滑滑模切换

/* =================== SMO PLL 锁相环参数 =================== */
#define FOC_SMO_PLL_KP             200.0f      // PLL 比例增益
#define FOC_SMO_PLL_KI             50.0f       // PLL 积分增益

/* =================== 调试功能开关 =================== */
#define FOC_ENABLE_DEBUG        1             // 调试开关（1=开启，0=关闭）

#endif // _LOS_FOC_CONFIG_H
