#ifndef __FILTER_H
#define __FILTER_H

#include "at32f403a_407.h"

/**
 * @brief 一阶低通滤波器模块
 *
 * 为电流环和速度环提供一阶低通滤波（LPF），用于平滑反馈信号。
 * 一阶 LPF 传递函数：H(s) = 1 / (1 + s/ωc)
 * 离散化（一阶后向差分）：y[n] = y[n-1] + α * (x[n] - y[n-1])
 * 其中 α = 1 / (1 + 1/(2π*fc*dt))
 */

/* =================== 电流滤波参数 =================== */
#define CURRENT_SAMPLE_FREQ  20000.0f   // 电流采样频率（Hz）
#define CURRENT_CUTOFF_FREQ  20.0f      // 电流 LPF 截止频率（Hz）

#define PI 3.1415f

/** @brief 一阶低通滤波器结构体 */
typedef struct {
    float alpha;     /**< 滤波系数（由采样周期和截止频率计算） 0<alpha<1 */
    float filtered;  /**< 滤波后的输出值 */
} LPF_Filter;

/** @brief dq 轴电流滤波器（两路，Id + Iq） */
typedef struct {
    LPF_Filter Id;   /**< d 轴电流滤波器 */
    LPF_Filter Iq;   /**< q 轴电流滤波器 */
} LPF_Current;

typedef LPF_Current *PLPF_Current;
extern PLPF_Current PM1_LPF;

void LPF_Init(PLPF_Current Pcurrent);
void LPF_Update(PLPF_Current filter, float Id_input, float Iq_input,
                float *Id_out, float *Iq_out);

/* =================== 速度滤波参数 =================== */
#define SPEED_SAMPLE_FREQ  1000.0f      // 速度采样频率（Hz）
#define SPEED_CUTOFF_FREQ  20.0f        // 速度 LPF 截止频率（Hz）

/** @brief 速度滤波器 */
typedef struct {
    LPF_Filter speed;  /**< 速度滤波器 */
} TYPELPF_Speed;

typedef TYPELPF_Speed *PLPF_Speed;
extern PLPF_Speed PM1_LPF_Speed;

void LPF_Speed_Init(PLPF_Speed Pspeed);
void LPF_Speed_Update(PLPF_Speed filter, float speed_input, float *speed_out);

#endif /* __FILTER_H */
