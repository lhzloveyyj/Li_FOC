#ifndef __FILTER_H
#define __FILTER_H

#include "at32f403a_407.h"  // AT32F403A/407 头文件

// 采样频率 24kHz, 截止频率 500Hz（适合低速电机）
#define CURRENT_SAMPLE_FREQ 20000.0f
#define CURRENT_CUTOFF_FREQ 20.0f

#define PI 3.1415f  

typedef struct {
    float alpha;     ///< 滤波系数（由采样周期和截止频率计算）
    float filtered;  ///< 滤波后的输出值
} LPF_Filter;

typedef struct {
    LPF_Filter Id; ///< d 轴电流滤波器
    LPF_Filter Iq; ///< q 轴电流滤波器
} LPF_Current;

typedef LPF_Current *PLPF_Current;
extern PLPF_Current PM1_LPF;
void LPF_Init(PLPF_Current Pcurrent);
void LPF_Update(PLPF_Current filter, float Id_input, float Iq_input, float *Id_out, float *Iq_out);



#define SPEED_SAMPLE_FREQ 1000.0f
#define SPEED_CUTOFF_FREQ 20.0f
typedef struct {
    LPF_Filter speed; ///< 速度滤波器
} TYPELPF_Speed;

typedef TYPELPF_Speed *PLPF_Speed;
extern PLPF_Speed PM1_LPF_Speed;
void LPF_Speed_Init(PLPF_Speed Pspeed);
void LPF_Speed_Update(PLPF_Speed filter, float speed_input, float *speed_out);

#endif // __FILTER_H
