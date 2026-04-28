#include "filter.h"

/* =================== 电流低通滤波器 =================== */

/** M1（第一组电机）的 dq 轴电流滤波器实例 */
LPF_Current M1_LPF;
PLPF_Current PM1_LPF = &M1_LPF;

/******************************************************************************
 * 函数名称：LPF_Init
 * 功能描述：初始化电流低通滤波器。
 *           根据采样频率和截止频率自动计算滤波系数 alpha。
 *
 *           alpha = 1 / (1 + 1/(2*π*fc*dt))
 *
 * 输入参数：Pcurrent - 电流滤波器结构体指针
 ******************************************************************************/
void LPF_Init(PLPF_Current Pcurrent)
{
    float dt = 1.0f / CURRENT_SAMPLE_FREQ;
    float alpha = 1.0f / (1.0f + (1.0f / (2.0f * PI * dt * CURRENT_CUTOFF_FREQ)));

    Pcurrent->Id.alpha = alpha;
    Pcurrent->Id.filtered = 0.0f;
    Pcurrent->Iq.alpha = alpha;
    Pcurrent->Iq.filtered = 0.0f;
}

/******************************************************************************
 * 函数名称：LPF_Update
 * 功能描述：对 dq 轴电流同时进行一阶低通滤波。
 *           y[n] = y[n-1] + α * (x[n] - y[n-1])
 * 输入参数：filter    - 电流滤波器结构体指针
 *           Id_input  - d 轴电流输入
 *           Iq_input  - q 轴电流输入
 * 输出参数：Id_out    - d 轴滤波结果
 *           Iq_out    - q 轴滤波结果
 ******************************************************************************/
void LPF_Update(PLPF_Current filter, float Id_input, float Iq_input,
                float *Id_out, float *Iq_out)
{
    filter->Id.filtered += filter->Id.alpha * (Id_input - filter->Id.filtered);
    *Id_out = filter->Id.filtered;

    filter->Iq.filtered += filter->Iq.alpha * (Iq_input - filter->Iq.filtered);
    *Iq_out = filter->Iq.filtered;
}

/* =================== 速度低通滤波器 =================== */

/** M1 速度滤波器实例 */
TYPELPF_Speed M1_LPF_Speed;
PLPF_Speed PM1_LPF_Speed = &M1_LPF_Speed;

/******************************************************************************
 * 函数名称：LPF_Speed_Init
 * 功能描述：初始化速度低通滤波器。
 * 输入参数：Pspeed - 速度滤波器结构体指针
 ******************************************************************************/
void LPF_Speed_Init(PLPF_Speed Pspeed)
{
    float dt = 1.0f / SPEED_SAMPLE_FREQ;
    float alpha = 1.0f / (1.0f + (1.0f / (2.0f * PI * dt * SPEED_CUTOFF_FREQ)));

    Pspeed->speed.alpha = alpha;
    Pspeed->speed.filtered = 0.0f;
}

/******************************************************************************
 * 函数名称：LPF_Speed_Update
 * 功能描述：对速度信号进行一阶低通滤波。
 * 输入参数：filter      - 速度滤波器结构体指针
 *           speed_input - 速度输入
 * 输出参数：speed_out   - 滤波后速度
 ******************************************************************************/
void LPF_Speed_Update(PLPF_Speed filter, float speed_input, float *speed_out)
{
    filter->speed.filtered += filter->speed.alpha
                              * (speed_input - filter->speed.filtered);
    *speed_out = filter->speed.filtered;
}
