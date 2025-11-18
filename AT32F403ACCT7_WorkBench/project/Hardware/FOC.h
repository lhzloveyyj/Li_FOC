#ifndef __FOC_H
#define __FOC_H

#include "svpwm.h"
#include "at32f403a_407.h"  // Device header

#ifdef __cplusplus
extern "C" {
#endif

extern float g_udc ;
/**
 * @brief 电流采样状态
 */
typedef struct {
    uint16_t adA;               // A相ADC原始值
    uint16_t adB;               // B相ADC原始值
    uint16_t voltageAOffset;    // A相电压偏移
    uint16_t voltageBOffset;    // B相电压偏移
} FocCurrentState;

/**
 * @brief FOC运行状态结构体
 */
typedef struct {
    FocCurrentState current;    // 电流采样

    float uAlpha, uBeta;        // 电压 αβ
    float iAlpha, iBeta;        // 电流 αβ
    float ia, ib, ic;           // 电流 ABC
    float ua, ub, uc;           // 电压 ABC
    float uq, ud;               // 电压 dq
    float iq, id;               // 电流 dq

    float mechanicalAngle;      // 机械角度（rad）
    float electricalAngle;      // 电角度（rad）
    float correctedAngle;       // 修正后的电角度
    float zeroOffset;           // 零电角度偏移

    //PIDController idPID;        // d轴电流 PID
    //PIDController iqPID;        // q轴电流 PID

    float speedLastAngle;       // 上次电角度（用于计算速度）
    float speed;                // 实际速度
    //PIDController speedPID;     // 速度 PID 控制器

    void (*setPwmCallback)(float pwmA, float pwmB, float pwmC); // 设置PWM函数指针
} FocState;


typedef FocState *PFocState;

extern uint16_t g_motorAdValues[3];  // ADC 原始值数组（外部使用）

extern PFocState g_pMotor;          // 电机 FOC 状态对象指针
extern int cnt;

void AngleInitZeroOffset(float *zeroOffset , float *correctedElecAngle);
void FocContorl(PFocState pFOC,  PSVpwm_State PSVpwm);

#ifdef __cplusplus
}
#endif

#endif // _LOS_FOC_H
