#ifndef __FOC_H
#define __FOC_H

#include "svpwm.h"
#include "at32f403a_407.h"  // Device header

#ifdef __cplusplus
extern "C" {
#endif

// 控制模式命令枚举
typedef enum
{
    FOC_OPEN_LOOP    = 0x01,  // 开环模式
    FPC_CURRENT_LOOP = 0x02,  // 电流环模式
    FOC_SPEED_LOOP   = 0x03,  // 速度环模式
    FOC_POSITION_LOOP= 0x04,  // 位置环模式
} CtrolMode_TypeDef;


extern float g_udc ;
/**
 * @brief 电流采样状态
 */
typedef struct {
    uint16_t adA;               // A相ADC原始值
    uint16_t adB;               // B相ADC原始值
    uint16_t adC;               // B相ADC原始值
    uint16_t voltageAOffset;    // A相电压偏移
    uint16_t voltageBOffset;    // B相电压偏移
    uint16_t voltageCOffset;    // B相电压偏移
} FocCurrentState;

/*
增量式PI调节
*/
struct PI_Struct
{
    /* data */
    float kp;
    float ki;
    float kd;

    float pre;
    float tar;
    float bias;
    float lastBias;
    float out;
    float outMax;
};

/**
 * @brief FOC运行状态结构体
 */
typedef struct {
    FocCurrentState current;    // 电流采样

    float uAlpha, uBeta;        // 电压 αβ
    float iAlpha, iBeta;        // 电流 αβ
    float Ia, Ib, Ic;           // 电流 ABC
    float ua, ub, uc;           // 电压 ABC
    float uq, ud;               // 电压 dq
    float iq, id;               // 电流 dq
    
    int   pole_pairs;
    int   dir;
    float mechanicalAngle;      // 机械角度（rad）
    float electricalAngle;      // 电角度（rad）
    float correctedAngle;       // 修正后的电角度
    float zeroOffset;           // 零电角度偏移

    struct PI_Struct idPID;            // d轴电流 PID
    struct PI_Struct iqPID;        // q轴电流 PID
    float tariq;
	float tarid;

    float speedLastAngle;       // 上次电角度（用于计算速度）
    float speed;                // 实际速度
    //PIDController speedPID;     // 速度 PID 控制器
    
    uint8_t ctrolmode;

    void (*setPwmCallback)(float pwmA, float pwmB, float pwmC); // 设置PWM函数指针
} FocState;


typedef FocState *PFocState;

extern volatile int g_motorAdValues[3];  // ADC 原始值数组（外部使用）
extern volatile uint16_t g_ADoffest[3];

extern PFocState g_pMotor;          // 电机 FOC 状态对象指针
extern int cnt;

void AngleInitZeroOffset(float *zeroOffset , float *correctedElecAngle);
void FocContorl(PFocState pFOC,  PSVpwm_State PSVpwm);
void getAdoffset(void);

#ifdef __cplusplus
}
#endif

#endif // _LOS_FOC_H
