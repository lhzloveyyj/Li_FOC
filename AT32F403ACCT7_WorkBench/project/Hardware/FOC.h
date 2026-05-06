#ifndef __FOC_H
#define __FOC_H

#include "SVPWM.h"
#include "at32f403a_407.h"  // Device header

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 控制模式枚举
 */
typedef enum
{
    FOC_OPEN_LOOP     = 0x01,  // 开环模式：直接给定 Uq/Ud，无反馈
    FPC_CURRENT_LOOP  = 0x02,  // 电流闭环模式：Id/Iq 电流 PI 闭环
    FOC_SPEED_LOOP    = 0x03,  // 速度闭环模式：速度 PI → 电流 PI 串联
    FOC_POSITION_LOOP = 0x04,  // 位置闭环模式：位置 PD → 速度 PI → 电流 PI 串联
} CtrolMode_TypeDef;

/**
 * @brief FOC 角度/速度反馈来源
 */
typedef enum
{
    FOC_SENSOR_MODE_SENSORED   = 0x00,  // 有感：MT6701 编码器
    FOC_SENSOR_MODE_SENSORLESS = 0x01,  // 无感：SMO + PLL
} FocSensorMode_TypeDef;

/**
 * @brief 无感 I/F 启动状态
 */
typedef enum
{
    FOC_SENSORLESS_IF_OFF  = 0x00,  // 未启动或已退出
    FOC_SENSORLESS_IF_ALIGN = 0x01, // 固定虚拟角度 + Id 对齐转子
    FOC_SENSORLESS_IF_RAMP = 0x02,  // I/F 虚拟角度 + 固定 Iq 拖动
    FOC_SENSORLESS_IF_DONE = 0x03,  // 已切换到 SMO/PLL 闭环
} FocSensorlessIFState_TypeDef;

extern float g_udc;

/**
 * @brief 电流采样状态结构体
 * ADC 采集三相电流原始值和偏置，用于电流重构。
 */
typedef struct {
    uint16_t adA;               // A 相 ADC 原始值
    uint16_t adB;               // B 相 ADC 原始值
    uint16_t adC;               // C 相 ADC 原始值
    uint16_t voltageAOffset;    // A 相电压零点偏置
    uint16_t voltageBOffset;    // B 相电压零点偏置
    uint16_t voltageCOffset;    // C 相电压零点偏置
} FocCurrentState;

/**
 * @brief 增量式 PID 控制器结构体
 * 采用增量式算法：out += Ki*(bias - lastBias) + Kp*bias
 * Kd 当前未在电流环使用，保留用于位置环。
 */
struct PI_Struct
{
    float kp;       // 比例系数
    float ki;       // 积分系数
    float kd;       // 微分系数

    float pre;      // 当前反馈值（实际值）
    float tar;      // 目标值
    float bias;     // 当前偏差（tar - pre）
    float lastBias; // 上次偏差
    float out;      // PID 输出
    float outMax;   // 输出限幅
};

/**
 * @brief FOC 运行状态结构体
 * 包含当前所有的电气量、角度、PID 控制器状态和控制模式。
 */
typedef struct {
    /* -------- 电流采样与重构 -------- */
    FocCurrentState current;    // 电流 ADC 采样状态
    float Ia, Ib, Ic;           // ABC 三相电流（单位：A）

    /* -------- 坐标变换中间量 -------- */
    float iAlpha, iBeta;        // 电流 αβ 轴分量（Clarke 变换输出）
    float iq, id;               // 电流 dq 轴分量（Park 变换输出）

    /* -------- 电压指令量 -------- */
    float uAlpha, uBeta;        // 电压 αβ 轴分量（逆 Park 变换输出）
    float ua, ub, uc;           // 三相电压（逆 Clarke 变换输出，当前未使用）
    float uq, ud;               // dq 轴电压指令

    /* -------- 电机参数 -------- */
    int   pole_pairs;           // 电机极对数
    int   dir;                  // 旋转方向（1 或 -1）
    float rs;                   // 电机相电阻（单位：Ω）
    float lq;                   // q 轴电感（单位：H）
    float ld;                   // d 轴电感（单位：H）

    /* -------- 角度 & 速度 -------- */
    float mechanicalAngle;      // 当前控制使用的机械角度（单位：rad）
    float electricalAngle;      // 当前控制使用的电角度（未修正/保留）
    float correctedAngle;       // 当前控制使用的修正电角度
    float zeroOffset;           // 零电角度偏移（对准时标定得到）
    float sensoredMechanicalAngle;    // 编码器机械角度（单位：rad）
    float sensoredCorrectedAngle;     // 编码器修正电角度（单位：rad）
    float sensorlessElectricalAngle;  // SMO/PLL 电角度（单位：rad）
    float sensorlessMechanicalSpeed;  // SMO/PLL 换算机械转速（单位：rpm）
    float sensorlessOpenLoopAngle;    // 无感开环虚拟电角度（单位：rad）
    uint8_t sensorMode;               // 当前反馈来源（FocSensorMode_TypeDef）

    /* -------- 无感 I/F 启动 -------- */
    uint8_t sensorlessIfState;         // 无感 I/F 启动状态（FocSensorlessIFState_TypeDef）
    uint16_t sensorlessIfAlignCount;   // I/F 对齐计数
    uint16_t sensorlessIfLockCount;    // SMO/PLL 连续满足切换条件计数
    float sensorlessIfAngle;           // I/F 虚拟电角度（单位：rad）
    float sensorlessIfSpeed;           // I/F 虚拟机械转速（单位：rpm，带符号）
    float sensorlessIfIq;              // I/F 启动 Iq 目标（单位：A，带符号）
    float sensorlessIfId;              // I/F 对齐 Id 目标（单位：A）

    /* -------- 电流环 PID -------- */
    struct PI_Struct idPID;     // d 轴电流 PI 控制器
    struct PI_Struct iqPID;     // q 轴电流 PI 控制器
    float tariq;                // q 轴电流目标值
    float tarid;                // d 轴电流目标值
    float tariqMax;             // Iq 参考电流上限（单位：A），防止过流

    /* -------- 速度环相关 -------- */
    float speedLastAngle;       // 上次电角度（用于差分法速度计算）
    float speed;                // 当前实际转速（单位：rpm）
    float speedDir;             // 速度方向符号（1 或 -1）
    float tar_speed;            // 速度目标值（单位：rpm）
    struct PI_Struct speedPID;  // 速度 PI 控制器

    /* -------- 位置环相关 -------- */
    struct PI_Struct positionPID;   // 位置 PD 控制器
    float position;                 // 当前积分位置
    float tarPosition;              // 位置目标值

    uint8_t ctrolmode;              // 当前控制模式（CtrolMode_TypeDef）
    void (*setPwmCallback)(float pwmA, float pwmB, float pwmC); // PWM 设置回调函数指针
} FocState;

typedef FocState *PFocState;

/* =================== 外部全局变量 =================== */
extern volatile int g_motorAdValues[3];      // ADC 三相电流原始值
extern volatile uint16_t g_ADoffest[3];      // ADC 三相偏置值
extern PFocState g_pMotor;                  // 电机 FOC 对象指针
extern int cnt;

/* =================== 函数声明 =================== */
void AngleInitZeroOffset(float *zeroOffset, float *correctedElecAngle);
void FocContorl(PFocState pFOC, PSVpwm_State PSVpwm);
void getAdoffset(void);
float NormalizeAngle(float angle);
void FOC_SetSensorMode(PFocState pFOC, FocSensorMode_TypeDef mode);
FocSensorMode_TypeDef FOC_GetSensorMode(PFocState pFOC);

#ifdef __cplusplus
}
#endif

#endif // _LOS_FOC_H
