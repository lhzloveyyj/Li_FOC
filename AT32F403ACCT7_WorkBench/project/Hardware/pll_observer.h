#ifndef __PLL_OBSERVER_H
#define __PLL_OBSERVER_H

#include "at32f403a_407.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PLL 锁相环配置结构体
 */
typedef struct {
    float kp;       /**< 比例增益 */
    float ki;       /**< 积分增益 */
    float ts;       /**< 采样周期（单位：s） */
} PllConfig;

/**
 * @brief PLL 锁相环状态结构体
 *
 * 用于从正交信号（如反电势）中提取角度和速度。
 * 使用 PI 调节器跟踪输入误差信号，输出平滑的角度/速度估计。
 */
typedef struct {
    PllConfig cfg;          /**< PLL 配置参数 */
    float theta;            /**< 估计角度（单位：rad） */
    float omega;            /**< 估计角速度（单位：rad/s） */
    float integral;         /**< PI 积分器累积值 */
    uint8_t valid;          /**< 是否已初始化有效角度（首次更新后置 1） */
} PllObserver;

/* =================== 函数接口 =================== */
void PLL_Init(PllObserver *pll, const PllConfig *cfg);
void PLL_Reset(PllObserver *pll);
void PLL_SetInitialAngle(PllObserver *pll, float theta);
void PLL_Update(PllObserver *pll, float error);

#ifdef __cplusplus
}
#endif

#endif /* __PLL_OBSERVER_H */
