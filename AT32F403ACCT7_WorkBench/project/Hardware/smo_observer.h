#ifndef __SMO_OBSERVER_H
#define __SMO_OBSERVER_H

#include "at32f403a_407.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 滑模观测器（SMO）配置结构体
 * 包含电机参数、滑模增益、滤波系数和 PLL 锁相环参数。
 */
typedef struct {
    float rs;               /**< 电机相电阻（单位：Ω） */
    float ls;               /**< 等效电感 Ls = (Lq + Ld) / 2（单位：H） */
    float ts;               /**< 采样周期（单位：s） */
    float k_slide;          /**< 滑模切换增益 */
    float e_lpf_alpha;      /**< 反电势低通滤波系数（0~1，越大截止频率越高） */
    float pll_kp;           /**< PLL 比例增益 */
    float pll_ki;           /**< PLL 积分增益 */
} SmoObserverConfig;

/**
 * @brief 滑模观测器（SMO）状态结构体
 * 实现基于滑模观测器的永磁同步电机无位置传感器控制。
 *
 * 基本原理：
 * - 构建电流观测模型，利用滑模面（电流误差）注入校正量
 * - 校正量经低通滤波后得到反电势估计值
 * - PLL 锁相环从反电势中提取转子角度和速度
 *
 * 相比传统的 atan2 + 差分求速的方法，PLL 方式：
 * 1. 角度/速度估计更平滑，抗噪性更好
 * 2. 速度不需要额外的低通滤波，减少相位延迟
 * 3. 在低速和稳态下跟踪精度更高
 */
typedef struct {
    SmoObserverConfig cfg;  /**< 观测器配置参数 */

    /* == 电流观测器状态 == */
    float iAlphaHat;        /**< 当前 α 轴电流估计值 */
    float iBetaHat;         /**< 当前 β 轴电流估计值 */
    float zAlpha;           /**< α 轴滑模注入量（等效反电势未滤波） */
    float zBeta;            /**< β 轴滑模注入量 */
    float eAlpha;           /**< α 轴反电势估计值（z 经低通滤波后） */
    float eBeta;            /**< β 轴反电势估计值 */

    /* == PLL 锁相环状态（替代 atan2 + 速度低通） == */
    float pllTheta;         /**< PLL 估计的电角度（单位：rad） */
    float pllOmega;         /**< PLL 估计的电角速度（单位：rad/s） */
    float pllIntegral;      /**< PLL 积分器累积值 */

    /* == 输出量（兼容旧接口）== */
    float angle;            /**< 最终估计电角度（rad），等于 pllTheta */
    float speed;            /**< 最终估计电角速度（rad/s），等于 pllOmega */

    uint8_t valid;          /**< 观测器是否已有效（首次更新后置 1） */
} SmoObserver;

/** 全局 SMO 观测器实例 */
extern SmoObserver g_smoObserver;

/* =================== 函数接口 =================== */
void SMO_Init(SmoObserver *smo, const SmoObserverConfig *cfg);
void SMO_Reset(SmoObserver *smo);
void SMO_Update(SmoObserver *smo, float uAlpha, float uBeta, float iAlpha, float iBeta);

#ifdef __cplusplus
}
#endif

#endif /* __SMO_OBSERVER_H */
