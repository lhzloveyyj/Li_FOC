#include "pll_observer.h"
#include "foc_config.h"
#include "FOC.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float pll_limit(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

/**
 * @brief 初始化 PLL 锁相环
 */
void PLL_Init(PllObserver *pll, const PllConfig *cfg)
{
    if ((pll == NULL) || (cfg == NULL)) {
        return;
    }

    memset(pll, 0, sizeof(*pll));
    pll->cfg = *cfg;

    /* 参数兜底/限幅 */
    pll->cfg.ts = fabsf(pll->cfg.ts) > FOC_EPSILON ? pll->cfg.ts : 0.0001f;
    pll->cfg.kp = pll_limit(pll->cfg.kp, 0.0f, 10000.0f);
    pll->cfg.ki = pll_limit(pll->cfg.ki, 0.0f, 10000.0f);
}

/**
 * @brief 重置 PLL 状态（保留配置参数）
 */
void PLL_Reset(PllObserver *pll)
{
    PllConfig cfg;

    if (pll == NULL) {
        return;
    }

    cfg = pll->cfg;
    memset(pll, 0, sizeof(*pll));
    pll->cfg = cfg;
}

/**
 * @brief 设置 PLL 初始角度（在首次有效反电势到来时调用）
 *
 * @param pll    PLL 对象指针
 * @param theta  初始角度（rad），需已归一化到 [-π, π]
 */
void PLL_SetInitialAngle(PllObserver *pll, float theta)
{
    if (pll == NULL) {
        return;
    }

    pll->theta    = theta;
    pll->omega    = 0.0f;
    pll->integral = 0.0f;
    pll->valid    = 1U;
}

/**
 * @brief PLL 核心更新函数（每个控制周期调用一次）
 *
 * PI 调节器跟踪误差信号，输出平滑的角度和速度估计。
 * 调用前需确保 PLL 已通过 PLL_SetInitialAngle 设置初始角度（valid != 0）。
 *
 * 算法：
 *   integral += Ki * error * Ts
 *   omega = Kp * error + integral
 *   theta += omega * Ts
 *   theta = normalize(theta)
 *
 * @param pll   PLL 对象指针
 * @param error 归一化后的角度误差（无量纲，≈ sin(θ_real - θ_hat)）
 */
void PLL_Update(PllObserver *pll, float error)
{
    if (pll == NULL || pll->valid == 0U) {
        return;
    }

    /* PI 调节器 */
    pll->integral += pll->cfg.ki * error * pll->cfg.ts;
    pll->omega = pll->cfg.kp * error + pll->integral;

    /* 角度积分 */
    pll->theta += pll->omega * pll->cfg.ts;
    pll->theta = NormalizeAngle(pll->theta);
}
