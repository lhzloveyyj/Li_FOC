#include "smo_observer.h"
#include "foc_config.h"
#include "FOC.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

SmoObserver g_smoObserver;

/******************************************************************************
 * SMO 内部辅助函数
 ******************************************************************************/

static float smo_limit(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

/**
 * @brief 饱和函数（代替硬符号函数）
 * 使用连续饱和函数替代 sign() 不连续切换，减小抖振。
 */
static float smo_sat(float value)
{
    return smo_limit(value, -1.0f, 1.0f);
}

/******************************************************************************
 * 函数名称：SMO_Init
 * 功能描述：初始化滑模观测器。
 *           - 清空所有状态量
 *           - 复制配置参数并做安全限幅
 *           - 初始化 PLL 锁相环
 * 输入参数：smo - SMO 对象指针
 *           cfg - SMO 配置参数指针
 ******************************************************************************/
void SMO_Init(SmoObserver *smo, const SmoObserverConfig *cfg)
{
    if ((smo == NULL) || (cfg == NULL)) {
        return;
    }

    memset(smo, 0, sizeof(*smo));
    smo->cfg = *cfg;

    /* 参数兜底/限幅，避免 Flash 参数异常导致 ISR 中除零或滤波发散 */
    smo->cfg.ls = fabsf(smo->cfg.ls) > FOC_EPSILON ? smo->cfg.ls : FOC_SMO_LS;
    smo->cfg.ts = fabsf(smo->cfg.ts) > FOC_EPSILON ? smo->cfg.ts : FOC_SMO_TS;
    smo->cfg.e_lpf_alpha = smo_limit(smo->cfg.e_lpf_alpha, 0.0f, 1.0f);

    /* 初始化 PLL */
    PLL_Init(&smo->pll, &smo->cfg.pll);
}

/******************************************************************************
 * 函数名称：SMO_Reset
 * 功能描述：重置观测器状态（保留配置参数）。
 *           在电机重新启动或切换控制模式时调用。
 * 输入参数：smo - SMO 对象指针
 ******************************************************************************/
void SMO_Reset(SmoObserver *smo)
{
    SmoObserverConfig cfg;

    if (smo == NULL) {
        return;
    }

    cfg = smo->cfg;
    memset(smo, 0, sizeof(*smo));
    smo->cfg = cfg;

    /* 重新初始化 PLL */
    PLL_Init(&smo->pll, &smo->cfg.pll);
}

/******************************************************************************
 * 函数名称：SMO_Update
 * 功能描述：SMO 核心更新函数（每个 PWM 周期调用一次）。
 *
 * 算法流程：
 *   1. 计算电流观测误差：err = i_hat - i
 *   2. 滑模注入量：z = k_slide * sat(err / band)
 *   3. 电流观测器离散积分：
 *        i_hat += Ts/Ls * (u - Rs*i_hat - e_hat - z)
 *   4. 反电势低通滤波：e += alpha * (z - e)
 *   5. 从反电势计算 PLL 误差，调用 PLL 模块提取角度和速度
 *
 * 相比 atan2 + 差分求速，PLL 方式更平滑、抗噪性更好。
 * 首次更新时使用 atan2 初始化 PLL 角度。
 *
 * 输入参数：smo    - SMO 对象指针
 *           uAlpha - α 轴电压（单位：V）
 *           uBeta  - β 轴电压（单位：V）
 *           iAlpha - α 轴电流（单位：A）
 *           iBeta  - β 轴电流（单位：A）
 ******************************************************************************/
void SMO_Update(SmoObserver *smo, float uAlpha, float uBeta,
                float iAlpha, float iBeta)
{
    float errAlpha;
    float errBeta;
    float invLs;
    float pllErr;

    if (smo == NULL) {
        return;
    }

    /* ============================================================
     * 第一步：电流观测误差计算
     * err = i_hat - i（估计值 - 实际值）
     * ============================================================ */
    errAlpha = smo->iAlphaHat - iAlpha;
    errBeta = smo->iBetaHat - iBeta;

    /* ============================================================
     * 第二步：滑模注入量计算
     * 使用饱和函数 sat() 代替 sign()，减小抖振。
     * z = k_slide * sat(err / band)
     * 当 |err| < band 时，注入量在 ±k_slide 之间线性过渡。
     * ============================================================ */
    float band = (fabsf(FOC_SMO_CURRENT_ERR_BAND) > FOC_EPSILON)
                 ? FOC_SMO_CURRENT_ERR_BAND : 1.0f;
    smo->zAlpha = smo->cfg.k_slide * smo_sat(errAlpha / band);
    smo->zBeta  = smo->cfg.k_slide * smo_sat(errBeta  / band);

    /* ============================================================
     * 第三步：电流观测器离散积分
     * 电流模型：di/dt = (u - Rs*i_hat - e_hat - z) / Ls
     * 一阶欧拉离散：i_hat += Ts * di/dt
     * ============================================================ */
    invLs = 1.0f / smo->cfg.ls;
    smo->iAlphaHat += smo->cfg.ts * invLs
                      * (uAlpha - smo->cfg.rs * smo->iAlphaHat
                         - smo->eAlpha - smo->zAlpha);
    smo->iBetaHat  += smo->cfg.ts * invLs
                      * (uBeta  - smo->cfg.rs * smo->iBetaHat
                         - smo->eBeta  - smo->zBeta);

    /* ============================================================
     * 第四步：反电势低通滤波
     * 滑模注入量 z 中包含高频开关噪声，经一阶 LPF 后得到平滑的反电势。
     * e += e_lpf_alpha * (z - e)
     * 截止频率由 e_lpf_alpha 决定：fc = alpha / (2*pi*Ts)
     * ============================================================ */
    smo->eAlpha += smo->cfg.e_lpf_alpha * (smo->zAlpha - smo->eAlpha);
    smo->eBeta  += smo->cfg.e_lpf_alpha * (smo->zBeta  - smo->eBeta);

    /* ============================================================
     * 第五步：从反电势计算 PLL 误差，调用 PLL 锁相环
     *
     * 反电势矢量：E = [eAlpha, eBeta] = E_mag * [-sin(θ), cos(θ)]
     * 所以真实角度 θ = atan2(-eAlpha, eBeta)
     *
     * PLL 误差检测（等效于反电势矢量和估计矢量的叉积）：
     *   ε = -eAlpha * cos(θ̂) - eBeta * sin(θ̂)
     *     = E_mag * sin(θ - θ̂)
     *     ≈ E_mag * (θ - θ̂) （小误差时线性化）
     *
     * 归一化去除幅值影响：
     *   ε_norm = ε / |E|（当 |E| > 阈值时）
     *
     * 首次更新时用 atan2 初始化 PLL 角度，之后由 PLL 自主跟踪。
     * ============================================================ */

    /* 计算反电势幅值 */
    float eMag = sqrtf(smo->eAlpha * smo->eAlpha + smo->eBeta * smo->eBeta);

    /* PLL 误差计算 */
    pllErr = -smo->eAlpha * cosf(smo->pll.theta)
             - smo->eBeta  * sinf(smo->pll.theta);

    /* 幅值归一化（使 PLL 增益与转速无关） */
    if (eMag > FOC_EPSILON) {
        pllErr /= eMag;
    }

    /* 首次更新：用 atan2 初始化 PLL 角度 */
    if (smo->pll.valid == 0U) {
        float angle = atan2f(-smo->eAlpha, smo->eBeta);
        PLL_SetInitialAngle(&smo->pll, NormalizeAngle(angle));
    }

    /* PLL 跟踪更新 */
    PLL_Update(&smo->pll, pllErr);

    /* 同步输出 */
    smo->angle = smo->pll.theta;
    smo->speed = smo->pll.omega;
}
