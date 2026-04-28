#include "smo_observer.h"
#include "foc_config.h"
#include "FOC.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

SmoObserver g_smoObserver;

/* 小工具函数只在 SMO 内部使用，避免把调参保护逻辑散到控制环里。 */
static float smo_limit(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static float smo_sat(float value)
{
    /* 滑模项使用饱和函数而不是硬 sign，减少电流估算误差过零时的抖振。 */
    return smo_limit(value, -1.0f, 1.0f);
}

static float smo_angle_delta(float now, float last)
{
    float delta = now - last;

    /* 角度跨 0/2pi 时要按最短路径求差，否则速度会出现一个巨大尖峰。 */
    if (delta > FOC_PI) {
        delta -= FOC_2PI;
    } else if (delta < -FOC_PI) {
        delta += FOC_2PI;
    }

    return delta;
}

void SMO_Init(SmoObserver *smo, const SmoObserverConfig *cfg)
{
    if ((smo == NULL) || (cfg == NULL)) {
        return;
    }

    memset(smo, 0, sizeof(*smo));
    smo->cfg = *cfg;

    /* 这里做一次参数兜底/限幅，避免 flash 参数异常导致 ISR 中除零或滤波发散。 */
    smo->cfg.ls = fabsf(smo->cfg.ls) > FOC_EPSILON ? smo->cfg.ls : FOC_SMO_LS;
    smo->cfg.ts = fabsf(smo->cfg.ts) > FOC_EPSILON ? smo->cfg.ts : FOC_SMO_TS;
    smo->cfg.e_lpf_alpha = smo_limit(smo->cfg.e_lpf_alpha, 0.0f, 1.0f);
    smo->cfg.speed_lpf_alpha = smo_limit(smo->cfg.speed_lpf_alpha, 0.0f, 1.0f);
}

void SMO_Reset(SmoObserver *smo)
{
    SmoObserverConfig cfg;

    if (smo == NULL) {
        return;
    }

    cfg = smo->cfg;

    /* Reset 只清观测状态，不清配置；调参后可保留新的 Rs/Ls/滤波参数。 */
    memset(smo, 0, sizeof(*smo));
    smo->cfg = cfg;
}

void SMO_Update(SmoObserver *smo, float uAlpha, float uBeta, float iAlpha, float iBeta)
{
    float errAlpha;
    float errBeta;
    float invLs;
    float angle;
    float delta;

    if (smo == NULL) {
        return;
    }

    errAlpha = smo->iAlphaHat - iAlpha;
    errBeta = smo->iBetaHat - iBeta;

    /* 滑模注入量 zAlpha/zBeta：
     * 电流估算误差越大，注入越接近 +/-k_slide；误差在 band 内线性变化。
     */
    smo->zAlpha = smo->cfg.k_slide * smo_sat(errAlpha / (fabsf(FOC_SMO_CURRENT_ERR_BAND) > FOC_EPSILON ? FOC_SMO_CURRENT_ERR_BAND : 1.0f));
    smo->zBeta = smo->cfg.k_slide * smo_sat(errBeta / (fabsf(FOC_SMO_CURRENT_ERR_BAND) > FOC_EPSILON ? FOC_SMO_CURRENT_ERR_BAND : 1.0f));

    invLs = 1.0f / smo->cfg.ls;

    /* 电流模型离散积分：
     * di/dt = (u - R*i_hat - e_hat - z) / L
     * 当前工程先把 Lq/Ld 平均成等效 Ls，用于后台观测调试。
     */
    smo->iAlphaHat += smo->cfg.ts * invLs * (uAlpha - smo->cfg.rs * smo->iAlphaHat - smo->eAlpha - smo->zAlpha);
    smo->iBetaHat += smo->cfg.ts * invLs * (uBeta - smo->cfg.rs * smo->iBetaHat - smo->eBeta - smo->zBeta);

    /* 滑模注入量经过低通后近似反电势，角度由反电势矢量计算。 */
    smo->eAlpha += smo->cfg.e_lpf_alpha * (smo->zAlpha - smo->eAlpha);
    smo->eBeta += smo->cfg.e_lpf_alpha * (smo->zBeta - smo->eBeta);

    angle = atan2f(-smo->eAlpha, smo->eBeta);
    angle = NormalizeAngle(angle);

    if (smo->valid != 0U) {
        /* 第二次更新后才计算速度，避免 lastAngle 尚未有效时产生假速度。 */
        delta = smo_angle_delta(angle, smo->lastAngle);
        smo->speedRaw = delta / smo->cfg.ts;
        smo->speed += smo->cfg.speed_lpf_alpha * (smo->speedRaw - smo->speed);
    } else {
        smo->valid = 1U;
    }

    smo->lastAngle = angle;
    smo->angle = angle;
}
