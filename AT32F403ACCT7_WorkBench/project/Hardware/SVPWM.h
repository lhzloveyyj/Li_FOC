#ifndef __SVPWM_H
#define __SVPWM_H

#include "at32f403a_407.h"              // Device header

/**
 * @brief SVPWM 状态结构体
 * 存储 SVPWM 调制所需的中间计算变量和最终三相导通时间。
 */
typedef struct {
    float Ts;           /**< PWM 周期（单位与 K 系数匹配即可） */
    uint8_t sector;     /**< 当前参考电压矢量所在扇区（1~6） */
    float Ta, Tb, Tc;   /**< 三相导通时间（归一化 0~1） */

    float u1, u2, u3;   /**< 中间变量：Clarke 变换后的参考量 */
    float t1, t2, t3;   /**< 相邻非零矢量作用时间 */
    float t4, t5, t6;   /**< 相邻非零矢量作用时间（另一组） */
    float t7;           /**< 零矢量作用时间 */
    float sum;          /**< 非零矢量总时间（用于过调制判断） */
    float k_svpwm;      /**< 过调制时的时间缩放系数 */

    float K;            /**< SVPWM 比例系数：K = sqrt(3) * Ts / Udc */
    int times;          /**< 通用计数器（保留未用） */
} SVpwm_State;

typedef SVpwm_State *PSVpwm_State;

/** 全局 SVPWM 状态对象指针 */
extern PSVpwm_State PSVpwm;

/**
 * @brief SVPWM 计算函数
 * @param PSVpwm   SVPWM 状态指针
 * @param U_alpha  α 轴电压参考值
 * @param U_beta   β 轴电压参考值
 */
void SVpwm(PSVpwm_State PSVpwm, float U_alpha, float U_beta);

#endif
