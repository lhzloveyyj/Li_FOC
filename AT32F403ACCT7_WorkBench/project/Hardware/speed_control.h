#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include "at32f403a_407.h"
#include "FOC.h"
#include "filter.h"

/* =================== 速度环函数声明 =================== */

/**
 * @brief 计算电机实际速度（差分法）
 * @param pFOC         FOC 状态指针
 * @param dt           速度计算周期（单位：s）
 * @param pSpeedFilter 速度低通滤波器指针
 */
void CalculateSpeed(PFocState pFOC, float dt, PLPF_Speed pSpeedFilter);

/**
 * @brief 速度 PI 控制（外环）
 * @param pFOC FOC 状态指针
 */
void SpeedPIControl(PFocState pFOC);

/**
 * @brief 设置速度 PI 参数
 * @param pFOC   FOC 状态指针
 * @param kp     比例系数
 * @param ki     积分系数
 * @param kd     微分系数（预留）
 * @param outMax 输出最大值
 */
void SetSpeedPIDParams(PFocState pFOC, float kp, float ki, float kd, float outMax);

/**
 * @brief 设置速度目标值
 * @param pFOC     FOC 状态指针
 * @param tarspeed 目标速度
 */
void SetSpeedPIDTar(PFocState pFOC, float tarspeed);

#endif
