#ifndef __POSITION_CONTROL_H
#define __POSITION_CONTROL_H

#include "at32f403a_407.h"
#include "FOC.h"

/* =================== 位置环函数声明 =================== */

/**
 * @brief 积分方式计算位置（通过机械角度差分累加）
 * @param pFOC FOC 状态指针
 */
void CalculatePosition(PFocState pFOC);

/**
 * @brief 设置位置目标值
 * @param pFOC        FOC 状态指针
 * @param tarposition 目标位置
 */
void SetPositionPIDTar(PFocState pFOC, float tarposition);

/**
 * @brief 设置位置环 PD 参数
 * @param pFOC   FOC 状态指针
 * @param kp     比例系数
 * @param ki     积分系数（预留）
 * @param kd     微分系数
 * @param outMax 输出最大值
 */
void SetPositionPIDParams(PFocState pFOC, float kp, float ki, float kd, float outMax);

/**
 * @brief 位置 PD 控制
 * @param pFOC FOC 状态指针
 */
void PositionPDControl(PFocState pFOC);

#endif
