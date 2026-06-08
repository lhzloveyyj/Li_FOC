#include "position_control.h"
#include "math.h"
#include "stdio.h"
#include "foc_config.h"

/******************************************************************************
 * 函数名称：CalculatePosition
 * 功能描述：通过机械角度差分积分来累计位置。
 *           处理跨 0/2π 的跳变，将角度增量累加到 position 变量。
 *           这样可以得到多圈连续位置（不受角度周期性影响）。
 * 输入参数：pFOC - FOC 状态指针
 ******************************************************************************/
void CalculatePosition(PFocState pFOC)
{
    static float mechanicalAngle_last;
    static int initialized = 0;
    float delta;

    if (!initialized) {
        mechanicalAngle_last = pFOC->mechanicalAngle;
        initialized = 1;
        return;
    }

    delta = pFOC->mechanicalAngle - mechanicalAngle_last;

    /* 角度跨周期修正 */
    if (delta >  FOC_PI) delta -= FOC_2PI;
    if (delta < -FOC_PI) delta += FOC_2PI;

    pFOC->position += pFOC->speedDir * delta;
    mechanicalAngle_last = pFOC->mechanicalAngle;
}

/******************************************************************************
 * 函数名称：PositionPDControl
 * 功能描述：位置 PD 控制。
 *           位置环输出作为速度环的目标值（即串联控制结构）。
 *           使用 PD（无 I）来避免位置超调。
 * 输入参数：pFOC - FOC 状态指针
 ******************************************************************************/
void PositionPDControl(PFocState pFOC)
{
    float outMax;
    float absBias;
    float slowdownDistance;

    pFOC->positionPID.pre = pFOC->position;
    pFOC->positionPID.tar = pFOC->tarPosition;

    pFOC->positionPID.bias = pFOC->positionPID.tar - pFOC->positionPID.pre;

    outMax = fabs(pFOC->positionPID.outMax);
    absBias = fabs(pFOC->positionPID.bias);

    if (outMax > FOC_EPSILON && fabs(pFOC->positionPID.kp) > FOC_EPSILON) {
        slowdownDistance = outMax / fabs(pFOC->positionPID.kp);

        if (absBias > slowdownDistance) {
            pFOC->positionPID.out = (pFOC->positionPID.bias > 0.0f) ? outMax : -outMax;
        } else {
            pFOC->positionPID.out = pFOC->positionPID.kp * pFOC->positionPID.bias
                                    + pFOC->positionPID.kd
                                      * (pFOC->positionPID.bias - pFOC->positionPID.lastBias);
        }
    } else {
        pFOC->positionPID.out = pFOC->positionPID.kp * pFOC->positionPID.bias
                                + pFOC->positionPID.kd
                                  * (pFOC->positionPID.bias - pFOC->positionPID.lastBias);
    }
    pFOC->positionPID.lastBias = pFOC->positionPID.bias;

    if (pFOC->positionPID.out > outMax) {
        pFOC->positionPID.out = outMax;
    }
    if (pFOC->positionPID.out < -outMax) {
        pFOC->positionPID.out = -outMax;
    }
}

/******************************************************************************
 * 函数名称：SetPositionPIDTar
 * 功能描述：设置位置环目标值。
 * 输入参数：pFOC        - FOC 状态指针
 *           tarposition - 目标位置
 ******************************************************************************/
void SetPositionPIDTar(PFocState pFOC, float tarposition)
{
    pFOC->tarPosition = tarposition;
}

/******************************************************************************
 * 函数名称：SetPositionPIDParams
 * 功能描述：设置位置环 PID 参数。
 * 输入参数：pFOC   - FOC 状态指针
 *           kp     - 比例系数
 *           ki     - 积分系数（预留）
 *           kd     - 微分系数
 *           outMax - 输出最大值
 ******************************************************************************/
void SetPositionPIDParams(PFocState pFOC, float kp, float ki, float kd, float outMax)
{
    pFOC->positionPID.kp = kp;
    pFOC->positionPID.ki = ki;
    pFOC->positionPID.kd = kd;
    pFOC->positionPID.outMax = outMax;
}
