#include "current_control.h"
#include "foc_config.h"
#include "math.h"
#include "stdio.h"

/******************************************************************************
 * 函数名称：CurrentPIControlID
 * 功能描述：D 轴电流 PI 闭环控制。
 *           增量式 PI 算法：
 *           out += Ki*(bias - lastBias) + Kp*bias
 *           输出若超过 outMax 则限幅。
 * 输入参数：pFOC - FOC 状态结构体指针
 ******************************************************************************/
void CurrentPIControlID(PFocState pFOC)
{
    pFOC->idPID.pre = pFOC->id;
    pFOC->idPID.tar = pFOC->tarid;

    pFOC->idPID.bias = pFOC->idPID.tar - pFOC->idPID.pre;
    pFOC->idPID.out += pFOC->idPID.ki * (pFOC->idPID.bias - pFOC->idPID.lastBias)
                       + pFOC->idPID.kp * pFOC->idPID.bias;
    pFOC->idPID.lastBias = pFOC->idPID.bias;

    if (pFOC->idPID.out > fabs(pFOC->idPID.outMax)) {
        pFOC->idPID.out = fabs(pFOC->idPID.outMax);
    }
    if (pFOC->idPID.out < -fabs(pFOC->idPID.outMax)) {
        pFOC->idPID.out = -fabs(pFOC->idPID.outMax);
    }
}

/******************************************************************************
 * 函数名称：CurrentPIControlIQ
 * 功能描述：Q 轴电流 PI 闭环控制。
 *           在速度环/位置环模式下，Q 轴目标值由外环 PID 输出提供。
 *           增量式 PI 算法同上。
 * 输入参数：pFOC - FOC 状态结构体指针
 ******************************************************************************/
void CurrentPIControlIQ(PFocState pFOC)
{
    pFOC->iqPID.pre = pFOC->iq;
    pFOC->iqPID.tar = pFOC->tariq;

    // 外层速度环/位置环输出作为 Q 轴电流目标
    if (g_pMotor->ctrolmode == FOC_SPEED_LOOP
        || g_pMotor->ctrolmode == FOC_POSITION_LOOP) {
        pFOC->iqPID.tar = pFOC->speedPID.out;

        /* tariq 作为速度环电流上限 */
        if (pFOC->tariq > FOC_EPSILON) {
            if (pFOC->iqPID.tar > pFOC->tariq) {
                pFOC->iqPID.tar = pFOC->tariq;
            }
            if (pFOC->iqPID.tar < -pFOC->tariq) {
                pFOC->iqPID.tar = -pFOC->tariq;
            }
        }
    }

    /* tariqMax 是绝对安全上限，所有模式下均生效 */
    if (pFOC->iqPID.tar > pFOC->tariqMax) {
        pFOC->iqPID.tar = pFOC->tariqMax;
    }
    if (pFOC->iqPID.tar < -pFOC->tariqMax) {
        pFOC->iqPID.tar = -pFOC->tariqMax;
    }

    pFOC->iqPID.bias = pFOC->iqPID.tar - pFOC->iqPID.pre;
    pFOC->iqPID.out += pFOC->iqPID.ki * (pFOC->iqPID.bias - pFOC->iqPID.lastBias)
                       + pFOC->iqPID.kp * pFOC->iqPID.bias;
    pFOC->iqPID.lastBias = pFOC->iqPID.bias;

    if (pFOC->iqPID.out > fabs(pFOC->iqPID.outMax)) {
        pFOC->iqPID.out = fabs(pFOC->iqPID.outMax);
    }
    if (pFOC->iqPID.out < -fabs(pFOC->iqPID.outMax)) {
        pFOC->iqPID.out = -fabs(pFOC->iqPID.outMax);
    }
}

/******************************************************************************
 * 函数名称：SetCurrentPIDTar
 * 功能描述：设置电流环 d 轴和 q 轴电流目标值。
 * 输入参数：pFOC  - FOC 状态结构体指针
 *           tarid  - d 轴目标电流（单位：A）
 *           tariq  - q 轴目标电流（单位：A）
 ******************************************************************************/
void SetCurrentPIDTar(PFocState pFOC, float tarid, float tariq)
{
    pFOC->tarid = tarid;
    pFOC->tariq = tariq;
    printf("set tar_id is %lf, tar_iq is %lf\r\n", tarid, tariq);
}

/******************************************************************************
 * 函数名称：SetCurrentPIDParams
 * 功能描述：设置电流环 PI 控制参数（Id/Iq 共用同一组参数）。
 * 输入参数：pFOC   - FOC 状态结构体指针
 *           kp     - 比例系数
 *           ki     - 积分系数
 *           kd     - 微分系数（当前未使用）
 *           outMax - 输出限幅
 ******************************************************************************/
void SetCurrentPIDParams(PFocState pFOC, float kp, float ki, float kd, float outMax)
{
    pFOC->idPID.kp = kp;
    pFOC->idPID.ki = ki;
    pFOC->idPID.kd = kd;
    pFOC->idPID.outMax = outMax;

    pFOC->iqPID.kp = kp;
    pFOC->iqPID.ki = ki;
    pFOC->iqPID.kd = kd;
    pFOC->iqPID.outMax = outMax;

    printf("set kp is %lf, ki is %lf, kd is %lf\r\n", kp, ki, kd);
}
