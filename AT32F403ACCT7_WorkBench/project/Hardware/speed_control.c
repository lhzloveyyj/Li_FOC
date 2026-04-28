#include "speed_control.h"
#include "math.h"
#include "stdio.h"

/******************************************************************************
 * 函数名称：CalculateSpeed
 * 功能描述：通过机械角度差分计算实际速度，再经低通滤波平滑。
 *
 * 算法：
 *   delta = mechanicalAngle - mechanicalAngle_last
 *   并处理 0/2π 跨周期跳变
 *   speed = speedDir * delta / dt
 *   最后经一阶低通滤波输出
 *
 * 输入参数：pFOC         - FOC 状态指针
 *           dt           - 速度计算周期（单位：s）
 *           pSpeedFilter - 速度低通滤波器指针
 ******************************************************************************/
void CalculateSpeed(PFocState pFOC, float dt, PLPF_Speed pSpeedFilter)
{
    static float mechanicalAngle_last;
    float angle_diff = (pFOC->mechanicalAngle - mechanicalAngle_last);

    /* 处理跨 0/2π 跳变 */
    if (angle_diff > 3.14159f)
        angle_diff -= 6.28318f;
    else if (angle_diff < -3.14159f)
        angle_diff += 6.28318f;

    /* 计算原始速度 */
    pFOC->speed = pFOC->speedDir * angle_diff / dt;

    /* 低通滤波平滑 */
    LPF_Speed_Update(pSpeedFilter, pFOC->speed, &(pFOC->speed));

    mechanicalAngle_last = pFOC->mechanicalAngle;
}

/******************************************************************************
 * 函数名称：SpeedPIControl
 * 功能描述：速度 PI 控制。
 *           增量式 PI：out += Ki*(bias - lastBias) + Kp*bias
 *           位置环模式时，速度目标由位置环 PID 输出提供。
 * 输入参数：pFOC - FOC 状态指针
 ******************************************************************************/
void SpeedPIControl(PFocState pFOC)
{
    pFOC->speedPID.pre = pFOC->speed;
    pFOC->speedPID.tar = pFOC->tar_speed;

    /* 位置环模式下，速度目标由位置 PD 输出提供 */
    if (g_pMotor->ctrolmode == FOC_POSITION_LOOP) {
        pFOC->speedPID.tar = pFOC->positionPID.out;
    }

    pFOC->speedPID.bias = pFOC->speedPID.tar - pFOC->speedPID.pre;
    pFOC->speedPID.out += pFOC->speedPID.ki
                          * (pFOC->speedPID.bias - pFOC->speedPID.lastBias)
                          + pFOC->speedPID.kp * pFOC->speedPID.bias;
    pFOC->speedPID.lastBias = pFOC->speedPID.bias;

    if (pFOC->speedPID.out > fabs(pFOC->speedPID.outMax)) {
        pFOC->speedPID.out = fabs(pFOC->speedPID.outMax);
    }
    if (pFOC->speedPID.out < -fabs(pFOC->speedPID.outMax)) {
        pFOC->speedPID.out = -fabs(pFOC->speedPID.outMax);
    }
}

/******************************************************************************
 * 函数名称：SetSpeedPIDTar
 * 功能描述：设置速度目标值。
 * 输入参数：pFOC     - FOC 状态指针
 *           tarspeed - 目标速度
 ******************************************************************************/
void SetSpeedPIDTar(PFocState pFOC, float tarspeed)
{
    pFOC->tar_speed = tarspeed;
}

/******************************************************************************
 * 函数名称：SetSpeedPIDParams
 * 功能描述：设置速度环 PID 参数。
 * 输入参数：pFOC   - FOC 状态指针
 *           kp     - 比例系数
 *           ki     - 积分系数
 *           kd     - 微分系数（预留）
 *           outMax - 输出最大值
 ******************************************************************************/
void SetSpeedPIDParams(PFocState pFOC, float kp, float ki, float kd, float outMax)
{
    pFOC->speedPID.kp = kp;
    pFOC->speedPID.ki = ki;
    pFOC->speedPID.kd = kd;
    pFOC->speedPID.outMax = outMax;
}
