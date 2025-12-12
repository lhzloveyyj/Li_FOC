#include "position_control.h"
#include "math.h"
#include "stdio.h"

void PositionPDControl(PFocState pFOC)
{
    //获取实际值
    pFOC->positionPID.pre = pFOC->mechanicalAngle ;
    //获取目标值
    pFOC->positionPID.tar = pFOC->tarPosition;
    
    //计算偏差
    pFOC->positionPID.bias = pFOC->positionPID.tar - pFOC->positionPID.pre;
    //计算PID输出值
    pFOC->positionPID.out = pFOC->positionPID.kp * pFOC->idPID.bias + pFOC->positionPID.kd * (pFOC->positionPID.bias - pFOC->positionPID.lastBias);
    //保存偏差
    pFOC->positionPID.lastBias = pFOC->positionPID.bias;

    if (pFOC->positionPID.out > fabs(pFOC->positionPID.outMax)) {
        pFOC->positionPID.out = fabs(pFOC->positionPID.outMax);
    }

    if (pFOC->positionPID.out < -fabs(pFOC->positionPID.outMax)) {
        pFOC->positionPID.out = -fabs(pFOC->positionPID.outMax);
    }
    
}


/******************************************************************************
  函数说明：设置位置环PID目标值
  @brief  设置位置环PID控制中角度目标值
  @param  pFOC   指向FOC状态结构体的指针
  @param  tarposition  位置目标值
  @retval 无
******************************************************************************/
void SetPositionPIDTar(PFocState pFOC, float tarposition)
{
    pFOC->tarPosition = tarposition;
}

/******************************************************************************
  函数说明：设置位置环PID控制参数
  @brief  设置位置环PID控制器的比例、积分、微分系数以及输出限幅
  @param  pFOC   指向FOC状态结构体的指针
  @param  kp     比例系数
  @param  ki     积分系数
  @param  kd     微分系数
  @param  outMax PID输出的最大值
  @retval 无
******************************************************************************/
void SetPositionPIDParams(PFocState pFOC,float kp,float ki,float kd,float outMax)
{
    pFOC->positionPID.kp = kp;
    pFOC->positionPID.ki = ki;
    pFOC->positionPID.kd = kd;
    pFOC->positionPID.outMax = outMax;
	
}




