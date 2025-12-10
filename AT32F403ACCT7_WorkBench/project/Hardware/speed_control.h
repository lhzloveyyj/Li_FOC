#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include "at32f403a_407.h"              // Device header
#include "FOC.h"
#include "filter.h"

void CalculateSpeed(PFocState pFOC, float dt, PLPF_Speed pSpeedFilter);
void CurrentPIControlID(PFocState pFOC);
void CurrentPIControlIQ(PFocState pFOC);
void SetCurrentPIDTar(PFocState pFOC,float tarid,float tariq);
void SetCurrentPIDParams(PFocState pFOC,float kp,float ki,float kd,float outMax);

#endif
