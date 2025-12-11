#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include "at32f403a_407.h"              // Device header
#include "FOC.h"
#include "filter.h"

void CalculateSpeed(PFocState pFOC, float dt, PLPF_Speed pSpeedFilter);
void SetSpeedPIDParams(PFocState pFOC,float kp,float ki,float kd,float outMax);
void SpeedPIControl(PFocState pFOC);
void SetSpeedPIDTar(PFocState pFOC,float tarspeed);

#endif
