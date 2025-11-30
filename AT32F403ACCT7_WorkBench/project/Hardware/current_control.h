#ifndef __PID_H
#define __PID_H

#include "at32f403a_407.h"              // Device header
#include "FOC.h"

void CurrentPIControlID(PFocState pFOC);
void CurrentPIControlIQ(PFocState pFOC);
void SetCurrentPIDTar(PFocState pFOC,float tarid,float tariq);
void SetCurrentPIDParams(PFocState pFOC,float kp,float ki,float kd,float outMax);

#endif
