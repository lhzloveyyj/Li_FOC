#ifndef __PID_H
#define __PID_H

#include "at32f403a_407.h"
#include "FOC.h"

/* =================== 电流环 PI 控制 =================== */
void CurrentPIControlID(PFocState pFOC);
void CurrentPIControlIQ(PFocState pFOC);
void SetCurrentPIDTar(PFocState pFOC, float tarid, float tariq);
void SetCurrentPIDParams(PFocState pFOC, float kp, float ki, float kd, float outMax);

#endif
