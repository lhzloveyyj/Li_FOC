#ifndef __POSITION_CONTROL_H
#define __POSITION_CONTROL_H

#include "at32f403a_407.h"             
#include "FOC.h"

void CalculatePosition(PFocState pFOC);
void SetPositionPIDTar(PFocState pFOC, float tarposition);
void SetPositionPIDParams(PFocState pFOC,float kp,float ki,float kd,float outMax);
void PositionPDControl(PFocState pFOC);

#endif
