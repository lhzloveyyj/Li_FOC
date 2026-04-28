#ifndef __MOSTEMP_H
#define __MOSTEMP_H

#include "at32f403a_407.h"

float GetMosTemp(void);
float getVbus(void);
float adcToVbus(uint16_t adc);

#endif
