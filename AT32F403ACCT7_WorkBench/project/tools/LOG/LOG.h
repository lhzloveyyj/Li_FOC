#ifndef __LOG_H
#define __LOG_H

#include "at32f403a_407.h"              // Device header

uint16_t PackFloatArrayToBytes(const float *input, uint8_t float_count, uint8_t *output);
void usb_printf(const char *fmt, ...);

#endif
