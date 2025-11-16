#ifndef __LED_H
#define __LED_H

#include "at32f403a_407.h"

/* LED 结构体 */
typedef struct led_device {
    const char *name;
    void *port;
    uint32_t pin;
    int state;
} led_device_t;

/* API */
void led_init(led_device_t *led, const char *name, void *port, uint32_t pin);
void led_set(led_device_t *led, int on);
int  led_get(led_device_t *led);

#endif
