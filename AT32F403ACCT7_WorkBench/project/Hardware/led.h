#ifndef __LED_H
#define __LED_H

#include "at32f403a_407.h"

/**
 * @brief LED 设备结构体
 */
typedef struct led_device {
    const char *name;    /**< LED 名称 */
    void *port;          /**< GPIO 端口 */
    uint32_t pin;        /**< GPIO 引脚编号 */
    int state;           /**< 当前状态（0=熄灭，1=点亮） */
} led_device_t;

/* =================== API 接口 =================== */
void led_init(led_device_t *led, const char *name, void *port, uint32_t pin);
void led_set(led_device_t *led, int on);
int  led_get(led_device_t *led);

#endif
