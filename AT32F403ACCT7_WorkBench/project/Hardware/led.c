#include "led.h"

/* 初始化 LED 对象（只记录端口和引脚） */
void led_init(led_device_t *led, const char *name, void *port, uint32_t pin)
{
    led->name  = name;
    led->port  = port;
    led->pin   = pin;
    led->state = 0;
}

/* 设置 LED */
void led_set(led_device_t *led, int on)
{
    if(on)
        gpio_bits_reset(led->port, led->pin);  // 点亮
    else
        gpio_bits_set(led->port, led->pin);    // 熄灭

    led->state = on;
}

/* 获取 LED 状态 */
int led_get(led_device_t *led)
{
    return led->state;
}
