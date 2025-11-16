#ifndef __MT6701_H
#define __MT6701_H

#include "at32f403a_407.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MT6701 编码器结构体定义
 */
typedef struct {
    spi_type *spix;         // SPI 外设句柄
    gpio_type *csGpio;      // 片选 GPIO 端口
    uint16_t csPin;         // 片选 GPIO 引脚
    float angle;            // 当前角度（单位：rad）
} Mt6701_t;

/* 默认编码器对象（外部可直接使用） */
extern Mt6701_t g_mt6701;

/**
 * @brief 获取当前机械角度（单位：rad）
 * @param encoder 编码器对象指针
 * @return 角度值 [0, 2π)
 */
float Mt6701GetAngle(Mt6701_t *encoder);

/**
 * @brief 默认包装函数（封装默认 g_mt6701）
 * @return 角度值 [0, 2π)
 */
float Mt6701GetAngleWrapper(void);

#ifdef __cplusplus
}
#endif

#endif // _LOS_MT6701_H
