#ifndef __MT6701_H
#define __MT6701_H

#include "at32f403a_407.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MT6701 磁编码器结构体
 *
 * MT6701 是一款 14-bit 分辨率的磁角度传感器，通信接口为 SPI。
 * 角度数据通过 3 字节 SPI 读取，提取高 14 位，对应 0~16383 计数/圈。
 */
typedef struct {
    spi_type *spix;         /**< SPI 外设句柄 */
    gpio_type *csGpio;      /**< 片选 GPIO 端口 */
    uint16_t csPin;         /**< 片选 GPIO 引脚 */
    float angle;            /**< 当前角度缓存（单位：rad） */
} Mt6701_t;

/** 默认 MT6701 编码器对象 */
extern Mt6701_t g_mt6701;

/**
 * @brief 获取 MT6701 编码器机械角度
 * @param encoder 编码器对象指针
 * @return 机械角度（单位：rad），范围 [0, 2π)
 */
float Mt6701GetAngle(Mt6701_t *encoder);

/**
 * @brief 获取机械角度的便捷包装函数
 * @return 机械角度（单位：rad），范围 [0, 2π)
 */
float Mt6701GetAngleWrapper(void);

#ifdef __cplusplus
}
#endif

#endif /* __MT6701_H */
