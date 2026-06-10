#include "mt6701.h"
#include "foc_config.h"

/* 片选宏 */
#define MT6701_CS_LOW(gpio, pin)   gpio_bits_reset(gpio, pin)
#define MT6701_CS_HIGH(gpio, pin)  gpio_bits_set(gpio, pin)

/******************************************************************************
 * 函数名称：Mt6701SpiReadByte
 * 功能描述：通过 SPI 从 MT6701 编码器读取一个字节。
 *           发送 0x00 并从 MISO 读取返回数据。
 * 输入参数：encoder - MT6701 编码器对象指针
 * 返回值：读取到的字节数据；超时返回 0
 ******************************************************************************/
static uint8_t Mt6701SpiReadByte(Mt6701_t *encoder)
{
    uint16_t retry = 0;

    while (spi_i2s_flag_get(encoder->spix, SPI_I2S_TDBE_FLAG) == RESET) {
        if (++retry > 1000) return 0;
    }
    spi_i2s_data_transmit(encoder->spix, 0x00);

    retry = 0;
    while (spi_i2s_flag_get(encoder->spix, SPI_I2S_RDBF_FLAG) == RESET) {
        if (++retry > 1000) return 0;
    }
    return spi_i2s_data_receive(encoder->spix);
}

/******************************************************************************
 * 函数名称：Mt6701GetAngle
 * 功能描述：读取 MT6701 磁编码器的机械角度。
 *
 * 通信协议：
 *   - SPI 读取 3 字节数据，提取高 14 位作为角度值
 *   - 14-bit 分辨率，对应 16384 个计数/圈
 *   - 结果转换为弧度（0~2π）
 *
 * 输入参数：encoder - MT6701 编码器对象指针
 * 返回值：机械角度（单位：rad），范围 [0, 2π)
 ******************************************************************************/
float Mt6701GetAngle(Mt6701_t *encoder)
{
    uint8_t raw1, raw2, raw3;
    uint32_t rawData;
    float angleDeg;

    MT6701_CS_LOW(encoder->csGpio, encoder->csPin);

    /* 清空 SPI RX FIFO，防止上次残留数据导致字节错位 */
    while (spi_i2s_flag_get(encoder->spix, SPI_I2S_RDBF_FLAG) != RESET) {
        spi_i2s_data_receive(encoder->spix);
    }

    raw1 = Mt6701SpiReadByte(encoder);
    raw2 = Mt6701SpiReadByte(encoder);
    raw3 = Mt6701SpiReadByte(encoder);

    MT6701_CS_HIGH(encoder->csGpio, encoder->csPin);

    /* 提取 14-bit 角度数据（高 14 位有效） */
    rawData = ((raw1 << 16) | (raw2 << 8) | raw3) >> 10;

    /* 转换为角度：计数 → 度 → 弧度 */
    angleDeg = ((float)rawData * 360.0f) / 16384.0f;
    encoder->angle = angleDeg * FOC_2PI / 360.0f;

    return encoder->angle;
}

/* 默认 MT6701 编码器全局实例 */
Mt6701_t g_mt6701 = {
    .spix = SPI1,
    .csGpio = GPIOB,
    .csPin = GPIO_PINS_0,
    .angle = 0.0f
};

/******************************************************************************
 * 函数名称：Mt6701GetAngleWrapper
 * 功能描述：MT6701 角度获取的包装函数（使用默认对象）。
 *           供外部模块统一调用，避免直接依赖全局对象。
 * 返回值：机械角度（单位：rad），范围 [0, 2π)
 ******************************************************************************/
float Mt6701GetAngleWrapper(void)
{
    return Mt6701GetAngle(&g_mt6701);
}
