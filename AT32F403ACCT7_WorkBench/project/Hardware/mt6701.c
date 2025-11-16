#include "mt6701.h"
#include "foc_config.h"

#define MT6701_CS_LOW(gpio, pin)   gpio_bits_reset(gpio, pin)
#define MT6701_CS_HIGH(gpio, pin)  gpio_bits_set(gpio, pin)

/**
 * @brief  通过 SPI 向编码器发送并接收一个字节
 * @param  encoder  指向 MT6701 编码器结构体
 * @return 读取的字节数据
 */
static uint8_t Mt6701SpiReadByte(Mt6701_t *encoder)
{
    uint16_t retry = 0;

    while (spi_i2s_flag_get(encoder->spix, SPI_I2S_TDBE_FLAG) == RESET) {
        if (++retry > 10) {
            return 0;
        }
    }

    spi_i2s_data_transmit(encoder->spix, 0x00);

    retry = 0;
    while (spi_i2s_flag_get(encoder->spix, SPI_I2S_RDBF_FLAG) == RESET) {
        if (++retry > 10) {
            return 0;
        }
    }

    return spi_i2s_data_receive(encoder->spix);
}

/**
 * @brief  获取编码器的机械角度（单位：rad）
 * @param  encoder  指向 MT6701 编码器结构体
 * @return 返回机械角度（0 ~ 2π）
 */
float Mt6701GetAngle(Mt6701_t *encoder)
{
    uint8_t raw1, raw2, raw3;
    uint32_t rawData;
    float angleDeg;

    MT6701_CS_LOW(encoder->csGpio, encoder->csPin);

    raw1 = Mt6701SpiReadByte(encoder);
    raw2 = Mt6701SpiReadByte(encoder);
    raw3 = Mt6701SpiReadByte(encoder);

    MT6701_CS_HIGH(encoder->csGpio, encoder->csPin);

    // 提取有效 14 位数据（高位 14bit）
    rawData = ((raw1 << 16) | (raw2 << 8) | raw3) >> 10;

    // 转换为角度（0~360°）
    angleDeg = ((float)rawData * 360.0f) / 16384.0f;

    // 转换为弧度（0~2π）
    encoder->angle = angleDeg * FOC_2PI / 360.0f;

    return encoder->angle;
}

// 默认编码器对象
Mt6701_t g_mt6701 = {
    .spix = SPI1,
    .csGpio = GPIOB,
    .csPin = GPIO_PINS_0,
    .angle = 0.0f
};

/**
 * @brief  获取编码器角度的函数包装器（供外部模块使用）
 * @return 当前机械角度（单位：rad）
 */
float Mt6701GetAngleWrapper(void)
{
    return Mt6701GetAngle(&g_mt6701);
}
