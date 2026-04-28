#include "flash_ops.h"
#include "flash.h"
#include <string.h>
#include <stdio.h>

#define FOC_FLASH_DEBUG  1

#if FOC_FLASH_DEBUG
#define FOC_DBG(...)   printf(__VA_ARGS__)
#else
#define FOC_DBG(...)
#endif

/* 参数缓冲区 */
foc_params_t g_params;
foc_params_t g_readback;

void foc_params_set_defaults(foc_params_t *p)
{
    if (p == NULL) {
        return;
    }

    memset(p, 0, sizeof(*p));
    p->elec_offset = 0.0f;
    p->pole_pairs = 11;
    p->reserved1 = 0.0f;
    p->dir = 1;
    p->speeddir = 1;

    /* 默认电机模型参数：
     * Rs = 0.198 R, Lq = 74 uH, Ld = 40 uH。
     * 单位保持为协议层直接传输的 float 单位：R/ohm 和 H。
     */
    p->rs = 0.198f;
    p->lq = 0.000074f;
    p->ld = 0.000040f;
}

/* ========== CRC32 ========== */
uint32_t crc32_compute(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }

    return ~crc;
}

/* ========== 写入参数 ========== */
void foc_params_save(foc_params_t *p)
{
    foc_params_block_t block;
    memcpy(&block.params, p, sizeof(foc_params_t));
    block.crc = crc32_compute((uint8_t*)&block.params, sizeof(foc_params_t));

    FOC_DBG("Saving Params, size=%d bytes\n", sizeof(block));

    uint8_t *raw = (uint8_t*)&block;

    flash_unlock();
    flash_sector_erase(FOC_PARAMS_FLASH_ADDR);

    /* 半字写入 */
    for (uint32_t i = 0; i < sizeof(block); i += 2)
    {
        uint16_t half = raw[i] | (raw[i+1] << 8);
        flash_halfword_program(FOC_PARAMS_FLASH_ADDR + i, half);

        FOC_DBG("  0x%08lX : 0x%04X\n", (uint32_t)(FOC_PARAMS_FLASH_ADDR + i), half);
    }

    flash_lock();

    FOC_DBG("Save Done.\n\n");
}

/* ========== 读取参数 ========== */
/* 返回：1 = OK，0 = CRC 错误 */
int foc_params_load(foc_params_t *p)
{
    foc_params_block_t block;
    foc_params_legacy_block_t legacy_block;
    uint8_t *raw = (uint8_t*)&block;

    /* 半字读取 */
    for (uint32_t i = 0; i < sizeof(block); i += 2)
    {
        uint16_t half = *(uint16_t*)(FOC_PARAMS_FLASH_ADDR + i);
        raw[i]   = half & 0xFF;
        raw[i+1] = (half >> 8) & 0xFF;
    }

    uint32_t calc_crc = crc32_compute((uint8_t*)&block.params, sizeof(foc_params_t));

    if (calc_crc != block.crc)
    {
        /* 如果新结构 CRC 不匹配，再按旧结构试读一次。
         * 这样已出厂/已调零的板子升级固件后，不会因为新增 Rs/Lq/Ld 丢掉旧参数。
         */
        uint8_t *legacy_raw = (uint8_t*)&legacy_block;
        for (uint32_t i = 0; i < sizeof(legacy_block); i += 2)
        {
            uint16_t half = *(uint16_t*)(FOC_PARAMS_FLASH_ADDR + i);
            legacy_raw[i]   = half & 0xFF;
            legacy_raw[i+1] = (half >> 8) & 0xFF;
        }

        uint32_t legacy_crc = crc32_compute((uint8_t*)&legacy_block.params, sizeof(foc_params_legacy_t));
        if (legacy_crc == legacy_block.crc)
        {
            /* 旧结构能通过 CRC 时，先填默认值，再覆盖旧结构中真实存在的字段。 */
            foc_params_set_defaults(p);
            p->elec_offset = legacy_block.params.elec_offset;
            p->pole_pairs = legacy_block.params.pole_pairs;
            p->reserved1 = legacy_block.params.reserved1;
            p->dir = legacy_block.params.dir;
            p->speeddir = legacy_block.params.speeddir;
            FOC_DBG("Legacy params loaded, motor Rs/Lq/Ld use defaults.\n");
            return 1;
        }

        foc_params_set_defaults(p);
        FOC_DBG("CRC ERROR! Flash data invalid, using defaults.\n");
        return 0;   // fail!
    }

    memcpy(p, &block.params, sizeof(foc_params_t));

    FOC_DBG("Load OK.\n");
    return 1;
}

/* ========== 测试函数 ========== */
void foc_params_test(void)
{
    printf("sizeof(foc_params_t) = %lu\n", (uint32_t)sizeof(foc_params_t));

    /* 填充测试值 */
    g_params.elec_offset = 66.6f;
    g_params.pole_pairs  = 11;
    g_params.reserved1   = 0.123f;
    g_params.dir         = 1;
    g_params.speeddir    = 1;
    g_params.rs          = 0.198f;
    g_params.lq          = 0.000074f;
    g_params.ld          = 0.000040f;

    foc_params_save(&g_params);

    printf("Try loading...\n");

    if (!foc_params_load(&g_readback))
    {
        printf("Flash invalid, using default!\n");
        return;
    }

    printf("Readback:\n");
    printf("  offset    = %.3f\n", g_readback.elec_offset);
    printf("  pole      = %ld\n",  g_readback.pole_pairs);
    printf("  reserved1 = %.3f\n", g_readback.reserved1);
    printf("  dir       = %ld\n",  g_readback.dir);
    printf("  speeddir  = %ld\n",  g_readback.speeddir);
    printf("  rs        = %.6f\n", g_readback.rs);
    printf("  lq        = %.6f\n", g_readback.lq);
    printf("  ld        = %.6f\n", g_readback.ld);
}
