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

        FOC_DBG("  0x%08X : 0x%04X\n", FOC_PARAMS_FLASH_ADDR + i, half);
    }

    flash_lock();

    FOC_DBG("Save Done.\n\n");
}

/* ========== 读取参数 ========== */
/* 返回：1 = OK，0 = CRC 错误 */
int foc_params_load(foc_params_t *p)
{
    foc_params_block_t block;
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
        FOC_DBG("CRC ERROR! Flash data invalid.\n");
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

    foc_params_save(&g_params);

    printf("Try loading...\n");

    if (!foc_params_load(&g_readback))
    {
        printf("Flash invalid, using default!\n");
        return;
    }

    printf("Readback:\n");
    printf("  offset    = %.3f\n", g_readback.elec_offset);
    printf("  pole      = %d\n",   g_readback.pole_pairs);
    printf("  reserved1 = %.3f\n", g_readback.reserved1);
    printf("  dir       = %d\n",   g_readback.dir);
    printf("  speeddir  = %d\n",   g_readback.speeddir);
}
