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

/******************************************************************************
 * 函数名称：foc_params_set_defaults
 * 功能描述：将参数结构体设置为默认值。
 *           电机参数使用当前实测值：Rs=0.198Ω, Lq=74µH, Ld=40µH。
 * 输入参数：p - 参数结构体指针
 ******************************************************************************/
void foc_params_set_defaults(foc_params_t *p)
{
    if (p == NULL) return;

    memset(p, 0, sizeof(*p));
    p->elec_offset = 0.0f;
    p->pole_pairs = 11;
    p->reserved1 = 0.0f;
    p->dir = 1;
    p->speeddir = 1;
    p->rs = 0.198f;
    p->lq = 0.000074f;
    p->ld = 0.000040f;
    p->speed_pid_kp = 0.002f;
    p->speed_pid_ki = 0.1f;
    p->speed_pid_out = 10.0f;
    p->position_pid_kp = 1.0f;
    p->position_pid_kd = 0.0001f;
    p->position_pid_out = 400.0f;
}

/******************************************************************************
 * 函数名称：crc32_compute
 * 功能描述：计算 CRC32 校验值（标准 CRC32 算法）。
 * 输入参数：data - 数据指针
 *           len  - 数据长度
 * 返回值：CRC32 校验值
 ******************************************************************************/
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

/******************************************************************************
 * 函数名称：foc_params_save
 * 功能描述：将电机参数写入 Flash（含 CRC32 校验）。
 *           先扇区擦除，再按半字编程写入。
 * 输入参数：p - 参数结构体指针
 ******************************************************************************/
void foc_params_save(foc_params_t *p)
{
    foc_params_block_t block;
    memcpy(&block.params, p, sizeof(foc_params_t));
    block.crc = crc32_compute((uint8_t*)&block.params, sizeof(foc_params_t));

    FOC_DBG("Saving Params, size=%d bytes\n", sizeof(block));

    uint8_t *raw = (uint8_t*)&block;

    flash_unlock();
    flash_sector_erase(FOC_PARAMS_FLASH_ADDR);

    for (uint32_t i = 0; i < sizeof(block); i += 2) {
        uint16_t half = raw[i] | (raw[i + 1] << 8);
        flash_halfword_program(FOC_PARAMS_FLASH_ADDR + i, half);
        FOC_DBG("  0x%08lX : 0x%04X\n", (uint32_t)(FOC_PARAMS_FLASH_ADDR + i), half);
    }

    flash_lock();
    FOC_DBG("Save Done.\n\n");
}

/******************************************************************************
 * 函数名称：foc_params_load
 * 功能描述：从 Flash 读取电机参数。
 *
 * 流程：
 *   1. 按新版结构体从 Flash 半字读取数据
 *   2. 校验 CRC32
 *   3. 如果 CRC 错误，尝试旧版结构体（兼容升级）
 *   4. 旧版也不匹配则返回默认值
 *
 * 返回值：1 - 加载成功，0 - CRC 错误（已填入默认值）
 ******************************************************************************/
int foc_params_load(foc_params_t *p)
{
    foc_params_block_t block;
    uint8_t *raw = (uint8_t*)&block;

    /* 半字读取全部 Flash 内容 */
    for (uint32_t i = 0; i < sizeof(block); i += 2) {
        uint16_t half = *(uint16_t*)(FOC_PARAMS_FLASH_ADDR + i);
        raw[i]   = half & 0xFF;
        raw[i+1] = (half >> 8) & 0xFF;
    }

    uint32_t calc_crc = crc32_compute((uint8_t*)&block.params, sizeof(foc_params_t));

    if (calc_crc == block.crc) {
        /* 新版 CRC 匹配 → 直接加载 */
        memcpy(p, &block.params, sizeof(foc_params_t));
        FOC_DBG("Load OK.\n");
        return 1;
    }

    /* 新版 CRC 不匹配 → 尝试 v3 格式（含 position PID，不含 speed outMax） */
    {
        uint32_t v3_params_size = FOC_PARAMS_V3_SIZE;

        uint32_t v3_crc = *(uint32_t*)(FOC_PARAMS_FLASH_ADDR + v3_params_size);
        uint32_t v3_calc = crc32_compute((uint8_t*)&block.params, v3_params_size);

        if (v3_calc == v3_crc) {
            foc_params_set_defaults(p);
            memcpy(p, &block.params, v3_params_size);
            FOC_DBG("v3 params loaded (speed output limit default).\n");
            return 1;
        }
    }

    /* 尝试 v2 格式（含 speed PID，不含 position PID） */
    {
        uint32_t v2_params_size = FOC_PARAMS_V2_SIZE;

        uint32_t v2_crc = *(uint32_t*)(FOC_PARAMS_FLASH_ADDR + v2_params_size);
        uint32_t v2_calc = crc32_compute((uint8_t*)&block.params, v2_params_size);

        if (v2_calc == v2_crc) {
            foc_params_set_defaults(p);
            memcpy(p, &block.params, v2_params_size);
            FOC_DBG("v2 params loaded (position PID defaults).\n");
            return 1;
        }
    }

    /* 尝试 v1 格式（不含 speed/position PID 字段） */
    {
        uint32_t v1_params_size = FOC_PARAMS_V1_SIZE;

        /* v1 CRC 紧跟在 v1 参数体之后 */
        uint32_t v1_crc = *(uint32_t*)(FOC_PARAMS_FLASH_ADDR + v1_params_size);
        uint32_t v1_calc = crc32_compute((uint8_t*)&block.params, v1_params_size);

        if (v1_calc == v1_crc) {
            /* v1 格式有效：迁移旧参数 + PID 用默认值 */
            foc_params_set_defaults(p);
            memcpy(p, &block.params, v1_params_size);  /* 覆盖旧字段 */
            FOC_DBG("v1 params loaded (PID defaults).\n");
            return 1;
        }
    }

    /* 尝试更旧的 legacy 格式 */
    {
        foc_params_legacy_block_t legacy_block;
        uint8_t *legacy_raw = (uint8_t*)&legacy_block;
        for (uint32_t i = 0; i < sizeof(legacy_block); i += 2) {
            uint16_t half = *(uint16_t*)(FOC_PARAMS_FLASH_ADDR + i);
            legacy_raw[i]   = half & 0xFF;
            legacy_raw[i+1] = (half >> 8) & 0xFF;
        }

        uint32_t legacy_crc = crc32_compute(
            (uint8_t*)&legacy_block.params, sizeof(foc_params_legacy_t));

        if (legacy_crc == legacy_block.crc) {
            foc_params_set_defaults(p);
            p->elec_offset = legacy_block.params.elec_offset;
            p->pole_pairs  = legacy_block.params.pole_pairs;
            p->reserved1   = legacy_block.params.reserved1;
            p->dir         = legacy_block.params.dir;
            p->speeddir    = legacy_block.params.speeddir;
            FOC_DBG("Legacy params loaded, motor Rs/Lq/Ld use defaults.\n");
            return 1;
        }
    }

    /* 全部格式都不匹配 → 填默认值 */
    foc_params_set_defaults(p);
    FOC_DBG("CRC ERROR! Flash data invalid, using defaults.\n");
    return 0;
}

/******************************************************************************
 * 函数名称：foc_params_test
 * 功能描述：测试函数，用于验证 Flash 读写和 CRC 校验功能。
 ******************************************************************************/
void foc_params_test(void)
{
    printf("sizeof(foc_params_t) = %lu\n", (uint32_t)sizeof(foc_params_t));

    g_params.elec_offset = 66.6f;
    g_params.pole_pairs  = 11;
    g_params.reserved1   = 0.123f;
    g_params.dir         = 1;
    g_params.speeddir    = 1;
    g_params.rs          = 0.198f;
    g_params.lq          = 0.000074f;
    g_params.ld          = 0.000040f;
    g_params.speed_pid_kp = 0.002f;
    g_params.speed_pid_ki = 0.1f;
    g_params.speed_pid_out = 10.0f;
    g_params.position_pid_kp = 1.0f;
    g_params.position_pid_kd = 0.0001f;
    g_params.position_pid_out = 400.0f;

    foc_params_save(&g_params);

    printf("Try loading...\n");
    if (!foc_params_load(&g_readback)) {
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
    printf("  ld           = %.6f\n", g_readback.ld);
    printf("  speed_pid_kp = %.6f\n", g_readback.speed_pid_kp);
    printf("  speed_pid_ki = %.6f\n", g_readback.speed_pid_ki);
    printf("  speed_pid_out = %.6f\n", g_readback.speed_pid_out);
    printf("  position_pid_kp  = %.6f\n", g_readback.position_pid_kp);
    printf("  position_pid_kd  = %.6f\n", g_readback.position_pid_kd);
    printf("  position_pid_out = %.6f\n", g_readback.position_pid_out);
}
