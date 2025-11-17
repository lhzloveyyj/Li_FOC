#include "flash_ops.h"
#include "flash.h"
#include <string.h>
#include <stdio.h>

/* 调试打印开关：1开启，0关闭 */
#define FOC_FLASH_DEBUG  1

#if FOC_FLASH_DEBUG
    #define FOC_DBG_PRINTF(...)  printf(__VA_ARGS__)
#else
    #define FOC_DBG_PRINTF(...)  do{}while(0)
#endif

foc_params_t g_readback;
foc_params_t g_params;

void foc_params_save(foc_params_t *p)
{
    uint8_t buf[sizeof(foc_params_t)];
    memcpy(buf, p, sizeof(foc_params_t));

    flash_unlock();
    if(flash_sector_erase(FOC_PARAMS_FLASH_ADDR) != FLASH_OPERATE_DONE)
    {
        FOC_DBG_PRINTF("Flash erase ERROR!\r\n");
        flash_lock();
        return;
    }

    FOC_DBG_PRINTF("Writing Flash:\r\n");
    for(uint32_t i = 0; i < sizeof(foc_params_t); i += 2)
    {
        uint16_t half = buf[i] | (buf[i+1] << 8);
        flash_halfword_program(FOC_PARAMS_FLASH_ADDR + i, half);
        FOC_DBG_PRINTF("  Addr 0x%08X : 0x%02X 0x%02X -> 0x%04X\r\n",
                       FOC_PARAMS_FLASH_ADDR + i, buf[i], buf[i+1], half);
    }
    FOC_DBG_PRINTF("\r\n");

    flash_lock();
}

void foc_params_load(foc_params_t *p)
{
    uint8_t buf[sizeof(foc_params_t)];
    FOC_DBG_PRINTF("Reading Flash:\r\n");
    for(uint32_t i = 0; i < sizeof(foc_params_t); i += 2)
    {
        uint16_t half = *(uint16_t*)(FOC_PARAMS_FLASH_ADDR + i);
        buf[i]   = half & 0xFF;
        buf[i+1] = (half >> 8) & 0xFF;
        FOC_DBG_PRINTF("  Addr 0x%08X : 0x%04X -> 0x%02X 0x%02X\r\n",
                       FOC_PARAMS_FLASH_ADDR + i, half, buf[i], buf[i+1]);
    }
    FOC_DBG_PRINTF("\r\n");
    memcpy(p, buf, sizeof(foc_params_t));
}

/* 调试测试函数 */
void foc_params_test(void)
{

    /* 填充测试数据 */
    g_params.elec_offset = 66.6f;
    g_params.pole_pairs  = 11;
    g_params.reserved1   = 0.123;
    g_params.dir   = 1;

    foc_params_save(&g_params);
    FOC_DBG_PRINTF("Flash write OK!\r\n");

    foc_params_load(&g_readback);
    FOC_DBG_PRINTF("Readback struct:\r\n");
    FOC_DBG_PRINTF("  offset = %.3f\r\n", g_readback.elec_offset);
    FOC_DBG_PRINTF("  pole   = %d\r\n", g_readback.pole_pairs);
    FOC_DBG_PRINTF("  res1   = %f\r\n", g_readback.reserved1);
    FOC_DBG_PRINTF("  dir   = %d\r\n", g_readback.dir);
    FOC_DBG_PRINTF("\r\n");

    /* 可选：打印每个半字 */
    uint16_t *p_half = (uint16_t*)&g_readback;
    FOC_DBG_PRINTF("Flash halfword readback:\r\n");
    for(uint32_t i=0;i<PARAM_HALFWORDS;i++)
        printf("0x%04X\r\n ", p_half[i]);
    FOC_DBG_PRINTF("\n");
}
