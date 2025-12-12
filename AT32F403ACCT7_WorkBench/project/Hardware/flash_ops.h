#ifndef __FLASH_OPS_H__
#define __FLASH_OPS_H__

#include <stdint.h>

/* Flash 存储地址 */
#define FOC_PARAMS_FLASH_ADDR   0x0803F800

/* ========== 参数结构体 ========== */
/* 强制 1 字节对齐，避免 padding */
#pragma pack(push,1)
typedef struct {
    float    elec_offset;   /* 4 */
    int32_t  pole_pairs;    /* 4 */
    float    reserved1;     /* 4 */
    int32_t  dir;           /* 4 */
    int32_t  speeddir;      /* 4 */
} foc_params_t;
#pragma pack(pop)

/* 参数 + CRC */
#pragma pack(push,1)
typedef struct {
    foc_params_t params;
    uint32_t crc;
} foc_params_block_t;
#pragma pack(pop)

/* 外部变量 */
extern foc_params_t g_params;
extern foc_params_t g_readback;

/* API */
void foc_params_save(foc_params_t *p);
int  foc_params_load(foc_params_t *p);
void foc_params_test(void);

/* CRC32 */
uint32_t crc32_compute(const uint8_t *data, uint32_t len);

#endif
