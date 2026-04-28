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
    /* SMO/电机模型参数：Rs 用 R/ohm，Lq/Ld 用 H。 */
    float    rs;            /* 4 */
    float    lq;            /* 4 */
    float    ld;            /* 4 */
} foc_params_t;
#pragma pack(pop)

/* 老版本 flash 结构没有 Rs/Lq/Ld。
 * 保留 legacy 结构是为了固件升级后还能读出旧板子的零偏、极对数和方向。
 */
#pragma pack(push,1)
typedef struct {
    float    elec_offset;
    int32_t  pole_pairs;
    float    reserved1;
    int32_t  dir;
    int32_t  speeddir;
} foc_params_legacy_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
    foc_params_legacy_t params;
    uint32_t crc;
} foc_params_legacy_block_t;
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
void foc_params_set_defaults(foc_params_t *p);
void foc_params_test(void);

/* CRC32 */
uint32_t crc32_compute(const uint8_t *data, uint32_t len);

#endif
