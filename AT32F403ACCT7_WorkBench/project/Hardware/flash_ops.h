#ifndef __FLASH_OPS_H__
#define __FLASH_OPS_H__

#include <stdint.h>

/* FOC 参数结构体 */
typedef struct {
    float    elec_offset;   /* 电角度偏移 */
    int    pole_pairs;    /* 极对数 */
    float reserved1;
    int dir;
    int speeddir;
} foc_params_t;

extern foc_params_t g_readback;
extern foc_params_t g_params;

/* Flash 参数存储地址 */
#define FOC_PARAMS_FLASH_ADDR   0x0803F800

/* 半字数，用于 Flash 写入/读取 */
#define PARAM_HALFWORDS  ((sizeof(foc_params_t)+1)/2)

/* API */
void foc_params_save(foc_params_t *p);
void foc_params_load(foc_params_t *p);
void foc_params_test(void);

#endif
