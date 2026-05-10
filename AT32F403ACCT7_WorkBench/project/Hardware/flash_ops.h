#ifndef __FLASH_OPS_H__
#define __FLASH_OPS_H__

#include <stdint.h>

/**
 * @brief Flash 参数存储模块
 *
 * 在 Flash 的固定扇区中保存电机标定参数和控制参数，
 * 确保掉电不丢失。
 */

/* Flash 存储地址（需确保该扇区不被程序代码占用） */
#define FOC_PARAMS_FLASH_ADDR   0x0803F800

/* =================== 参数结构体（1 字节对齐） =================== */
#pragma pack(push,1)
typedef struct {
    float    elec_offset;      /**< 零电角度偏移（单位：rad） */
    int32_t  pole_pairs;       /**< 电机极对数 */
    float    reserved1;        /**< 保留字段 */
    int32_t  dir;              /**< 旋转方向（1 或 -1） */
    int32_t  speeddir;         /**< 速度方向 */
    /* SMO/电机模型参数 */
    float    rs;               /**< 相电阻（单位：Ω） */
    float    lq;               /**< q 轴电感（单位：H） */
    float    ld;               /**< d 轴电感（单位：H） */
    /* 速度环 PID 参数 */
    float    speed_pid_kp;     /**< 速度环 Kp */
    float    speed_pid_ki;     /**< 速度环 Ki */
    /* 位置环 PID 参数 */
    float    position_pid_kp;  /**< 位置环 Kp */
    float    position_pid_kd;  /**< 位置环 Kd */
    float    position_pid_out; /**< 位置环输出限幅 */
    float    speed_pid_out;    /**< 速度环输出限幅 */
} foc_params_t;
#pragma pack(pop)

/* 旧版结构体大小，用于兼容升级 */
#define FOC_PARAMS_V1_SIZE  32  /* 不含速度/位置 PID */
#define FOC_PARAMS_V2_SIZE  40  /* 含速度 PID，不含位置 PID */
#define FOC_PARAMS_V3_SIZE  52  /* 含速度 PID 和位置 PID，不含速度输出限幅 */
#pragma pack(push,1)
typedef struct {
    float    elec_offset;
    int32_t  pole_pairs;
    float    reserved1;
    int32_t  dir;
    int32_t  speeddir;
    float    rs;
    float    lq;
    float    ld;
} foc_params_v1_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
    float    elec_offset;
    int32_t  pole_pairs;
    float    reserved1;
    int32_t  dir;
    int32_t  speeddir;
    float    rs;
    float    lq;
    float    ld;
    float    speed_pid_kp;
    float    speed_pid_ki;
} foc_params_v2_t;
#pragma pack(pop)

/* =================== 旧版结构体（兼容旧固件升级） =================== */
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

/* 参数块（参数 + CRC32 校验） */
#pragma pack(push,1)
typedef struct {
    foc_params_t params;
    uint32_t crc;
} foc_params_block_t;
#pragma pack(pop)

/* =================== 外部变量 =================== */
extern foc_params_t g_params;
extern foc_params_t g_readback;

/* =================== 函数声明 =================== */
void foc_params_save(foc_params_t *p);
int  foc_params_load(foc_params_t *p);
void foc_params_set_defaults(foc_params_t *p);
void foc_params_test(void);
uint32_t crc32_compute(const uint8_t *data, uint32_t len);

#endif
