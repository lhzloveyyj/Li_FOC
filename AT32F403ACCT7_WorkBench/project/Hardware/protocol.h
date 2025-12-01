#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>

// ======================= 帧结构定义 ======================= //
#define FRAME_HEAD   0xA5
#define FRAME_TAIL   0x49

// ======================= 命令字定义 ======================= //
typedef enum
{
    CMD_NONE                    = 0x00,  // 无效命令
    CMD_CONNECT_MOTOR           = 0x01,  // 连接电机
    CMD_MECHANICALANGLE         = 0x02,  // 打开机械角度传输
    CMD_MECHANICALANGLE_CLOSE   = 0X03,  // 关闭机械角度传输
    CMD_SETPAIRS                = 0x04,  // 设置极对数
    CMD_SETDIR                  = 0x05,  // 设置方向
    CMD_ZEROCALIBRATIO          = 0x06,  // 零点校准
    CMD_ZEROCALIBRATIO_OVER     = 0x07,  // 零点校准结束
    CMD_UABC                    = 0x08,  // 打印三项电压ua,ub,uc
    CMD_UABC_CLOSE              = 0x09,  // 关闭打印三项电压ua,ub,uc
    CMD_SETUQ                   = 0x0A,  // 设置Uq
    CMD_ADC                     = 0x0B,  // 打印三项ADC采样值
    CMD_ADC_CLOSE               = 0x0C,  // 关闭打印三项ADC采样值
    CMD_DCVBUS                  = 0x0D,  // 获取母线电压
    CMD_TABC                    = 0x0E,  // 三相SVPWM输出
    CMD_TABC_CLOSE              = 0x0F,  // 关闭三相SVPWM输出
    CMD_IABC                    = 0x10,  // 打印三相电流
    CMD_IABC_CLOSE              = 0x11,  // 关闭打印三相电流
    CMD_UALPHA_BETA             = 0x12,  // 打印 uAlpha, uBeta
    CMD_UALPHA_BETA_CLOSE       = 0x13,  // 关闭打印 uAlpha, uBeta
    CMD_IALPHA_BETA             = 0x14,  // 打印 IAlpha, uBeta
    CMD_IALPHA_BETA_CLOSE       = 0x15,  // 关闭打印 IAlpha, uBeta
    CMD_IQ_ID                   = 0x16,  // 打印 IQ,ID
    CMD_IQ_ID_CLOSE             = 0x17,  // 关闭打印 IQ,ID
    CMD_SETIQ                   = 0x18,  // 设置IQ
    CMD_SETID                   = 0x19,  // 设置ID
    CMD_OPEN_LOOP               = 0x20,  // 开环模式
    CMD_CURRENT_LOOP            = 0x21,  // 电流环
    CMD_SPEED_LOOP              = 0x22,  // 速度-电流环
    CMD_POSITION_LOOP              = 0x23,  // 位置-速度-电流环
    // ... 未来可以继续扩展
} CMD_TypeDef;

extern uint8_t g_Commcmd ;

#endif
