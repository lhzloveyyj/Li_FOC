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

    // ... 未来可以继续扩展
} CMD_TypeDef;

extern uint8_t g_Commcmd ;

#endif
