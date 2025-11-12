#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>

// ======================= 帧结构定义 ======================= //
#define FRAME_HEAD   0xA5
#define FRAME_TAIL   0x49

// ======================= 命令字定义 ======================= //
typedef enum
{
    CMD_NONE            = 0x00,  // 无效命令
    CMD_CONNECT_MOTOR   = 0x01,  // 连接电机

    // ... 未来可以继续扩展
} CMD_TypeDef;

extern uint8_t g_Commcmd ;

#endif
