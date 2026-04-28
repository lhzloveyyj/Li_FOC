#ifndef __USART3_H
#define __USART3_H

#include "at32f403a_407.h"
#include "at32f403a_407_usart.h"
#include "freertos_app.h"

/* 调试打印串口 */
#define PRINT_UART                       USART3

/* 发送缓冲区大小 */
#define USART3_TX_BUFFER_SIZE       128

/**
 * @brief 通用数据帧结构（仅用于帧格式说明，实际发送用字节流组装）
 */
typedef struct __attribute__((packed))
{
    uint8_t header;      // 帧头 (0xA5)
    uint8_t cmd;         // 命令字
    uint8_t length;      // 数据长度
    uint8_t payload[64]; // 数据区
    uint8_t checksum;    // 校验和
    uint8_t tail;        // 帧尾 (0x49)
} Frame_t;

/* =================== 外部变量 =================== */
extern uint8_t uart3_tx_buffer[USART3_TX_BUFFER_SIZE];
extern volatile uint8_t usart3_tx_dma_status;
extern SemaphoreHandle_t usart3_dma_tx_sem;

extern uint8_t g_commCmd;
extern float g_cmdData;

/* =================== 函数声明 =================== */
void USART3_SendPacket(uint8_t cmd, float *values, uint8_t count);
void USART3_ParseFixedCommand(uint8_t byte);

#endif
