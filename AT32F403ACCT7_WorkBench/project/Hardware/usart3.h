#ifndef __USART3_H
#define __USART3_H

#include "at32f403a_407.h"  
#include "at32f403a_407_usart.h"

#define N								 3
#define USART3_TX_BUFFER_SIZE            (4 * N + 4)


typedef struct __attribute__((packed))
{
    uint8_t header;      // 帧头
    uint8_t cmd;         // 命令字
    uint8_t length;      // 数据长度
    uint8_t payload[32]; // 数据区
    uint8_t checksum;    // 校验
    uint8_t tail;        // 帧尾
} Frame_t;

extern uint8_t uart3_tx_buffer[USART3_TX_BUFFER_SIZE] ;
extern volatile uint8_t usart3_tx_dma_status;

void USART3_SendPacket(float *values, uint8_t count);

#endif
